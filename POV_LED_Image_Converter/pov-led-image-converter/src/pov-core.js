(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.POVCore = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const HEADER_BYTES = 48;
  const FORMAT_VERSION = 1;
  const ANGLE_MIN_DEG = -90;
  const ANGLE_MAX_DEG = 90;
  const ANGLE_SPAN_DEG = ANGLE_MAX_DEG - ANGLE_MIN_DEG;
  const FIXED_LED_COUNT = 35;
  const SK9822_START_BYTES = 4;
  const SK9822_BYTES_PER_LED = 4;
  const SK9822_END_BYTES = 4;
  const SK9822_ENCODING = 1;
  const WAND_FLAGS = 0x0007;

  const DEFAULT_CONFIG = Object.freeze({
    ledCount: FIXED_LED_COUNT,
    stripLengthCm: 25,
    pivotOffsetCm: 0,
    requestedStepDeg: 1,
    imageCenterXCm: 0,
    imageCenterYCm: 12.5,
    imageWidthCm: 22,
    imageHeightCm: 22,
    imageRotationDeg: 0,
    interpolation: "bilinear",
    rgbBrightnessPercent: 100,
    globalBrightness: 1,
  });

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function finiteNumber(value, fallback) {
    const number = Number(value);
    return Number.isFinite(number) ? number : fallback;
  }

  function frameBytesForLedCount(ledCount) {
    return SK9822_START_BYTES + ledCount * SK9822_BYTES_PER_LED + SK9822_END_BYTES;
  }

  function normalizeConfig(input) {
    const source = Object.assign({}, DEFAULT_CONFIG, input || {});
    const requestedStepDeg = clamp(finiteNumber(source.requestedStepDeg, 1), 0.1, 45);
    const intervalCount = clamp(Math.round(ANGLE_SPAN_DEG / requestedStepDeg), 4, 1800);
    const actualStepDeg = ANGLE_SPAN_DEG / intervalCount;
    const stripLengthCm = clamp(finiteNumber(source.stripLengthCm, 25), 0, 1000);
    const ledPitchCm = stripLengthCm / (FIXED_LED_COUNT - 1);

    return {
      ledCount: FIXED_LED_COUNT,
      stripLengthCm,
      pivotOffsetCm: clamp(finiteNumber(source.pivotOffsetCm, 0), -1000, 1000),
      requestedStepDeg,
      actualStepDeg,
      sampleCount: intervalCount + 1,
      ledPitchCm,
      frameBytes: frameBytesForLedCount(FIXED_LED_COUNT),
      minAngleDeg: ANGLE_MIN_DEG,
      maxAngleDeg: ANGLE_MAX_DEG,
      imageCenterXCm: clamp(finiteNumber(source.imageCenterXCm, 0), -10000, 10000),
      imageCenterYCm: clamp(finiteNumber(source.imageCenterYCm, 12.5), -10000, 10000),
      imageWidthCm: clamp(finiteNumber(source.imageWidthCm, 22), 0.001, 10000),
      imageHeightCm: clamp(finiteNumber(source.imageHeightCm, 22), 0.001, 10000),
      imageRotationDeg: finiteNumber(source.imageRotationDeg, 0),
      interpolation: source.interpolation === "nearest" ? "nearest" : "bilinear",
      rgbBrightnessPercent: clamp(finiteNumber(source.rgbBrightnessPercent, 100), 0, 100),
      globalBrightness: clamp(Math.round(finiteNumber(source.globalBrightness, 1)), 0, 31),
    };
  }

  function radiusForLed(config, ledIndex) {
    return config.pivotOffsetCm + config.ledPitchCm * ledIndex;
  }

  function angleForSample(config, sampleIndex) {
    return config.minAngleDeg + clamp(Math.round(sampleIndex), 0, config.sampleCount - 1) * config.actualStepDeg;
  }

  function sampleForAngle(config, angleDeg) {
    const clampedAngle = clamp(finiteNumber(angleDeg, 0), config.minAngleDeg, config.maxAngleDeg);
    return clamp(
      Math.round((clampedAngle - config.minAngleDeg) / config.actualStepDeg),
      0,
      config.sampleCount - 1,
    );
  }

  function ledWorldPosition(config, sampleIndex, ledIndex) {
    const mechanicalAngleDeg = angleForSample(config, sampleIndex);
    const angle = mechanicalAngleDeg * Math.PI / 180;
    const radius = radiusForLed(config, ledIndex);
    return {
      x: radius * Math.sin(angle),
      y: radius * Math.cos(angle),
      radius,
      mechanicalAngleDeg,
    };
  }

  function worldToImageUv(config, worldX, worldY) {
    const dx = worldX - config.imageCenterXCm;
    const dy = worldY - config.imageCenterYCm;
    const rotation = config.imageRotationDeg * Math.PI / 180;
    const cosine = Math.cos(rotation);
    const sine = Math.sin(rotation);
    const localX = cosine * dx - sine * dy;
    const localY = sine * dx + cosine * dy;
    return {
      u: localX / config.imageWidthCm + 0.5,
      v: 0.5 - localY / config.imageHeightCm,
    };
  }

  function pixelOffset(width, x, y) {
    return (y * width + x) * 4;
  }

  function readCompositedPixel(image, x, y) {
    const offset = pixelOffset(image.width, x, y);
    const alpha = image.data[offset + 3] / 255;
    return [
      image.data[offset] * alpha,
      image.data[offset + 1] * alpha,
      image.data[offset + 2] * alpha,
    ];
  }

  function sampleImage(image, u, v, interpolation) {
    if (!image || !image.data || image.width < 1 || image.height < 1) return [0, 0, 0];
    if (u < 0 || u > 1 || v < 0 || v > 1) return [0, 0, 0];

    const x = u * (image.width - 1);
    const y = v * (image.height - 1);
    if (interpolation === "nearest") {
      return readCompositedPixel(image, Math.round(x), Math.round(y));
    }

    const x0 = Math.floor(x);
    const y0 = Math.floor(y);
    const x1 = Math.min(image.width - 1, x0 + 1);
    const y1 = Math.min(image.height - 1, y0 + 1);
    const tx = x - x0;
    const ty = y - y0;
    const p00 = readCompositedPixel(image, x0, y0);
    const p10 = readCompositedPixel(image, x1, y0);
    const p01 = readCompositedPixel(image, x0, y1);
    const p11 = readCompositedPixel(image, x1, y1);
    const output = [0, 0, 0];
    for (let channel = 0; channel < 3; channel += 1) {
      const top = p00[channel] + (p10[channel] - p00[channel]) * tx;
      const bottom = p01[channel] + (p11[channel] - p01[channel]) * tx;
      output[channel] = top + (bottom - top) * ty;
    }
    return output;
  }

  function convertImage(image, inputConfig) {
    const config = normalizeConfig(inputConfig);
    const payload = new Uint8Array(config.sampleCount * config.frameBytes);
    const rgbScale = config.rgbBrightnessPercent / 100;
    let litSamples = 0;

    for (let sample = 0; sample < config.sampleCount; sample += 1) {
      const frameOffset = sample * config.frameBytes;
      payload.fill(0x00, frameOffset, frameOffset + SK9822_START_BYTES);

      for (let led = 0; led < config.ledCount; led += 1) {
        const world = ledWorldPosition(config, sample, led);
        const uv = worldToImageUv(config, world.x, world.y);
        const rgb = sampleImage(image, uv.u, uv.v, config.interpolation);
        const red = Math.round(clamp(rgb[0] * rgbScale, 0, 255));
        const green = Math.round(clamp(rgb[1] * rgbScale, 0, 255));
        const blue = Math.round(clamp(rgb[2] * rgbScale, 0, 255));
        const output = frameOffset + SK9822_START_BYTES + led * SK9822_BYTES_PER_LED;

        payload[output] = 0xE0 | config.globalBrightness;
        payload[output + 1] = green;
        payload[output + 2] = red;
        payload[output + 3] = blue;
        if (red || green || blue) litSamples += 1;
      }

      payload.fill(0xFF, frameOffset + config.frameBytes - SK9822_END_BYTES, frameOffset + config.frameBytes);
    }

    return {
      config,
      payload,
      litSamples,
      totalLedSamples: config.sampleCount * config.ledCount,
    };
  }

  function crc32(bytes, start, end) {
    let crc = 0xFFFFFFFF;
    const from = start == null ? 0 : start;
    const to = end == null ? bytes.length : end;
    for (let index = from; index < to; index += 1) {
      crc ^= bytes[index];
      for (let bit = 0; bit < 8; bit += 1) {
        crc = (crc >>> 1) ^ (0xEDB88320 & -(crc & 1));
      }
    }
    return (crc ^ 0xFFFFFFFF) >>> 0;
  }

  function buildWandBinary(conversion) {
    if (!conversion || !conversion.config || !conversion.payload) {
      throw new Error("A completed conversion is required.");
    }
    const config = conversion.config;
    const payload = conversion.payload;
    const bytes = new Uint8Array(HEADER_BYTES + payload.length);
    const view = new DataView(bytes.buffer);

    bytes.set([0x57, 0x41, 0x4E, 0x44], 0); // WAND
    view.setUint16(4, FORMAT_VERSION, true);
    view.setUint16(6, HEADER_BYTES, true);
    view.setUint16(8, config.ledCount, true);
    view.setUint16(10, config.frameBytes, true);
    view.setUint32(12, config.sampleCount, true);
    view.setInt32(16, Math.round(config.minAngleDeg * 1000), true);
    view.setInt32(20, Math.round(config.maxAngleDeg * 1000), true);
    view.setUint32(24, Math.round(config.actualStepDeg * 1000000), true);
    view.setUint32(28, payload.length, true);
    view.setUint32(32, crc32(payload), true);
    view.setUint8(36, SK9822_ENCODING);
    view.setUint8(37, 0);
    view.setUint16(38, WAND_FLAGS, true);
    view.setUint32(40, bytes.length, true);
    view.setUint32(44, crc32(bytes, 0, 44), true);
    bytes.set(payload, HEADER_BYTES);
    return bytes;
  }

  function parseWandBinary(input) {
    const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
    if (bytes.length < HEADER_BYTES) throw new Error("File is shorter than the 48-byte WAND1 header.");
    if (bytes[0] !== 0x57 || bytes[1] !== 0x41 || bytes[2] !== 0x4E || bytes[3] !== 0x44) {
      throw new Error("Invalid magic; expected WAND.");
    }

    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const headerBytes = view.getUint16(6, true);
    const payloadBytes = view.getUint32(28, true);
    const fileBytes = view.getUint32(40, true);
    if (headerBytes !== HEADER_BYTES) throw new Error("Unsupported WAND1 header size.");
    if (bytes.length !== headerBytes + payloadBytes || bytes.length !== fileBytes) {
      throw new Error("File length does not match its header.");
    }

    const payload = bytes.subarray(headerBytes);
    const headerCrcExpected = view.getUint32(44, true);
    const payloadCrcExpected = view.getUint32(32, true);
    return {
      version: view.getUint16(4, true),
      headerBytes,
      ledCount: view.getUint16(8, true),
      frameBytes: view.getUint16(10, true),
      sampleCount: view.getUint32(12, true),
      minAngleMdeg: view.getInt32(16, true),
      maxAngleMdeg: view.getInt32(20, true),
      angleStepUdeg: view.getUint32(24, true),
      payloadBytes,
      payloadCrc32: payloadCrcExpected,
      encoding: view.getUint8(36),
      flags: view.getUint16(38, true),
      fileBytes,
      headerCrc32: headerCrcExpected,
      headerCrcValid: crc32(bytes, 0, 44) === headerCrcExpected,
      payloadCrcValid: crc32(payload) === payloadCrcExpected,
      payload,
    };
  }

  function frameWireBytes(conversion, sampleIndex) {
    const sample = clamp(Math.round(sampleIndex), 0, conversion.config.sampleCount - 1);
    const offset = sample * conversion.config.frameBytes;
    return conversion.payload.subarray(offset, offset + conversion.config.frameBytes);
  }

  function frameRgb(conversion, sampleIndex, ledIndex) {
    const config = conversion.config;
    const sample = clamp(Math.round(sampleIndex), 0, config.sampleCount - 1);
    const led = clamp(Math.round(ledIndex), 0, config.ledCount - 1);
    const offset = sample * config.frameBytes + SK9822_START_BYTES + led * SK9822_BYTES_PER_LED;
    return [conversion.payload[offset + 2], conversion.payload[offset + 1], conversion.payload[offset + 3]];
  }

  function bytesToHex(bytes) {
    return Array.from(bytes, (value) => value.toString(16).padStart(2, "0").toUpperCase()).join("");
  }

  function buildJsonExport(conversion, sourceName, motion) {
    const config = conversion.config;
    const frames = [];
    for (let sample = 0; sample < config.sampleCount; sample += 1) {
      const frame = frameWireBytes(conversion, sample);
      const leds = [];
      for (let led = 0; led < config.ledCount; led += 1) {
        const offset = SK9822_START_BYTES + led * SK9822_BYTES_PER_LED;
        const command = frame.subarray(offset, offset + SK9822_BYTES_PER_LED);
        leds.push({
          led,
          global_brightness: command[0] & 0x1F,
          rgb: [command[2], command[1], command[3]],
          sk9822_bytes_hex: bytesToHex(command),
        });
      }
      frames.push({
        angle_deg: Number(angleForSample(config, sample).toFixed(6)),
        wire_frame_hex: bytesToHex(frame),
        leds,
      });
    }

    return {
      format: "WAND1-JSON",
      version: FORMAT_VERSION,
      source_image: sourceName || "unknown",
      binary_payload: "Complete SK9822 frames: 00000000 + 35 x [111BBBBB,G,R,B] + FFFFFFFF",
      geometry: {
        led_count: config.ledCount,
        strip_length_cm: config.stripLengthCm,
        pivot_to_led0_cm: config.pivotOffsetCm,
        led_pitch_cm: config.ledPitchCm,
        image_center_cm: [config.imageCenterXCm, config.imageCenterYCm],
        image_size_cm: [config.imageWidthCm, config.imageHeightCm],
        image_rotation_deg_clockwise: config.imageRotationDeg,
      },
      sampling: {
        angle_convention: "0 degrees upright; negative counterclockwise; positive clockwise",
        min_angle_deg: config.minAngleDeg,
        max_angle_deg: config.maxAngleDeg,
        requested_step_deg: config.requestedStepDeg,
        actual_step_deg: config.actualStepDeg,
        sample_count: config.sampleCount,
        interpolation: config.interpolation,
        alpha_background: "black",
      },
      motion_estimate: motion || null,
      frames,
    };
  }

  return {
    HEADER_BYTES,
    FORMAT_VERSION,
    ANGLE_MIN_DEG,
    ANGLE_MAX_DEG,
    FIXED_LED_COUNT,
    SK9822_START_BYTES,
    SK9822_BYTES_PER_LED,
    SK9822_END_BYTES,
    SK9822_ENCODING,
    WAND_FLAGS,
    DEFAULT_CONFIG,
    normalizeConfig,
    frameBytesForLedCount,
    radiusForLed,
    angleForSample,
    sampleForAngle,
    ledWorldPosition,
    worldToImageUv,
    sampleImage,
    convertImage,
    crc32,
    buildWandBinary,
    parseWandBinary,
    frameWireBytes,
    frameRgb,
    buildJsonExport,
  };
});

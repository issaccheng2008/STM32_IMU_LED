(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.POVCore = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const HEADER_BYTES = 80;
  const FULL_TURN_UDEG = 360000000;

  const DEFAULT_CONFIG = Object.freeze({
    ledCount: 35,
    stripLengthCm: 25,
    pivotOffsetCm: 0,
    requestedStepDeg: 1,
    imageCenterXCm: 0,
    imageCenterYCm: 12.5,
    imageWidthCm: 22,
    imageHeightCm: 22,
    imageRotationDeg: 0,
    zeroAngleDeg: 0,
    clockwise: true,
    interpolation: "bilinear",
    rgbBrightnessPercent: 100,
    globalBrightness: 1,
  });

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function finiteNumber(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function normalizeConfig(input) {
    const source = Object.assign({}, DEFAULT_CONFIG, input || {});
    const requestedStepDeg = clamp(finiteNumber(source.requestedStepDeg, 1), 0.05, 45);
    const sampleCount = clamp(Math.round(360 / requestedStepDeg), 8, 7200);
    const actualStepDeg = 360 / sampleCount;
    const ledCount = clamp(Math.round(finiteNumber(source.ledCount, 35)), 1, 1024);
    const stripLengthCm = clamp(finiteNumber(source.stripLengthCm, 25), 0, 1000);
    const ledPitchCm = ledCount > 1 ? stripLengthCm / (ledCount - 1) : 0;

    return {
      ledCount,
      stripLengthCm,
      pivotOffsetCm: clamp(finiteNumber(source.pivotOffsetCm, 0), -1000, 1000),
      requestedStepDeg,
      actualStepDeg,
      sampleCount,
      ledPitchCm,
      imageCenterXCm: clamp(finiteNumber(source.imageCenterXCm, 0), -10000, 10000),
      imageCenterYCm: clamp(finiteNumber(source.imageCenterYCm, 12.5), -10000, 10000),
      imageWidthCm: clamp(finiteNumber(source.imageWidthCm, 22), 0.001, 10000),
      imageHeightCm: clamp(finiteNumber(source.imageHeightCm, 22), 0.001, 10000),
      imageRotationDeg: finiteNumber(source.imageRotationDeg, 0),
      zeroAngleDeg: finiteNumber(source.zeroAngleDeg, 0),
      clockwise: source.clockwise !== false && source.clockwise !== "false",
      interpolation: source.interpolation === "nearest" ? "nearest" : "bilinear",
      rgbBrightnessPercent: clamp(finiteNumber(source.rgbBrightnessPercent, 100), 0, 100),
      globalBrightness: clamp(Math.round(finiteNumber(source.globalBrightness, 1)), 0, 31),
    };
  }

  function radiusForLed(config, ledIndex) {
    return config.pivotOffsetCm + config.ledPitchCm * ledIndex;
  }

  function ledWorldPosition(config, sampleIndex, ledIndex) {
    const mechanicalAngleDeg = sampleIndex * config.actualStepDeg;
    const signedAngleDeg = config.zeroAngleDeg + (config.clockwise ? mechanicalAngleDeg : -mechanicalAngleDeg);
    const angle = signedAngleDeg * Math.PI / 180;
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
    const c = Math.cos(rotation);
    const s = Math.sin(rotation);
    const localX = c * dx - s * dy;
    const localY = s * dx + c * dy;
    return {
      u: localX / config.imageWidthCm + 0.5,
      v: 0.5 - localY / config.imageHeightCm,
    };
  }

  function pixelOffset(width, x, y) {
    return (y * width + x) * 4;
  }

  function readCompositedPixel(image, x, y) {
    const i = pixelOffset(image.width, x, y);
    const alpha = image.data[i + 3] / 255;
    return [
      image.data[i] * alpha,
      image.data[i + 1] * alpha,
      image.data[i + 2] * alpha,
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
    const out = [0, 0, 0];
    for (let channel = 0; channel < 3; channel += 1) {
      const top = p00[channel] + (p10[channel] - p00[channel]) * tx;
      const bottom = p01[channel] + (p11[channel] - p01[channel]) * tx;
      out[channel] = top + (bottom - top) * ty;
    }
    return out;
  }

  function convertImage(image, inputConfig) {
    const config = normalizeConfig(inputConfig);
    const payload = new Uint8Array(config.sampleCount * config.ledCount * 3);
    const brightness = config.rgbBrightnessPercent / 100;
    let litSamples = 0;

    for (let sample = 0; sample < config.sampleCount; sample += 1) {
      for (let led = 0; led < config.ledCount; led += 1) {
        const world = ledWorldPosition(config, sample, led);
        const uv = worldToImageUv(config, world.x, world.y);
        const rgb = sampleImage(image, uv.u, uv.v, config.interpolation);
        const offset = (sample * config.ledCount + led) * 3;
        payload[offset] = Math.round(clamp(rgb[0] * brightness, 0, 255));
        payload[offset + 1] = Math.round(clamp(rgb[1] * brightness, 0, 255));
        payload[offset + 2] = Math.round(clamp(rgb[2] * brightness, 0, 255));
        if (payload[offset] || payload[offset + 1] || payload[offset + 2]) litSamples += 1;
      }
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
    for (let i = from; i < to; i += 1) {
      crc ^= bytes[i];
      for (let bit = 0; bit < 8; bit += 1) {
        crc = (crc >>> 1) ^ (0xEDB88320 & -(crc & 1));
      }
    }
    return (crc ^ 0xFFFFFFFF) >>> 0;
  }

  function toMicrometres(cm) {
    return Math.round(cm * 10000);
  }

  function buildPovBinary(conversion) {
    if (!conversion || !conversion.config || !conversion.payload) {
      throw new Error("A completed conversion is required.");
    }
    const config = conversion.config;
    const payload = conversion.payload;
    const bytes = new Uint8Array(HEADER_BYTES + payload.length);
    const view = new DataView(bytes.buffer);
    bytes.set([0x50, 0x4F, 0x56, 0x31], 0); // POV1
    view.setUint16(4, 1, true);
    view.setUint16(6, HEADER_BYTES, true);
    view.setUint16(8, config.ledCount, true);
    view.setUint8(10, 3); // channels
    view.setUint8(11, 0); // raw RGB encoding
    view.setUint32(12, config.sampleCount, true);
    view.setUint32(16, Math.round(config.actualStepDeg * 1000000), true);
    view.setUint32(20, payload.length, true);
    view.setUint32(24, crc32(payload), true);
    view.setInt32(28, toMicrometres(config.stripLengthCm), true);
    view.setInt32(32, toMicrometres(config.pivotOffsetCm), true);
    view.setInt32(36, toMicrometres(config.imageCenterXCm), true);
    view.setInt32(40, toMicrometres(config.imageCenterYCm), true);
    view.setUint32(44, toMicrometres(config.imageWidthCm), true);
    view.setUint32(48, toMicrometres(config.imageHeightCm), true);
    view.setInt32(52, Math.round(config.imageRotationDeg * 1000), true);
    view.setInt32(56, Math.round(config.zeroAngleDeg * 1000), true);
    let flags = 0;
    if (config.clockwise) flags |= 1;
    if (config.interpolation === "bilinear") flags |= 2;
    flags |= 4; // RGB scale already applied
    flags |= 8; // alpha composited over black
    view.setUint32(60, flags, true);
    view.setUint32(64, toMicrometres(config.ledPitchCm), true);
    view.setInt32(68, toMicrometres(config.pivotOffsetCm), true);
    view.setUint8(72, config.globalBrightness);
    view.setUint8(73, 0); // payload channel order: RGB
    view.setUint16(74, 0, true);
    view.setUint32(76, crc32(bytes, 0, 76), true);
    bytes.set(payload, HEADER_BYTES);
    return bytes;
  }

  function parsePovBinary(input) {
    const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
    if (bytes.length < HEADER_BYTES) throw new Error("File is shorter than the 80-byte POV1 header.");
    if (bytes[0] !== 0x50 || bytes[1] !== 0x4F || bytes[2] !== 0x56 || bytes[3] !== 0x31) {
      throw new Error("Invalid magic; expected POV1.");
    }
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const headerBytes = view.getUint16(6, true);
    const payloadBytes = view.getUint32(20, true);
    if (headerBytes !== HEADER_BYTES) throw new Error("Unsupported POV1 header size.");
    if (bytes.length !== headerBytes + payloadBytes) throw new Error("File length does not match its header.");
    const payload = bytes.subarray(headerBytes);
    const headerCrcExpected = view.getUint32(76, true);
    const payloadCrcExpected = view.getUint32(24, true);
    return {
      version: view.getUint16(4, true),
      headerBytes,
      ledCount: view.getUint16(8, true),
      channels: view.getUint8(10),
      encoding: view.getUint8(11),
      sampleCount: view.getUint32(12, true),
      angleStepUdeg: view.getUint32(16, true),
      payloadBytes,
      payloadCrc32: payloadCrcExpected,
      headerCrc32: headerCrcExpected,
      headerCrcValid: crc32(bytes, 0, 76) === headerCrcExpected,
      payloadCrcValid: crc32(payload) === payloadCrcExpected,
      stripLengthUm: view.getInt32(28, true),
      pivotToLed0Um: view.getInt32(32, true),
      imageCenterXUm: view.getInt32(36, true),
      imageCenterYUm: view.getInt32(40, true),
      imageWidthUm: view.getUint32(44, true),
      imageHeightUm: view.getUint32(48, true),
      imageRotationMdeg: view.getInt32(52, true),
      zeroAngleMdeg: view.getInt32(56, true),
      flags: view.getUint32(60, true),
      ledPitchUm: view.getUint32(64, true),
      led0RadiusUm: view.getInt32(68, true),
      globalBrightness: view.getUint8(72),
      rgbOrder: view.getUint8(73),
      payload,
    };
  }

  function frameRgb(conversion, sampleIndex, ledIndex) {
    const config = conversion.config;
    const sample = ((Math.round(sampleIndex) % config.sampleCount) + config.sampleCount) % config.sampleCount;
    const led = clamp(Math.round(ledIndex), 0, config.ledCount - 1);
    const offset = (sample * config.ledCount + led) * 3;
    return [conversion.payload[offset], conversion.payload[offset + 1], conversion.payload[offset + 2]];
  }

  function buildJsonExport(conversion, sourceName) {
    const config = conversion.config;
    const frames = [];
    for (let sample = 0; sample < config.sampleCount; sample += 1) {
      const leds = [];
      for (let led = 0; led < config.ledCount; led += 1) {
        leds.push(frameRgb(conversion, sample, led));
      }
      frames.push({ angle_deg: Number((sample * config.actualStepDeg).toFixed(6)), leds_rgb: leds });
    }
    return {
      format: "POV1-JSON",
      version: 1,
      source_image: sourceName || "unknown",
      geometry: {
        led_count: config.ledCount,
        strip_length_cm: config.stripLengthCm,
        pivot_to_led0_cm: config.pivotOffsetCm,
        led_pitch_cm: config.ledPitchCm,
        image_center_cm: [config.imageCenterXCm, config.imageCenterYCm],
        image_size_cm: [config.imageWidthCm, config.imageHeightCm],
        image_rotation_deg_clockwise: config.imageRotationDeg,
        zero_direction_deg_clockwise_from_up: config.zeroAngleDeg,
        increasing_angle_direction: config.clockwise ? "clockwise" : "counterclockwise",
      },
      sampling: {
        requested_step_deg: config.requestedStepDeg,
        actual_step_deg: config.actualStepDeg,
        sample_count: config.sampleCount,
        interpolation: config.interpolation,
        alpha_background: "black",
      },
      sk9822_global_brightness: config.globalBrightness,
      frames,
    };
  }

  return {
    HEADER_BYTES,
    FULL_TURN_UDEG,
    DEFAULT_CONFIG,
    normalizeConfig,
    radiusForLed,
    ledWorldPosition,
    worldToImageUv,
    sampleImage,
    convertImage,
    crc32,
    buildPovBinary,
    parsePovBinary,
    frameRgb,
    buildJsonExport,
  };
});

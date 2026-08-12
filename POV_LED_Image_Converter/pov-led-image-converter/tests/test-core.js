"use strict";

const assert = require("assert");
const Core = require("../src/pov-core.js");

function image(width, height, rgba) {
  return { width, height, data: Uint8ClampedArray.from(rgba) };
}

function nearlyEqual(actual, expected, tolerance = 1e-12) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
}

function run() {
  const adjusted = Core.normalizeConfig({ requestedStepDeg: 7, stripLengthCm: 25 });
  assert.strictEqual(adjusted.ledCount, 35);
  assert.strictEqual(adjusted.sampleCount, 27);
  assert.strictEqual(adjusted.actualStepDeg, 180 / 26);
  nearlyEqual(adjusted.ledPitchCm, 25 / 34);
  assert.strictEqual(adjusted.frameBytes, 148);

  const finest = Core.normalizeConfig({ requestedStepDeg: 0.1 });
  assert.strictEqual(finest.sampleCount, 1801);
  assert.strictEqual(finest.actualStepDeg, 0.1);
  assert.strictEqual(finest.sampleCount * finest.frameBytes, 266548);

  const geometry = Core.normalizeConfig({
    stripLengthCm: 34,
    pivotOffsetCm: 5,
    requestedStepDeg: 45,
  });
  const atMinus90 = Core.ledWorldPosition(geometry, 0, 0);
  nearlyEqual(atMinus90.x, -5);
  nearlyEqual(atMinus90.y, 0, 1e-10);
  assert.strictEqual(atMinus90.mechanicalAngleDeg, -90);

  const upright = Core.ledWorldPosition(geometry, 2, 34);
  nearlyEqual(upright.x, 0, 1e-10);
  nearlyEqual(upright.y, 39);
  assert.strictEqual(upright.mechanicalAngleDeg, 0);
  assert.strictEqual(Core.sampleForAngle(geometry, -90), 0);
  assert.strictEqual(Core.sampleForAngle(geometry, 0), 2);
  assert.strictEqual(Core.sampleForAngle(geometry, 90), 4);

  const pixels = image(2, 2, [
    255, 0, 0, 255,     0, 255, 0, 255,
    0, 0, 255, 255,     255, 255, 255, 255,
  ]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 0, 0, "nearest").map(Math.round), [255, 0, 0]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 1, 1, "nearest").map(Math.round), [255, 255, 255]);
  assert.deepStrictEqual(Core.sampleImage(pixels, .5, .5, "bilinear").map(Math.round), [128, 128, 128]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 2, 2, "nearest"), [0, 0, 0]);

  const solid = image(1, 1, [0x12, 0x34, 0x56, 0xFF]);
  const conversion = Core.convertImage(solid, {
    stripLengthCm: 25,
    pivotOffsetCm: 0,
    requestedStepDeg: 45,
    imageCenterXCm: 0,
    imageCenterYCm: 0,
    imageWidthCm: 1000,
    imageHeightCm: 1000,
    rgbBrightnessPercent: 100,
    globalBrightness: 3,
  });
  assert.strictEqual(conversion.config.sampleCount, 5);
  assert.strictEqual(conversion.payload.length, 5 * 148);
  assert.deepStrictEqual(Array.from(conversion.payload.subarray(0, 8)), [0, 0, 0, 0, 0xE3, 0x56, 0x34, 0x12]);
  assert.deepStrictEqual(Array.from(conversion.payload.subarray(144, 148)), [0xFF, 0xFF, 0xFF, 0xFF]);
  assert.deepStrictEqual(Core.frameRgb(conversion, 0, 0), [0x12, 0x34, 0x56]);

  const primaryColors = [
    { rgba: [255, 0, 0, 255], wireBgr: [0, 0, 255] },
    { rgba: [0, 255, 0, 255], wireBgr: [0, 255, 0] },
    { rgba: [0, 0, 255, 255], wireBgr: [255, 0, 0] },
  ];
  for (const primary of primaryColors) {
    const primaryConversion = Core.convertImage(image(1, 1, primary.rgba), conversion.config);
    assert.deepStrictEqual(Array.from(primaryConversion.payload.subarray(5, 8)), primary.wireBgr);
  }

  const binary = Core.buildWandBinary(conversion);
  assert.strictEqual(binary.length, Core.HEADER_BYTES + conversion.payload.length);
  const parsed = Core.parseWandBinary(binary);
  assert.strictEqual(parsed.version, 1);
  assert.strictEqual(parsed.ledCount, 35);
  assert.strictEqual(parsed.frameBytes, 148);
  assert.strictEqual(parsed.sampleCount, 5);
  assert.strictEqual(parsed.minAngleMdeg, -90000);
  assert.strictEqual(parsed.maxAngleMdeg, 90000);
  assert.strictEqual(parsed.encoding, Core.SK9822_ENCODING);
  assert.strictEqual(parsed.headerCrcValid, true);
  assert.strictEqual(parsed.payloadCrcValid, true);
  assert.deepStrictEqual(Array.from(parsed.payload), Array.from(conversion.payload));

  const corrupted = binary.slice();
  corrupted[corrupted.length - 1] ^= 1;
  assert.strictEqual(Core.parseWandBinary(corrupted).payloadCrcValid, false);

  const json = Core.buildJsonExport(conversion, "solid.png", {
    cycles_per_second: 10,
    maximum_deviation_deg: 35,
  });
  assert.strictEqual(json.format, "WAND1-JSON");
  assert.strictEqual(json.frames.length, 5);
  assert.strictEqual(json.frames[0].angle_deg, -90);
  assert.strictEqual(json.frames[2].angle_deg, 0);
  assert.strictEqual(json.frames[4].angle_deg, 90);
  assert.deepStrictEqual(json.frames[0].leds[0].rgb, [0x12, 0x34, 0x56]);
  assert.strictEqual(json.frames[0].leds[0].sk9822_bytes_hex, "E3563412");
  assert.strictEqual(json.frames[0].wire_frame_hex.length, 148 * 2);

  console.log("All WAND core tests passed.");
}

run();

"use strict";

const assert = require("assert");
const Core = require("../src/pov-core.js");

function image(width, height, rgba) {
  return { width, height, data: Uint8ClampedArray.from(rgba) };
}

function run() {
  const adjusted = Core.normalizeConfig({ requestedStepDeg: 7, ledCount: 35, stripLengthCm: 25 });
  assert.strictEqual(adjusted.sampleCount, 51);
  assert.strictEqual(adjusted.actualStepDeg, 360 / 51);
  assert.ok(Math.abs(adjusted.ledPitchCm - 25 / 34) < 1e-12);

  const geometry = Core.normalizeConfig({
    ledCount: 2,
    stripLengthCm: 10,
    pivotOffsetCm: 5,
    requestedStepDeg: 90,
    clockwise: true,
  });
  assert.deepStrictEqual(Core.ledWorldPosition(geometry, 0, 0), {
    x: 0,
    y: 5,
    radius: 5,
    mechanicalAngleDeg: 0,
  });
  const at90 = Core.ledWorldPosition(geometry, 2, 1);
  assert.ok(Math.abs(at90.x - 15) < 1e-12);
  assert.ok(Math.abs(at90.y) < 1e-12);

  const pixels = image(2, 2, [
    255, 0, 0, 255,     0, 255, 0, 255,
    0, 0, 255, 255,     255, 255, 255, 255,
  ]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 0, 0, "nearest").map(Math.round), [255, 0, 0]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 1, 1, "nearest").map(Math.round), [255, 255, 255]);
  assert.deepStrictEqual(Core.sampleImage(pixels, .5, .5, "bilinear").map(Math.round), [128, 128, 128]);
  assert.deepStrictEqual(Core.sampleImage(pixels, 2, 2, "nearest"), [0, 0, 0]);

  const solid = image(1, 1, [12, 34, 56, 255]);
  const conversion = Core.convertImage(solid, {
    ledCount: 3,
    stripLengthCm: 2,
    pivotOffsetCm: 0,
    requestedStepDeg: 90,
    imageCenterXCm: 0,
    imageCenterYCm: 0,
    imageWidthCm: 100,
    imageHeightCm: 100,
    rgbBrightnessPercent: 100,
    globalBrightness: 3,
  });
  assert.strictEqual(conversion.config.sampleCount, 8, "minimum frame count is 8");
  assert.strictEqual(conversion.payload.length, 8 * 3 * 3);
  assert.deepStrictEqual(Array.from(conversion.payload.subarray(0, 3)), [12, 34, 56]);

  const binary = Core.buildPovBinary(conversion);
  assert.strictEqual(binary.length, Core.HEADER_BYTES + conversion.payload.length);
  const parsed = Core.parsePovBinary(binary);
  assert.strictEqual(parsed.version, 1);
  assert.strictEqual(parsed.ledCount, 3);
  assert.strictEqual(parsed.sampleCount, 8);
  assert.strictEqual(parsed.globalBrightness, 3);
  assert.strictEqual(parsed.headerCrcValid, true);
  assert.strictEqual(parsed.payloadCrcValid, true);
  assert.deepStrictEqual(Array.from(parsed.payload), Array.from(conversion.payload));

  const corrupted = binary.slice();
  corrupted[corrupted.length - 1] ^= 1;
  assert.strictEqual(Core.parsePovBinary(corrupted).payloadCrcValid, false);

  const json = Core.buildJsonExport(conversion, "solid.png");
  assert.strictEqual(json.frames.length, 8);
  assert.deepStrictEqual(json.frames[0].leds_rgb[0], [12, 34, 56]);

  console.log("All POV core tests passed.");
}

run();

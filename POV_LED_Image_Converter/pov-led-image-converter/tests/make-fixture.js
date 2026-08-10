"use strict";

const fs = require("fs");
const path = require("path");
const Core = require("../src/pov-core.js");

const solidPixel = {
  width: 1,
  height: 1,
  data: Uint8ClampedArray.from([0x12, 0x34, 0x56, 0xFF]),
};

const conversion = Core.convertImage(solidPixel, {
  ledCount: 2,
  stripLengthCm: 1,
  pivotOffsetCm: 2,
  requestedStepDeg: 45,
  imageCenterXCm: 0,
  imageCenterYCm: 0,
  imageWidthCm: 100,
  imageHeightCm: 100,
  globalBrightness: 3,
});

fs.writeFileSync(path.join(__dirname, "fixture.pov"), Core.buildPovBinary(conversion));


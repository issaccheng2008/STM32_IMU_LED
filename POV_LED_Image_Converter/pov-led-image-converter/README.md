# POV wand image converter

This offline browser tool converts an image into angle-indexed commands for the
35-pixel SK9822 strip in the STM32 IMU POV wand.  It models a hand-held sweep,
not a continuously rotating fan.

## What changed from the fan converter

- The mechanical range is fixed to **-90° through +90°**, including both
  endpoints.  Upright is 0°, counterclockwise is negative, and clockwise is
  positive.
- LED count is fixed at 35 to match the firmware and tested strip.
- Rotation direction, hardware zero-angle, RPM, full-turn wrapping, and the
  animation control were removed.  They do not describe a back-and-forth wand.
- `pivot → LED 0` remains because the hand/arm pivot changes where every LED
  samples the source image.
- The app adds **cycles per second** (default 10) and **wave angle** (default
  ±35°) to estimate average/peak frame changes, peak angular rate, and LED-bus
  use.
- The binary payload stores complete SK9822 wire frames instead of RGB triples.
  The STM32 can transmit a selected frame without rearranging or expanding it.
- Every binary export is accompanied by an optional human-readable JSON export
  with the exact wire bytes and decoded brightness/RGB values.
- The binary filename is fixed to `WAND.POV`, matching the firmware's read-only
  FAT filesystem.  Converter-only geometry and motion estimates remain in
  `WAND.json`; they are not needed during playback.

## Run the app

For development:

```sh
npm ci
npm run dev
```

Open the URL printed by Vite.  For a production bundle, run `npm run build` and
serve the `dist/` directory.  You can also open `index.html` directly in modern
browsers.  Image processing stays in the browser and no file is uploaded.

## Workflow

1. Load an image.
2. Enter the strip length and signed pivot-to-LED-0 distance.
3. Position, size, and rotate the source image, or drag it in the preview.
4. Choose the requested angle interval.  The app adjusts it slightly when
   necessary so an integer number of intervals covers exactly 180°.
5. Set the planned cycles/second and maximum wave deviation to inspect timing.
6. Check the reconstruction, angle × LED map, and individual LED frame.
7. Download `WAND.POV` and the translated `WAND.json`.
8. Copy `WAND.POV` to the root of a FAT32 microSD card.

## Coordinate model

- The arm/hand pivot is `(0, 0)`.
- `+X` is right and `+Y` is above the pivot.
- Wand angle 0° points straight up.
- Negative angles move counterclockwise/left; positive angles move
  clockwise/right.
- LED 0 is the pixel nearest the handle end.
- `pivot → LED 0` is signed.  A positive value puts LED 0 along the wand above
  the pivot; a negative value places the pivot beyond LED 0.
- Strip length is the LED-center distance from LED 0 to LED 34.  At 25 cm, the
  pitch is `25 / 34 = 0.735294 cm`.

For an LED with radius `r` and wand angle `θ`:

```text
x = r × sin(θ)
y = r × cos(θ)
```

The point is transformed into the configured source-image rectangle and
sampled with nearest-neighbor or bilinear interpolation.  Transparent and
out-of-bounds pixels become black.

## Sampling and memory limits

The requested interval is clamped to 0.1°–45°.  The app uses:

```text
interval_count = round(180° / requested_interval)
actual_interval = 180° / interval_count
sample_count = interval_count + 1
```

This gives 5–1,801 frames and always stores both -90° and +90°.  Every frame is
148 bytes, so the finest 0.1° grid produces a 266,548-byte payload plus the
48-byte header.  This is the largest file the firmware's preload buffer accepts
and fits in STM32H743 RAM_D1.

## Motion estimates

The estimator assumes sinusoidal motion with maximum deviation `A` degrees and
frequency `f` complete left→right→left cycles per second:

```text
average frame changes/s = 4 × A × f / angle_interval
peak frame changes/s    = 2π × A × f / angle_interval
peak angular rate       = 2π × A × f degrees/s
peak SPI use            = peak frame changes/s × 148 × 8 / SPI_bit_rate
```

The default 10 Hz, ±35° wave peaks near 2,199°/s, which is why the STM32 uses
the LSM6DSV320X ±4,000 dps range.  The estimate is a planning bound: the
firmware sends only when a new orientation estimate selects a different frame.

## Output files

`WAND.POV` is the compact, CRC-protected runtime file:

```text
48-byte WAND1 header
frame -90°: 00 00 00 00 + 35 × [111BBBBB,B,G,R] + FF FF FF FF
next angle frame
...
frame +90°
```

`WAND.json` is the readable translation.  Each frame contains:

- its wand angle;
- the complete 148-byte wire frame as hexadecimal;
- all 35 LED commands as global brightness, RGB, and four SK9822 bytes;
- source geometry, sampling settings, and the motion estimate.

See [docs/FILE_FORMAT.md](docs/FILE_FORMAT.md) for the byte-level contract.  The
matching STM32 implementation is in `Core/Inc/wand_file.h` and
`Core/Src/wand_file.c` at the repository root.

## Tests

```sh
npm test
npm run build
```

The test suite checks geometry, angle endpoints, image sampling, direct SK9822
byte order, both CRCs, JSON translation, and compatibility between the
JavaScript writer and the exact pure-C parser used by the firmware.

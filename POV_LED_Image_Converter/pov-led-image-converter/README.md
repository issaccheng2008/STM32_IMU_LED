# POV LED Image Converter

An offline, browser-based converter for turning an ordinary image into angle-indexed RGB commands for a rotating LED strip. The defaults match this project:

- 35 full-color SK9822 LEDs
- 25 cm LED-center span
- adjustable pivot-to-LED-0 distance
- adjustable image position, physical size, and rotation
- adjustable angular interval (1°, 0.5°, 0.1°, and other values)
- binary output intended for an STM32H743 reading from microSD

## Run it

There are **no dependencies to install**. The tool does not upload the selected image and does not need an internet connection.

1. Extract the project folder.
2. Double-click `index.html`.
3. If your browser blocks local scripts (most current browsers do not), run a local server from this folder with either:

   ```sh
   python -m http.server 8080
   ```

   or:

   ```sh
   python3 -m http.server 8080
   ```

4. Open `http://localhost:8080`.

Python is only needed for the optional fallback server. Node.js is only needed to run the developer tests.

## Coordinate model

- The **pivot** is the origin `(0, 0)`.
- `+X` is right; `+Y` is above the pivot.
- LED 0 is the LED nearest the lower end of the strip.
- `pivot → LED 0` is signed. A positive value puts LED 0 above the pivot. A negative value means the pivot lies above LED 0.
- The strip length is the LED-center distance from LED 0 to the last LED. With the defaults, pitch is `25 cm / (35 - 1) = 0.735294 cm`.
- Hardware angle 0° points upward by default. Its direction and the direction of increasing mechanical angle are configurable.
- Image position is its center, measured relative to the pivot.

At each angle, the converter calculates every LED center's world position, transforms it into the image's coordinate system, and samples the image. Samples outside the image and transparent pixels become black.

## Typical workflow

1. Load an image.
2. Enter the physical strip dimensions and pivot offset.
3. Position and size the image numerically or drag it in the preview.
4. Choose the angular interval.
5. Inspect the reconstructed image, the angle × LED heatmap, and individual angle frames.
6. Enter the planned RPM and SPI clock to check whether the SK9822 bus is fast enough.
7. Export the `.pov` binary for the microSD card.

The JSON export is intended for debugging and verification. It is much larger and should not normally be used by the STM32.

## Output files

The compact `.pov` file is:

```text
80-byte POV1 header
frame 0: LED 0 RGB, LED 1 RGB, ... LED N RGB
frame 1: LED 0 RGB, LED 1 RGB, ... LED N RGB
...
```

Each frame is contiguous, so firmware can seek to one angle without parsing earlier frames:

```text
frame_offset = 80 + angle_index * led_count * 3
```

See [docs/FILE_FORMAT.md](docs/FILE_FORMAT.md) for the complete specification and [firmware/pov_file.c](firmware/pov_file.c) for a portable STM32-oriented decoder.

## SK9822 note

The `.pov` payload stores ordinary **RGB**. Your supplied SK9822 specification expects each physical pixel as:

```text
0xE0 | global_brightness, green, red, blue
```

The included firmware helper performs this RGB-to-GRB packaging. Keeping the disk file as RGB makes it easier to inspect and reuse.

## Timing note

The converter's runtime estimator includes the SK9822 start frame, 4 bytes per LED, and end frame. A setting can generate valid image data but still demand more LED updates than the selected SPI clock can transmit. For firmware, use buffered SD reads and SPI DMA; do not perform a separate filesystem seek for every LED.

## Developer tests

With Node.js 18 or newer installed:

```sh
node tests/test-core.js
```

The test suite checks geometry, pixel sampling, binary layout, CRCs, and payload round-tripping. There are no npm packages to install.


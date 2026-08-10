# STM32 IMU POV wand

This repository is the integrated firmware and image-conversion tool for a
hand-waved, 35-pixel SK9822 persistence-of-vision wand.  It combines the
working LSM6DSV320X orientation code from this repository with the proven
SK9822 protocol and pin configuration from `LED_strip_test`.

The runtime path is deliberately simple:

1. The browser converter samples an image at every stored wand angle from
   -90° through +90°.
2. It exports `WAND.POV`, whose payload already contains complete SK9822 wire
   frames, and `WAND.json`, a human-readable translation.
3. At boot, the STM32 mounts the microSD card, validates `WAND.POV`, and copies
   its complete payload into internal RAM.
4. The LSM6DSV320X orientation filter produces a new roll estimate at up to
   1,920 samples/second.
5. The firmware maps roll to the nearest stored angle and sends that frame to
   the LED strip.  The real-time loop never accesses the SD card and never
   repacks RGB values.

## Hardware connections

These assignments match the custom STM32H743VIT6 board schematic and the
tested LED project.

| Device | Signal | STM32 pin | Peripheral |
|---|---|---|---|
| LSM6DSV320X | SCL | PB6 | I2C1 SCL |
| LSM6DSV320X | SDA | PB7 | I2C1 SDA |
| SK9822 strip | Clock | PA5 | SPI1 SCK |
| SK9822 strip | Data | PA7 | SPI1 MOSI |
| microSD | D0, D1, D2, D3 | PC8, PC9, PC10, PC11 | SDMMC1 |
| microSD | Clock | PC12 | SDMMC1 CK |
| microSD | Command | PD2 | SDMMC1 CMD |

Use a common ground for the board, IMU, LED strip, and LED power supply.  Size
the LED supply for the intended brightness; do not power a fully illuminated
35-pixel strip from an STM32 GPIO or regulator output.

## Angle conventions

The converter, command file, diagnostics, and playback code all use one wand
angle convention:

| Physical pose | Fusion roll | Wand angle | Stored endpoint/index |
|---|---:|---:|---|
| 90° counterclockwise from upright | 0° | -90° | first frame |
| Upright | -90° | 0° | middle frame |
| 90° clockwise from upright | -180° | +90° | last frame |

The firmware implements `wand_angle = normalize(-90° - roll)` and clamps the
result to `[-90°, +90°]`.  Positive wand angles are clockwise when viewed in
the converter.

## First run

1. Open `POV_LED_Image_Converter/pov-led-image-converter` and run:

   ```sh
   npm ci
   npm run dev
   ```

2. Load an image, set its geometry and the wand parameters, then download both
   exports.
3. Format a microSD card as FAT32 and copy `WAND.POV` to its root directory.
   The firmware is read-only and uses the fixed 8.3 filename `WAND.POV`.
4. Insert the card before resetting the STM32.
5. Import this directory into STM32CubeIDE, build the `STM32_IMU_LED` project,
   and flash the Debug or Release image.
6. Hold the wand still during the orientation filter's initial convergence.
   The LEDs remain blank during this startup phase.

The legacy filenames `LED-test.ioc` and `LED-test Debug.launch` are retained so
existing CubeIDE workspaces can find them, but the Eclipse/CubeIDE project and
build artifact are named `STM32_IMU_LED`.

## Firmware configuration

| Path | Setting | Value and reason |
|---|---|---|
| IMU bus | I2C1 | 1 MHz Fast-mode Plus on PB6/PB7 |
| Accelerometer/gyro ODR | LSM6DSV320X | 1,920 Hz high-performance mode; a robust polling rate within 1 MHz I2C and 64 MHz CPU timing |
| Gyro full scale | LSM6DSV320X | ±4,000 dps, needed for fast hand motion; the default 10 Hz, ±35° estimate peaks near 2,199 dps |
| Accelerometer filter | LSM6DSV320X | LPF1 output, bypassing the former LPF2 delay |
| Gyro filter | LSM6DSV320X | LPF1 ultra-light for minimum sensor-side delay |
| LED bus | SPI1 mode 0 | 9.375 MHz, MSB first, transmit-only |
| SD bus | SDMMC1 | 4-bit, 18.75 MHz transfer clock after card initialization |
| Orientation timer | TIM2 | 1 MHz free-running timestamp counter |

The sensor supports higher ODR settings, but 1,920 Hz is used because every
poll must also transfer status, acceleration, and gyroscope data, run the
Fusion update, and occasionally transmit a 148-byte LED frame.  This avoids
claiming a sensor register rate the complete polling loop cannot sustain.

## Startup and playback behavior

- `MX_SDMMC1_SD_Init()` configures the handle, while FatFs initializes the card
  lazily.  A missing card becomes a diagnostic error instead of stopping IMU
  operation in `Error_Handler()`.
- `WAND_StorageLoad()` validates the header CRC, exact file length, payload CRC,
  fixed 35-pixel layout, angle range, and every SK9822 start/end frame.
- Up to 1,801 frames at 0.1° spacing occupy 266,548 bytes of RAM.  The buffer is
  32-byte aligned and fits in the STM32H743's 512 KiB RAM_D1 region.
- The strip is blanked at boot.  Playback begins only when a valid file is in
  RAM and the Fusion startup flag clears.
- A new SPI transfer is skipped when two orientation updates select the same
  angle frame.

## Debug console and LED self-test

The Debug configuration now uses the same standard `printf()` plus librdimon
semihosting path as `imu_calibration_session.c`.  Start the included
`LED-test Debug.launch` configuration, keep **Enable terminal and File I/O
mode** selected, open the CubeIDE Console, and press **Resume (F8)**.  Debug
output requires the ST-Link debugger to remain connected; use the Release
configuration for standalone operation.

Every Debug boot performs a one-second, 1/31-brightness LED hardware test before
the SD card or IMU can gate playback.  LEDs 0-11 should be red, LEDs 12-23
green, and LEDs 24-34 blue, after which the strip is blanked.  Set
`wand_debug_led_self_test_on_boot` to `0` in Live Expressions while stopped at
`main()` to skip it.

The console then reports each boundary independently:

| Prefix | What it proves or identifies |
|---|---|
| `[TEST][LED] ... HAL_OK` | STM32 SPI accepted the 148-byte command; if the RGB test is invisible, inspect LED power, common ground, strip input direction, PA5/PA7, and logic levels |
| `[ERROR][SD]` | Exact loader phase, FatFs result, WAND parser result, actual byte count, and expected size |
| `[OK][SD]` | `WAND.POV` was completely loaded and both CRCs passed; the following content count warns if all commands are black |
| `[ERROR][IMU]` | IMU initialization step, WHO_AM_I, selected I2C address, or live read status failed |
| `[WAIT][IMU]` | Fusion startup is still intentionally preventing playback |
| `[RUN]` | Once-per-second orientation rate, roll, mapped angle, frame index, illuminated LED count, successful transmissions, attempts, and SPI errors |
| `[FATAL][HAL]` | Cube HAL initialization entered `Error_Handler()`, including the boot stage and peripheral error flags |

The fastest diagnosis is:

1. If the RGB boot test is invisible but reports `HAL_OK`, debug the LED power
   and SPI electrical path before the SD card or IMU.
2. If the boot test is visible, follow the first `[ERROR][SD]` or
   `[ERROR][IMU]` message.
3. If startup reaches `[RUN]`, confirm `startup=0`, `orient` is nonzero,
   `lit` is nonzero for the selected frame, `tx_ok` increases as the angle
   changes, and `spi=HAL_OK`.
4. A changing frame with `lit=0` is a valid black command, not an SPI failure;
   inspect that angle in `WAND.json` or reposition the image in the converter.

Periodic output is deliberately limited to one line per second because every
semihosting call pauses the MCU.  Set `wand_debug_periodic_output_enabled` to
`0` in Live Expressions when measuring maximum IMU/playback timing.

Useful Live Expressions are:

| Variable | Meaning |
|---|---|
| `wand_storage_status` | `0` when the file was mounted, loaded, and validated |
| `wand_loaded` | `1` when playback data is resident in RAM |
| `wand_angle_deg` | Current mapped angle in the converter convention |
| `wand_sample_index` | Nearest selected WAND1 frame |
| `wand_frame_update_count` | Successful LED frame transmissions |
| `wand_frame_attempt_count` | All SPI attempts, including the Debug boot test |
| `wand_spi_status` | Latest `HAL_SPI_Transmit` result |
| `wand_spi_hal_error` | Detailed HAL SPI error bitmask |
| `wand_spi_error_count` | Number of failed SPI attempts |
| `wand_selected_lit_led_count` | Non-black LEDs in the selected command frame |
| `imu_orientation_sample_rate_hz` | Measured orientation update rate |
| `imu_status` | Latest IMU initialization/read result |

The SD loader uses these main error ranges: `-101` mount/card failure, `-102`
missing `WAND.POV`, `-103` short header, `-12x` invalid WAND1 header, `-140`
wrong file size, `-141` file too large, `-142` short payload, and `-16x`
invalid payload.  Exact values are defined in `Core/Inc/wand_storage.h`.

## Command-file format

`WAND.POV` uses the WAND1 format documented in
`POV_LED_Image_Converter/pov-led-image-converter/docs/FILE_FORMAT.md`.  Each
148-byte payload frame is exactly what SPI transmits:

```text
00 00 00 00
35 × [111BBBBB, green, red, blue]
FF FF FF FF
```

`WAND.json` contains the same full frame as hexadecimal plus decoded
brightness and RGB values for every LED.  It is for inspection and is not read
by the STM32.

## Calibration

The existing low-g, high-g, and gyroscope calibration workflow remains
available.  Set `imu_run_calibration_on_boot` in Live Expressions while stopped
at `main()` to enter the semihosting calibration session.  Normal playback
leaves it at `0`.  The Debug build also uses semihosting for the diagnostics
above; the Release build contains neither console calls nor the RGB self-test.

## Tests

Run the converter, format, and cross-language parser tests with:

```sh
cd POV_LED_Image_Converter/pov-led-image-converter
npm ci
npm test
npm run build
```

`npm test` creates a JavaScript WAND1 fixture, then compiles the same pure-C
header/CRC/angle parser used by the STM32 and verifies it against that fixture.
The repository also retains the existing host-side orientation and calibration
tests under `tests/`.

Hardware timing, SD electrical behavior, LED signal integrity, and the final
orientation axis/sign must still be verified on the physical wand.  If the IMU
is remounted, update and re-test only the documented roll-to-wand mapping rather
than reversing angles in both the converter and firmware.

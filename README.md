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

Useful Live Expressions are:

| Variable | Meaning |
|---|---|
| `wand_storage_status` | `0` when the file was mounted, loaded, and validated |
| `wand_loaded` | `1` when playback data is resident in RAM |
| `wand_angle_deg` | Current mapped angle in the converter convention |
| `wand_sample_index` | Nearest selected WAND1 frame |
| `wand_frame_update_count` | Successful LED frame transmissions |
| `wand_spi_status` | Latest `HAL_SPI_Transmit` result |
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
at `main()` to enter the semihosting calibration session.  Normal standalone
playback leaves it at `0`; semihosting is not used in the real-time loop.

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

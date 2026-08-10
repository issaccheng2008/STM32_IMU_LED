# STM32 integration notes

`pov_file.c` is deliberately independent of HAL, FatFs, and a specific SD-card interface. Supply a small random-access read callback and it will:

- validate the 80-byte header and header CRC;
- optionally validate the full payload CRC in small chunks;
- convert an encoder angle to the nearest frame index;
- read one contiguous RGB frame;
- package that frame into the SK9822 start / G-R-B pixel / end sequence described by the supplied strip documentation.

## FatFs callback example

Adapt the file handle type and error handling to your project:

```c
#include "ff.h"
#include "pov_file.h"

static bool fatfs_read_at(void *context,
                          uint32_t offset,
                          uint8_t *destination,
                          uint32_t length)
{
    FIL *file = (FIL *)context;
    UINT bytes_read = 0;

    if (f_lseek(file, offset) != FR_OK) {
        return false;
    }
    if (f_read(file, destination, length, &bytes_read) != FR_OK) {
        return false;
    }
    return bytes_read == length;
}
```

Basic use for 35 LEDs:

```c
static uint8_t rgb_frame[35u * 3u];
static uint8_t spi_frame[8u + 35u * 4u];

pov1_header_t pov_header;
uint32_t spi_length;
uint32_t sample;

if (pov1_read_header(fatfs_read_at, &pov_fil, &pov_header) != POV_OK) {
    Error_Handler();
}

sample = pov1_angle_to_sample(&pov_header, measured_angle_udeg);
if (pov1_read_rgb_frame(fatfs_read_at,
                        &pov_fil,
                        &pov_header,
                        sample,
                        rgb_frame,
                        sizeof(rgb_frame)) != POV_OK) {
    Error_Handler();
}

if (pov1_build_sk9822_frame(&pov_header,
                            rgb_frame,
                            sizeof(rgb_frame),
                            POV1_USE_HEADER_BRIGHTNESS,
                            spi_frame,
                            sizeof(spi_frame),
                            &spi_length) != POV_OK) {
    Error_Handler();
}

HAL_SPI_Transmit_DMA(&hspi1, spi_frame, (uint16_t)spi_length);
```

## Important real-time recommendation

The callback above illustrates correctness, but an `f_lseek` plus `f_read` in every angle interrupt can have unpredictable latency. For rotation:

1. Read ahead multiple contiguous frames into a RAM ring buffer.
2. Let the angle/timer path select already-buffered data.
3. Use SPI DMA and do not modify `spi_frame` until its DMA completion callback.
4. Keep two SPI buffers if the next frame must be prepared while the current frame is transmitting.
5. Verify the payload CRC once before enabling the motor, not during the real-time loop.

At the default 35 LEDs, an RGB frame is 105 bytes and an SK9822 wire frame is 148 bytes.


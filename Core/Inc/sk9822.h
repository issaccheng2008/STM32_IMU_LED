#ifndef SK9822_H
#define SK9822_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define SK9822_LED_COUNT          35U
#define SK9822_START_FRAME_BYTES  4U
#define SK9822_BYTES_PER_LED      4U
#define SK9822_END_FRAME_BYTES    4U
#define SK9822_FRAME_BYTES        \
  (SK9822_START_FRAME_BYTES + \
   (SK9822_LED_COUNT * SK9822_BYTES_PER_LED) + \
   SK9822_END_FRAME_BYTES)

/** Fill one complete frame that turns every LED off. */
void SK9822_MakeBlankFrame(uint8_t *frame, uint32_t capacity);

/** Check the fixed start/end framing used by this strip. */
bool SK9822_IsFrameValid(const uint8_t *frame, uint32_t length);

/**
 * Transmit one prebuilt frame synchronously.
 *
 * This is retained for boot diagnostics. Runtime playback should use the DMA
 * API below so orientation polling is not blocked by the SPI wire time.
 */
HAL_StatusTypeDef SK9822_TransmitFrame(SPI_HandleTypeDef *hspi,
                                      const uint8_t *frame,
                                      uint32_t length);

/** Start one asynchronous transmission using CubeMX-configured SPI TX DMA. */
HAL_StatusTypeDef SK9822_TransmitFrameDma(SPI_HandleTypeDef *hspi,
                                         const uint8_t *frame,
                                         uint32_t length);

#endif /* SK9822_H */

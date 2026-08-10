#include "sk9822.h"

#include <string.h>

#define SK9822_SPI_TIMEOUT_MS  20U

void SK9822_MakeBlankFrame(uint8_t *frame, uint32_t capacity)
{
  uint32_t led;

  if ((frame == NULL) || (capacity < SK9822_FRAME_BYTES))
    return;

  memset(frame, 0x00, SK9822_START_FRAME_BYTES);
  for (led = 0U; led < SK9822_LED_COUNT; led++)
  {
    const uint32_t offset = SK9822_START_FRAME_BYTES +
                            led * SK9822_BYTES_PER_LED;
    frame[offset] = 0xE0U;
    frame[offset + 1U] = 0x00U;
    frame[offset + 2U] = 0x00U;
    frame[offset + 3U] = 0x00U;
  }
  memset(&frame[SK9822_FRAME_BYTES - SK9822_END_FRAME_BYTES],
         0xFF,
         SK9822_END_FRAME_BYTES);
}

bool SK9822_IsFrameValid(const uint8_t *frame, uint32_t length)
{
  const uint32_t end = SK9822_FRAME_BYTES - SK9822_END_FRAME_BYTES;

  if ((frame == NULL) || (length != SK9822_FRAME_BYTES))
    return false;

  return (frame[0] == 0x00U) && (frame[1] == 0x00U) &&
         (frame[2] == 0x00U) && (frame[3] == 0x00U) &&
         (frame[end] == 0xFFU) && (frame[end + 1U] == 0xFFU) &&
         (frame[end + 2U] == 0xFFU) && (frame[end + 3U] == 0xFFU);
}

HAL_StatusTypeDef SK9822_TransmitFrame(SPI_HandleTypeDef *hspi,
                                      const uint8_t *frame,
                                      uint32_t length)
{
  if ((hspi == NULL) || (frame == NULL) ||
      (length != SK9822_FRAME_BYTES))
  {
    return HAL_ERROR;
  }

  return HAL_SPI_Transmit(hspi,
                          (uint8_t *)frame,
                          (uint16_t)length,
                          SK9822_SPI_TIMEOUT_MS);
}

#include "wand_file.h"

#include <string.h>

static uint16_t read_u16_le(const uint8_t *bytes)
{
  return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *bytes)
{
  return (uint32_t)bytes[0] |
         ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) |
         ((uint32_t)bytes[3] << 24);
}

static int32_t read_i32_le(const uint8_t *bytes)
{
  return (int32_t)read_u32_le(bytes);
}

uint32_t WAND_Crc32Begin(void)
{
  return 0xFFFFFFFFU;
}

uint32_t WAND_Crc32Update(uint32_t crc,
                         const uint8_t *data,
                         uint32_t length)
{
  uint32_t index;
  uint32_t bit;

  if (data == NULL)
    return crc;

  for (index = 0U; index < length; index++)
  {
    crc ^= data[index];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = (crc >> 1) ^
            (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
    }
  }

  return crc;
}

uint32_t WAND_Crc32Finish(uint32_t crc)
{
  return crc ^ 0xFFFFFFFFU;
}

wand_result_t WAND_ParseHeader(const uint8_t *bytes,
                               uint32_t length,
                               wand1_header_t *header)
{
  uint64_t expected_payload_bytes;
  uint32_t expected_angle_step_udeg;
  uint32_t expected_header_crc;
  uint32_t actual_header_crc;

  if ((bytes == NULL) || (header == NULL))
    return WAND_ERROR_ARGUMENT;

  if (length < WAND1_HEADER_BYTES)
    return WAND_ERROR_SHORT_HEADER;

  if ((bytes[0] != 'W') || (bytes[1] != 'A') ||
      (bytes[2] != 'N') || (bytes[3] != 'D'))
  {
    return WAND_ERROR_MAGIC;
  }

  if (read_u16_le(&bytes[4]) != WAND1_VERSION)
    return WAND_ERROR_VERSION;

  expected_header_crc = read_u32_le(&bytes[44]);
  actual_header_crc = WAND_Crc32Finish(
      WAND_Crc32Update(WAND_Crc32Begin(), bytes, 44U));
  if (actual_header_crc != expected_header_crc)
    return WAND_ERROR_HEADER_CRC;

  memset(header, 0, sizeof(*header));
  header->led_count = read_u16_le(&bytes[8]);
  header->frame_bytes = read_u16_le(&bytes[10]);
  header->sample_count = read_u32_le(&bytes[12]);
  header->min_angle_mdeg = read_i32_le(&bytes[16]);
  header->max_angle_mdeg = read_i32_le(&bytes[20]);
  header->angle_step_udeg = read_u32_le(&bytes[24]);
  header->payload_bytes = read_u32_le(&bytes[28]);
  header->payload_crc32 = read_u32_le(&bytes[32]);
  header->encoding = bytes[36];
  header->flags = read_u16_le(&bytes[38]);
  header->file_bytes = read_u32_le(&bytes[40]);

  if ((read_u16_le(&bytes[6]) != WAND1_HEADER_BYTES) ||
      (header->led_count != WAND1_LED_COUNT) ||
      (header->frame_bytes != WAND1_SK9822_FRAME_BYTES) ||
      (header->sample_count < WAND1_MIN_SAMPLE_COUNT) ||
      (header->sample_count > WAND1_MAX_SAMPLE_COUNT) ||
      (header->min_angle_mdeg != WAND1_MIN_ANGLE_MDEG) ||
      (header->max_angle_mdeg != WAND1_MAX_ANGLE_MDEG) ||
      (header->angle_step_udeg == 0U) ||
      (header->encoding != WAND1_ENCODING_SK9822_WIRE) ||
      ((header->flags & WAND1_REQUIRED_FLAGS) != WAND1_REQUIRED_FLAGS))
  {
    return WAND_ERROR_FORMAT;
  }

  expected_angle_step_udeg =
      (180000000U + ((header->sample_count - 1U) / 2U)) /
      (header->sample_count - 1U);
  if (header->angle_step_udeg != expected_angle_step_udeg)
    return WAND_ERROR_FORMAT;

  expected_payload_bytes =
      (uint64_t)header->sample_count * (uint64_t)header->frame_bytes;
  if ((expected_payload_bytes > WAND1_MAX_PAYLOAD_BYTES) ||
      (header->payload_bytes != (uint32_t)expected_payload_bytes) ||
      (header->file_bytes != WAND1_HEADER_BYTES + header->payload_bytes))
  {
    return WAND_ERROR_FORMAT;
  }

  return WAND_OK;
}

wand_result_t WAND_ValidatePayload(const wand1_header_t *header,
                                   const uint8_t *payload,
                                   uint32_t length)
{
  uint32_t sample;
  uint32_t actual_crc;

  if ((header == NULL) || (payload == NULL))
    return WAND_ERROR_ARGUMENT;

  if (length != header->payload_bytes)
    return WAND_ERROR_PAYLOAD_LENGTH;

  actual_crc = WAND_Crc32Finish(
      WAND_Crc32Update(WAND_Crc32Begin(), payload, length));
  if (actual_crc != header->payload_crc32)
    return WAND_ERROR_PAYLOAD_CRC;

  for (sample = 0U; sample < header->sample_count; sample++)
  {
    const uint8_t *frame = &payload[sample * header->frame_bytes];
    const uint32_t end = header->frame_bytes - 4U;
    uint32_t led;

    if ((frame[0] != 0x00U) || (frame[1] != 0x00U) ||
        (frame[2] != 0x00U) || (frame[3] != 0x00U) ||
        (frame[end] != 0xFFU) || (frame[end + 1U] != 0xFFU) ||
        (frame[end + 2U] != 0xFFU) || (frame[end + 3U] != 0xFFU))
    {
      return WAND_ERROR_SK9822_FRAME;
    }

    for (led = 0U; led < WAND1_LED_COUNT; led++)
    {
      if ((frame[4U + led * 4U] & 0xE0U) != 0xE0U)
        return WAND_ERROR_SK9822_FRAME;
    }
  }

  return WAND_OK;
}

int32_t WAND_RollToAngleMdeg(int32_t roll_mdeg)
{
  int32_t angle_mdeg = -90000 - roll_mdeg;

  while (angle_mdeg > 180000)
    angle_mdeg -= 360000;
  while (angle_mdeg < -180000)
    angle_mdeg += 360000;

  if (angle_mdeg < WAND1_MIN_ANGLE_MDEG)
    angle_mdeg = WAND1_MIN_ANGLE_MDEG;
  if (angle_mdeg > WAND1_MAX_ANGLE_MDEG)
    angle_mdeg = WAND1_MAX_ANGLE_MDEG;

  return angle_mdeg;
}

uint32_t WAND_AngleToSample(const wand1_header_t *header,
                            int32_t angle_mdeg)
{
  int64_t relative_udeg;
  uint64_t rounded_udeg;
  uint32_t sample;

  if ((header == NULL) || (header->sample_count == 0U) ||
      (header->angle_step_udeg == 0U))
  {
    return 0U;
  }

  if (angle_mdeg < header->min_angle_mdeg)
    angle_mdeg = header->min_angle_mdeg;
  if (angle_mdeg > header->max_angle_mdeg)
    angle_mdeg = header->max_angle_mdeg;

  relative_udeg =
      ((int64_t)angle_mdeg - (int64_t)header->min_angle_mdeg) * 1000;
  rounded_udeg = (uint64_t)relative_udeg +
                 ((uint64_t)header->angle_step_udeg / 2U);
  sample = (uint32_t)(rounded_udeg / header->angle_step_udeg);
  if (sample >= header->sample_count)
    sample = header->sample_count - 1U;

  return sample;
}

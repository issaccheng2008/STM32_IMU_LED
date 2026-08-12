#include "wand_file.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static int fail(const char *message)
{
  fprintf(stderr, "FAIL: %s\n", message);
  return 1;
}

int main(void)
{
  FILE *input;
  long file_length;
  uint8_t *file_bytes;
  wand1_header_t header;
  const uint8_t *payload;

  input = fopen("tests/fixture.pov", "rb");
  if (input == NULL)
    return fail("could not open fixture.pov");
  if (fseek(input, 0, SEEK_END) != 0)
    return fail("seek failed");
  file_length = ftell(input);
  if ((file_length <= 0) || ((unsigned long)file_length > UINT32_MAX))
    return fail("bad fixture size");
  if (fseek(input, 0, SEEK_SET) != 0)
    return fail("rewind failed");

  file_bytes = (uint8_t *)malloc((size_t)file_length);
  if (file_bytes == NULL)
    return fail("allocation failed");
  if (fread(file_bytes, 1U, (size_t)file_length, input) !=
      (size_t)file_length)
  {
    return fail("read failed");
  }
  fclose(input);

  if (WAND_ParseHeader(file_bytes, (uint32_t)file_length, &header) != WAND_OK)
    return fail("C parser rejected the JavaScript header");
  if ((header.led_count != 35U) ||
      (header.sample_count != 5U) ||
      (header.frame_bytes != 148U))
  {
    return fail("header values differ");
  }

  payload = &file_bytes[WAND1_HEADER_BYTES];
  if (WAND_ValidatePayload(&header, payload, header.payload_bytes) != WAND_OK)
    return fail("C parser rejected the JavaScript payload");
  if ((payload[4] != 0xE3U) || (payload[5] != 0x56U) ||
      (payload[6] != 0x34U) || (payload[7] != 0x12U))
  {
    return fail("direct SK9822 command bytes differ");
  }

  if ((WAND_AngleToSample(&header, -90000) != 0U) ||
      (WAND_AngleToSample(&header, 0) != 2U) ||
      (WAND_AngleToSample(&header, 90000) != 4U))
  {
    return fail("angle lookup differs");
  }

  if ((WAND_RollToAngleMdeg(-90000) != 0) ||
      (WAND_RollToAngleMdeg(0) != -90000) ||
      (WAND_RollToAngleMdeg(-180000) != 90000))
  {
    return fail("roll-to-wand mapping differs");
  }

  free(file_bytes);
  puts("JavaScript-to-STM32 WAND1 compatibility test passed.");
  return 0;
}

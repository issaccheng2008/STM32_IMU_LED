#ifndef WAND_FILE_H
#define WAND_FILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WAND1_HEADER_BYTES                 48U
#define WAND1_VERSION                      1U
#define WAND1_LED_COUNT                    35U
#define WAND1_SK9822_FRAME_BYTES           148U
#define WAND1_MIN_ANGLE_MDEG               (-90000)
#define WAND1_MAX_ANGLE_MDEG               90000
#define WAND1_ENCODING_SK9822_WIRE         1U
#define WAND1_REQUIRED_FLAGS               0x0007U
#define WAND1_MIN_SAMPLE_COUNT             5U
#define WAND1_MAX_SAMPLE_COUNT             1801U
#define WAND1_MAX_PAYLOAD_BYTES            \
  (WAND1_MAX_SAMPLE_COUNT * WAND1_SK9822_FRAME_BYTES)

typedef enum
{
  WAND_OK = 0,
  WAND_ERROR_ARGUMENT,
  WAND_ERROR_SHORT_HEADER,
  WAND_ERROR_MAGIC,
  WAND_ERROR_VERSION,
  WAND_ERROR_FORMAT,
  WAND_ERROR_HEADER_CRC,
  WAND_ERROR_PAYLOAD_LENGTH,
  WAND_ERROR_PAYLOAD_CRC,
  WAND_ERROR_SK9822_FRAME
} wand_result_t;

typedef struct
{
  uint16_t led_count;
  uint16_t frame_bytes;
  uint32_t sample_count;
  int32_t min_angle_mdeg;
  int32_t max_angle_mdeg;
  uint32_t angle_step_udeg;
  uint32_t payload_bytes;
  uint32_t payload_crc32;
  uint8_t encoding;
  uint16_t flags;
  uint32_t file_bytes;
} wand1_header_t;

uint32_t WAND_Crc32Begin(void);
uint32_t WAND_Crc32Update(uint32_t crc,
                         const uint8_t *data,
                         uint32_t length);
uint32_t WAND_Crc32Finish(uint32_t crc);

wand_result_t WAND_ParseHeader(const uint8_t *bytes,
                               uint32_t length,
                               wand1_header_t *header);

wand_result_t WAND_ValidatePayload(const wand1_header_t *header,
                                   const uint8_t *payload,
                                   uint32_t length);

/** Convert Fusion roll to the wand convention, in millidegrees.
 *
 * Fusion roll is -90 degrees upright, 0 degrees on the counterclockwise side,
 * and -180 degrees on the clockwise side.  The returned wand angle is 0
 * upright, -90 counterclockwise, and +90 clockwise.
 */
int32_t WAND_RollToAngleMdeg(int32_t roll_mdeg);

/** Select the nearest stored frame, clamping outside the -90 to +90 range. */
uint32_t WAND_AngleToSample(const wand1_header_t *header,
                            int32_t angle_mdeg);

#ifdef __cplusplus
}
#endif

#endif /* WAND_FILE_H */

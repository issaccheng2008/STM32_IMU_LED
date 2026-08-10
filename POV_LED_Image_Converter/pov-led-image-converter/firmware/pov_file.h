#ifndef POV_FILE_H
#define POV_FILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POV1_HEADER_BYTES            80u
#define POV1_FULL_TURN_UDEG           360000000u
#define POV1_USE_HEADER_BRIGHTNESS    0xFFu
#define POV1_RGB_FRAME_BYTES(leds)    ((uint32_t)(leds) * 3u)
#define POV1_SK9822_FRAME_BYTES(leds) (8u + (uint32_t)(leds) * 4u)

typedef bool (*pov_read_fn)(void *context, uint32_t offset, uint8_t *destination, uint32_t length);

typedef enum {
    POV_OK = 0,
    POV_ERROR_ARGUMENT,
    POV_ERROR_READ,
    POV_ERROR_MAGIC,
    POV_ERROR_VERSION,
    POV_ERROR_FORMAT,
    POV_ERROR_HEADER_CRC,
    POV_ERROR_PAYLOAD_CRC,
    POV_ERROR_BUFFER_TOO_SMALL,
    POV_ERROR_INDEX
} pov_result_t;

typedef struct {
    uint16_t led_count;
    uint32_t sample_count;
    uint32_t angle_step_udeg;
    uint32_t payload_bytes;
    uint32_t payload_crc32;
    int32_t strip_length_um;
    int32_t pivot_to_led0_um;
    int32_t image_center_x_um;
    int32_t image_center_y_um;
    uint32_t image_width_um;
    uint32_t image_height_um;
    int32_t image_rotation_mdeg;
    int32_t zero_angle_mdeg;
    uint32_t flags;
    uint32_t led_pitch_um;
    int32_t led0_radius_um;
    uint8_t global_brightness;
} pov1_header_t;

/* Reads and validates the fixed header. This performs no dynamic allocation. */
pov_result_t pov1_read_header(pov_read_fn read_fn, void *context, pov1_header_t *header);

/* Optional full-payload validation. scratch_length can be small (for example 512 bytes). */
pov_result_t pov1_validate_payload_crc(
    pov_read_fn read_fn,
    void *context,
    const pov1_header_t *header,
    uint8_t *scratch,
    uint32_t scratch_length
);

/* Selects the nearest stored frame for an angle in microdegrees. */
uint32_t pov1_angle_to_sample(const pov1_header_t *header, uint32_t angle_udeg);

/* Reads one contiguous RGB frame. rgb_capacity must be at least led_count * 3. */
pov_result_t pov1_read_rgb_frame(
    pov_read_fn read_fn,
    void *context,
    const pov1_header_t *header,
    uint32_t sample_index,
    uint8_t *rgb,
    uint32_t rgb_capacity
);

/*
 * Packages one RGB frame for the supplied SK9822 strip format:
 * start frame, then [0xE0|brightness, G, R, B] per LED, then 0xFF end frame.
 * Pass POV1_USE_HEADER_BRIGHTNESS to use header->global_brightness.
 */
pov_result_t pov1_build_sk9822_frame(
    const pov1_header_t *header,
    const uint8_t *rgb,
    uint32_t rgb_length,
    uint8_t brightness,
    uint8_t *spi_bytes,
    uint32_t spi_capacity,
    uint32_t *spi_length
);

uint32_t pov1_crc32_begin(void);
uint32_t pov1_crc32_update(uint32_t crc, const uint8_t *data, uint32_t length);
uint32_t pov1_crc32_finish(uint32_t crc);

#ifdef __cplusplus
}
#endif

#endif /* POV_FILE_H */


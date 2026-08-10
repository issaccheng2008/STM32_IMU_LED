#include "pov_file.h"

#include <string.h>

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
}

uint32_t pov1_crc32_begin(void)
{
    return 0xFFFFFFFFu;
}

uint32_t pov1_crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;
    uint32_t bit;

    if (data == NULL) {
        return crc;
    }

    for (i = 0u; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc;
}

uint32_t pov1_crc32_finish(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

pov_result_t pov1_read_header(pov_read_fn read_fn, void *context, pov1_header_t *header)
{
    uint8_t bytes[POV1_HEADER_BYTES];
    uint32_t header_crc;
    uint32_t expected_payload_bytes;
    uint64_t payload_product;

    if ((read_fn == NULL) || (header == NULL)) {
        return POV_ERROR_ARGUMENT;
    }
    if (!read_fn(context, 0u, bytes, POV1_HEADER_BYTES)) {
        return POV_ERROR_READ;
    }
    if ((bytes[0] != 'P') || (bytes[1] != 'O') || (bytes[2] != 'V') || (bytes[3] != '1')) {
        return POV_ERROR_MAGIC;
    }
    if (read_u16_le(&bytes[4]) != 1u) {
        return POV_ERROR_VERSION;
    }
    if ((read_u16_le(&bytes[6]) != POV1_HEADER_BYTES)
        || (bytes[10] != 3u)
        || (bytes[11] != 0u)
        || (bytes[73] != 0u)) {
        return POV_ERROR_FORMAT;
    }

    header_crc = pov1_crc32_begin();
    header_crc = pov1_crc32_update(header_crc, bytes, 76u);
    header_crc = pov1_crc32_finish(header_crc);
    if (header_crc != read_u32_le(&bytes[76])) {
        return POV_ERROR_HEADER_CRC;
    }

    memset(header, 0, sizeof(*header));
    header->led_count = read_u16_le(&bytes[8]);
    header->sample_count = read_u32_le(&bytes[12]);
    header->angle_step_udeg = read_u32_le(&bytes[16]);
    header->payload_bytes = read_u32_le(&bytes[20]);
    header->payload_crc32 = read_u32_le(&bytes[24]);
    header->strip_length_um = read_i32_le(&bytes[28]);
    header->pivot_to_led0_um = read_i32_le(&bytes[32]);
    header->image_center_x_um = read_i32_le(&bytes[36]);
    header->image_center_y_um = read_i32_le(&bytes[40]);
    header->image_width_um = read_u32_le(&bytes[44]);
    header->image_height_um = read_u32_le(&bytes[48]);
    header->image_rotation_mdeg = read_i32_le(&bytes[52]);
    header->zero_angle_mdeg = read_i32_le(&bytes[56]);
    header->flags = read_u32_le(&bytes[60]);
    header->led_pitch_um = read_u32_le(&bytes[64]);
    header->led0_radius_um = read_i32_le(&bytes[68]);
    header->global_brightness = bytes[72];

    payload_product = (uint64_t)header->sample_count * (uint64_t)header->led_count * 3u;
    if ((header->led_count == 0u)
        || (header->sample_count == 0u)
        || (header->angle_step_udeg == 0u)
        || (header->global_brightness > 31u)
        || (payload_product > UINT32_MAX)) {
        return POV_ERROR_FORMAT;
    }
    expected_payload_bytes = (uint32_t)payload_product;
    if (header->payload_bytes != expected_payload_bytes) {
        return POV_ERROR_FORMAT;
    }

    return POV_OK;
}

pov_result_t pov1_validate_payload_crc(
    pov_read_fn read_fn,
    void *context,
    const pov1_header_t *header,
    uint8_t *scratch,
    uint32_t scratch_length)
{
    uint32_t offset;
    uint32_t remaining;
    uint32_t crc;

    if ((read_fn == NULL) || (header == NULL) || (scratch == NULL) || (scratch_length == 0u)) {
        return POV_ERROR_ARGUMENT;
    }

    offset = POV1_HEADER_BYTES;
    remaining = header->payload_bytes;
    crc = pov1_crc32_begin();
    while (remaining > 0u) {
        const uint32_t amount = (remaining < scratch_length) ? remaining : scratch_length;
        if (!read_fn(context, offset, scratch, amount)) {
            return POV_ERROR_READ;
        }
        crc = pov1_crc32_update(crc, scratch, amount);
        offset += amount;
        remaining -= amount;
    }
    crc = pov1_crc32_finish(crc);
    return (crc == header->payload_crc32) ? POV_OK : POV_ERROR_PAYLOAD_CRC;
}

uint32_t pov1_angle_to_sample(const pov1_header_t *header, uint32_t angle_udeg)
{
    uint64_t rounded;
    if ((header == NULL) || (header->sample_count == 0u) || (header->angle_step_udeg == 0u)) {
        return 0u;
    }
    angle_udeg %= POV1_FULL_TURN_UDEG;
    rounded = (uint64_t)angle_udeg + ((uint64_t)header->angle_step_udeg / 2u);
    return (uint32_t)(rounded / header->angle_step_udeg) % header->sample_count;
}

pov_result_t pov1_read_rgb_frame(
    pov_read_fn read_fn,
    void *context,
    const pov1_header_t *header,
    uint32_t sample_index,
    uint8_t *rgb,
    uint32_t rgb_capacity)
{
    uint32_t frame_bytes;
    uint32_t offset;

    if ((read_fn == NULL) || (header == NULL) || (rgb == NULL)) {
        return POV_ERROR_ARGUMENT;
    }
    if (sample_index >= header->sample_count) {
        return POV_ERROR_INDEX;
    }
    frame_bytes = POV1_RGB_FRAME_BYTES(header->led_count);
    if (rgb_capacity < frame_bytes) {
        return POV_ERROR_BUFFER_TOO_SMALL;
    }
    offset = POV1_HEADER_BYTES + sample_index * frame_bytes;
    if (!read_fn(context, offset, rgb, frame_bytes)) {
        return POV_ERROR_READ;
    }
    return POV_OK;
}

pov_result_t pov1_build_sk9822_frame(
    const pov1_header_t *header,
    const uint8_t *rgb,
    uint32_t rgb_length,
    uint8_t brightness,
    uint8_t *spi_bytes,
    uint32_t spi_capacity,
    uint32_t *spi_length)
{
    uint32_t led;
    uint32_t required_rgb;
    uint32_t required_spi;
    uint32_t output;

    if ((header == NULL) || (rgb == NULL) || (spi_bytes == NULL) || (spi_length == NULL)) {
        return POV_ERROR_ARGUMENT;
    }
    required_rgb = POV1_RGB_FRAME_BYTES(header->led_count);
    required_spi = POV1_SK9822_FRAME_BYTES(header->led_count);
    if ((rgb_length < required_rgb) || (spi_capacity < required_spi)) {
        return POV_ERROR_BUFFER_TOO_SMALL;
    }

    if (brightness == POV1_USE_HEADER_BRIGHTNESS) {
        brightness = header->global_brightness;
    }
    if (brightness > 31u) {
        return POV_ERROR_ARGUMENT;
    }

    memset(spi_bytes, 0x00, 4u);
    output = 4u;
    for (led = 0u; led < header->led_count; ++led) {
        const uint32_t input = led * 3u;
        spi_bytes[output++] = (uint8_t)(0xE0u | brightness);
        spi_bytes[output++] = rgb[input + 1u]; /* Green */
        spi_bytes[output++] = rgb[input + 0u]; /* Red */
        spi_bytes[output++] = rgb[input + 2u]; /* Blue */
    }
    memset(&spi_bytes[output], 0xFF, 4u);
    output += 4u;
    *spi_length = output;
    return POV_OK;
}


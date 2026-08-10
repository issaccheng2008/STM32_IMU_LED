#include "../firmware/pov_file.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t *data;
    uint32_t length;
} memory_file_t;

static bool read_at(void *context, uint32_t offset, uint8_t *destination, uint32_t length)
{
    memory_file_t *file = (memory_file_t *)context;
    if ((offset > file->length) || (length > file->length - offset)) {
        return false;
    }
    memcpy(destination, &file->data[offset], length);
    return true;
}

static int fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

int main(void)
{
    FILE *input;
    long file_length;
    memory_file_t file;
    pov1_header_t header;
    uint8_t scratch[7];
    uint8_t rgb[6];
    uint8_t spi[16];
    uint32_t spi_length = 0u;

    input = fopen("tests/fixture.pov", "rb");
    if (input == NULL) return fail("could not open fixture.pov");
    if (fseek(input, 0, SEEK_END) != 0) return fail("seek failed");
    file_length = ftell(input);
    if ((file_length <= 0) || ((unsigned long)file_length > UINT32_MAX)) return fail("bad fixture size");
    if (fseek(input, 0, SEEK_SET) != 0) return fail("rewind failed");
    file.length = (uint32_t)file_length;
    file.data = (uint8_t *)malloc(file.length);
    if (file.data == NULL) return fail("allocation failed");
    if (fread(file.data, 1u, file.length, input) != file.length) return fail("read failed");
    fclose(input);

    if (pov1_read_header(read_at, &file, &header) != POV_OK) return fail("header validation failed");
    if ((header.led_count != 2u) || (header.sample_count != 8u)) return fail("header values differ");
    if (pov1_validate_payload_crc(read_at, &file, &header, scratch, sizeof(scratch)) != POV_OK) return fail("payload CRC failed");
    if (pov1_angle_to_sample(&header, 45000000u) != 1u) return fail("angle lookup failed");
    if (pov1_read_rgb_frame(read_at, &file, &header, 1u, rgb, sizeof(rgb)) != POV_OK) return fail("frame read failed");
    if ((rgb[0] != 0x12u) || (rgb[1] != 0x34u) || (rgb[2] != 0x56u)) return fail("RGB values differ");
    if (pov1_build_sk9822_frame(&header, rgb, sizeof(rgb), POV1_USE_HEADER_BRIGHTNESS,
                                spi, sizeof(spi), &spi_length) != POV_OK) return fail("SK9822 build failed");
    if (spi_length != sizeof(spi)) return fail("SK9822 length differs");
    if ((spi[4] != 0xE3u) || (spi[5] != 0x34u) || (spi[6] != 0x12u) || (spi[7] != 0x56u)) {
        return fail("SK9822 byte order differs");
    }

    free(file.data);
    puts("JavaScript-to-C POV1 compatibility test passed.");
    return 0;
}

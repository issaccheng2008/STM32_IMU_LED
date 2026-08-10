#ifndef WAND_STORAGE_H
#define WAND_STORAGE_H

#include "wand_file.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WAND_STORAGE_FILENAME  "WAND.POV"

typedef enum
{
  WAND_STORAGE_OK = 0,
  WAND_STORAGE_ERROR_ARGUMENT = -100,
  WAND_STORAGE_ERROR_MOUNT = -101,
  WAND_STORAGE_ERROR_OPEN = -102,
  WAND_STORAGE_ERROR_HEADER_READ = -103,
  WAND_STORAGE_ERROR_HEADER_BASE = -120,
  WAND_STORAGE_ERROR_FILE_SIZE = -140,
  WAND_STORAGE_ERROR_CAPACITY = -141,
  WAND_STORAGE_ERROR_PAYLOAD_READ = -142,
  WAND_STORAGE_ERROR_PAYLOAD_BASE = -160,
  WAND_STORAGE_ERROR_CLOSE = -180
} wand_storage_result_t;

/**
 * Mount the FAT volume, load and validate WAND.POV, then unmount it.
 * The payload remains in caller-owned RAM and the card is never accessed by
 * the real-time playback loop.
 */
wand_storage_result_t WAND_StorageLoad(uint8_t *payload,
                                       uint32_t capacity,
                                       wand1_header_t *header);

#ifdef __cplusplus
}
#endif

#endif /* WAND_STORAGE_H */

#include "wand_storage.h"

#include "ff.h"

static FATFS wand_file_system;

wand_storage_result_t WAND_StorageLoad(uint8_t *payload,
                                       uint32_t capacity,
                                       wand1_header_t *header)
{
  FIL file;
  FRESULT fat_result;
  UINT bytes_read;
  uint8_t header_bytes[WAND1_HEADER_BYTES];
  wand_result_t wand_result;
  wand_storage_result_t result = WAND_STORAGE_OK;
  uint8_t file_is_open = 0U;

  if ((payload == NULL) || (header == NULL))
    return WAND_STORAGE_ERROR_ARGUMENT;

  fat_result = f_mount(&wand_file_system, "", 1U);
  if (fat_result != FR_OK)
    return WAND_STORAGE_ERROR_MOUNT;

  fat_result = f_open(&file, WAND_STORAGE_FILENAME, FA_READ);
  if (fat_result != FR_OK)
  {
    result = WAND_STORAGE_ERROR_OPEN;
    goto unmount;
  }
  file_is_open = 1U;

  fat_result = f_read(&file,
                      header_bytes,
                      WAND1_HEADER_BYTES,
                      &bytes_read);
  if ((fat_result != FR_OK) || (bytes_read != WAND1_HEADER_BYTES))
  {
    result = WAND_STORAGE_ERROR_HEADER_READ;
    goto close;
  }

  wand_result = WAND_ParseHeader(header_bytes,
                                 WAND1_HEADER_BYTES,
                                 header);
  if (wand_result != WAND_OK)
  {
    result = (wand_storage_result_t)
        (WAND_STORAGE_ERROR_HEADER_BASE - (int32_t)wand_result);
    goto close;
  }

  if ((uint32_t)f_size(&file) != header->file_bytes)
  {
    result = WAND_STORAGE_ERROR_FILE_SIZE;
    goto close;
  }

  if (header->payload_bytes > capacity)
  {
    result = WAND_STORAGE_ERROR_CAPACITY;
    goto close;
  }

  fat_result = f_read(&file,
                      payload,
                      header->payload_bytes,
                      &bytes_read);
  if ((fat_result != FR_OK) || (bytes_read != header->payload_bytes))
  {
    result = WAND_STORAGE_ERROR_PAYLOAD_READ;
    goto close;
  }

  wand_result = WAND_ValidatePayload(header,
                                     payload,
                                     header->payload_bytes);
  if (wand_result != WAND_OK)
  {
    result = (wand_storage_result_t)
        (WAND_STORAGE_ERROR_PAYLOAD_BASE - (int32_t)wand_result);
  }

close:
  if (file_is_open != 0U)
  {
    fat_result = f_close(&file);
    if ((fat_result != FR_OK) && (result == WAND_STORAGE_OK))
      result = WAND_STORAGE_ERROR_CLOSE;
  }

unmount:
  (void)f_mount(NULL, "", 0U);
  return result;
}

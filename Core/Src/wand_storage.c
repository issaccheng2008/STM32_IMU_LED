#include "wand_storage.h"

#include "ff.h"

#include <string.h>

static FATFS wand_file_system;
static wand_storage_diagnostics_t wand_storage_diagnostics;

void WAND_StorageGetDiagnostics(wand_storage_diagnostics_t *diagnostics)
{
  if (diagnostics != NULL)
    *diagnostics = wand_storage_diagnostics;
}

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

  memset(&wand_storage_diagnostics, 0, sizeof(wand_storage_diagnostics));
  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_ARGUMENT;
  wand_storage_diagnostics.fatfs_result = (int32_t)FR_OK;
  wand_storage_diagnostics.wand_result = WAND_OK;

  if ((payload == NULL) || (header == NULL))
    return WAND_STORAGE_ERROR_ARGUMENT;

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_MOUNT;
  fat_result = f_mount(&wand_file_system, "", 1U);
  wand_storage_diagnostics.fatfs_result = (int32_t)fat_result;
  if (fat_result != FR_OK)
    return WAND_STORAGE_ERROR_MOUNT;

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_OPEN;
  fat_result = f_open(&file, WAND_STORAGE_FILENAME, FA_READ);
  wand_storage_diagnostics.fatfs_result = (int32_t)fat_result;
  if (fat_result != FR_OK)
  {
    result = WAND_STORAGE_ERROR_OPEN;
    goto unmount;
  }
  file_is_open = 1U;
  wand_storage_diagnostics.file_bytes = (uint32_t)f_size(&file);

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_HEADER_READ;
  fat_result = f_read(&file,
                      header_bytes,
                      WAND1_HEADER_BYTES,
                      &bytes_read);
  wand_storage_diagnostics.fatfs_result = (int32_t)fat_result;
  wand_storage_diagnostics.bytes_read = (uint32_t)bytes_read;
  if ((fat_result != FR_OK) || (bytes_read != WAND1_HEADER_BYTES))
  {
    result = WAND_STORAGE_ERROR_HEADER_READ;
    goto close;
  }

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_HEADER_VALIDATE;
  wand_result = WAND_ParseHeader(header_bytes,
                                 WAND1_HEADER_BYTES,
                                 header);
  wand_storage_diagnostics.wand_result = wand_result;
  if (wand_result != WAND_OK)
  {
    result = (wand_storage_result_t)
        (WAND_STORAGE_ERROR_HEADER_BASE - (int32_t)wand_result);
    goto close;
  }

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_FILE_SIZE;
  wand_storage_diagnostics.expected_file_bytes = header->file_bytes;
  if ((uint32_t)f_size(&file) != header->file_bytes)
  {
    result = WAND_STORAGE_ERROR_FILE_SIZE;
    goto close;
  }

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_CAPACITY;
  if (header->payload_bytes > capacity)
  {
    result = WAND_STORAGE_ERROR_CAPACITY;
    goto close;
  }

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_PAYLOAD_READ;
  fat_result = f_read(&file,
                      payload,
                      header->payload_bytes,
                      &bytes_read);
  wand_storage_diagnostics.fatfs_result = (int32_t)fat_result;
  wand_storage_diagnostics.bytes_read = (uint32_t)bytes_read;
  if ((fat_result != FR_OK) || (bytes_read != header->payload_bytes))
  {
    result = WAND_STORAGE_ERROR_PAYLOAD_READ;
    goto close;
  }

  wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_PAYLOAD_VALIDATE;
  wand_result = WAND_ValidatePayload(header,
                                     payload,
                                     header->payload_bytes);
  wand_storage_diagnostics.wand_result = wand_result;
  if (wand_result != WAND_OK)
  {
    result = (wand_storage_result_t)
        (WAND_STORAGE_ERROR_PAYLOAD_BASE - (int32_t)wand_result);
  }

close:
  if (file_is_open != 0U)
  {
    if (result == WAND_STORAGE_OK)
      wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_CLOSE;
    fat_result = f_close(&file);
    if ((fat_result != FR_OK) && (result == WAND_STORAGE_OK))
      wand_storage_diagnostics.fatfs_result = (int32_t)fat_result;
    if ((fat_result != FR_OK) && (result == WAND_STORAGE_OK))
      result = WAND_STORAGE_ERROR_CLOSE;
  }

unmount:
  (void)f_mount(NULL, "", 0U);
  if (result == WAND_STORAGE_OK)
    wand_storage_diagnostics.phase = WAND_STORAGE_PHASE_COMPLETE;
  return result;
}

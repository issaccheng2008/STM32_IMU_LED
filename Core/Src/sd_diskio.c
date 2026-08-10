#include "diskio.h"

#include "main.h"

#include <stdint.h>

#define SD_DISK_DRIVE_NUMBER  0U
#define SD_DISK_TIMEOUT_MS    5000U

extern SD_HandleTypeDef hsd1;

static DSTATUS sd_wait_until_ready(uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
      return STA_NOINIT;
  }

  return 0U;
}

DSTATUS disk_initialize(BYTE physical_drive)
{
  if (physical_drive != SD_DISK_DRIVE_NUMBER)
    return STA_NOINIT;

  if (HAL_SD_GetState(&hsd1) == HAL_SD_STATE_RESET)
  {
    if (HAL_SD_Init(&hsd1) != HAL_OK)
      return STA_NOINIT;
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
    return STA_NOINIT;

  return sd_wait_until_ready(SD_DISK_TIMEOUT_MS);
}

DSTATUS disk_status(BYTE physical_drive)
{
  if ((physical_drive != SD_DISK_DRIVE_NUMBER) ||
      (HAL_SD_GetState(&hsd1) == HAL_SD_STATE_RESET))
  {
    return STA_NOINIT;
  }

  return (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
             ? 0U
             : STA_NOINIT;
}

DRESULT disk_read(BYTE physical_drive,
                  BYTE *buffer,
                  DWORD sector,
                  UINT count)
{
  if ((physical_drive != SD_DISK_DRIVE_NUMBER) ||
      (buffer == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if (HAL_SD_ReadBlocks(&hsd1,
                        buffer,
                        (uint32_t)sector,
                        (uint32_t)count,
                        SD_DISK_TIMEOUT_MS) != HAL_OK)
  {
    return RES_ERROR;
  }

  return (sd_wait_until_ready(SD_DISK_TIMEOUT_MS) == 0U)
             ? RES_OK
             : RES_ERROR;
}

DRESULT disk_write(BYTE physical_drive,
                   const BYTE *buffer,
                   DWORD sector,
                   UINT count)
{
  (void)physical_drive;
  (void)buffer;
  (void)sector;
  (void)count;
  return RES_WRPRT;
}

DRESULT disk_ioctl(BYTE physical_drive, BYTE command, void *buffer)
{
  if (physical_drive != SD_DISK_DRIVE_NUMBER)
    return RES_PARERR;

  switch (command)
  {
    case CTRL_SYNC:
      return (sd_wait_until_ready(SD_DISK_TIMEOUT_MS) == 0U)
                 ? RES_OK
                 : RES_ERROR;

    case GET_SECTOR_COUNT:
      if (buffer == NULL)
        return RES_PARERR;
      *(DWORD *)buffer = (DWORD)hsd1.SdCard.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      if (buffer == NULL)
        return RES_PARERR;
      *(WORD *)buffer = (WORD)hsd1.SdCard.LogBlockSize;
      return RES_OK;

    case GET_BLOCK_SIZE:
      if (buffer == NULL)
        return RES_PARERR;
      *(DWORD *)buffer = 1U;
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

DWORD get_fattime(void)
{
  return 0U;
}

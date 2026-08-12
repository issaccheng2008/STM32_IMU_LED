/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_calibration.h"
#include "imu_calibration_session.h"
#include "imu_orientation.h"
#include "lsm6dsv320x_reg.h"
#include "sk9822.h"
#include "wand_file.h"
#include "wand_storage.h"

#include <math.h>
#if defined(DEBUG)
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_OUTPUT_DATA_RATE_HZ 1920.0f
#define IMU_ORIENTATION_TIMER_SECONDS_PER_TICK 1.0e-6f
#define IMU_GYRO_ACCEL_BURST_BYTES 12U
#define IMU_TEMPERATURE_UPDATE_PERIOD_MS 67U
#define WAND_DEBUG_STATUS_PERIOD_MS 1000U

#if (WAND1_LED_COUNT != SK9822_LED_COUNT) || \
    (WAND1_SK9822_FRAME_BYTES != SK9822_FRAME_BYTES)
#error "WAND1 and SK9822 layouts must stay identical"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static stmdev_ctx_t imu_ctx;
static uint8_t imu_orientation_pending_mask = 0U;
static uint8_t imu_orientation_timestamp_valid = 0U;
static uint32_t imu_orientation_last_timestamp_us = 0U;
static uint32_t imu_temperature_last_read_ms = 0U;
static uint32_t wand_last_orientation_update_count = 0U;
static uint32_t wand_requested_sample_index = 0U;
static uint8_t wand_requested_sample_valid = 0U;
static const uint8_t *wand_dma_pending_frame = NULL;
static uint32_t wand_dma_pending_sample_index = 0U;
static uint32_t wand_dma_active_sample_index = 0U;
static volatile uint32_t wand_dma_completed_sample_index = 0U;
static volatile uint32_t wand_dma_callback_error = HAL_SPI_ERROR_NONE;
static volatile uint8_t wand_dma_completion_pending = 0U;
static volatile uint8_t wand_dma_error_pending = 0U;

/* The largest legal WAND1 payload is 1,801 complete 148-byte SK9822
 * frames (266,548 bytes).  STM32H743 RAM_D1 has room for it, so playback
 * performs no SD-card reads after startup. */
static uint8_t wand_payload[WAND1_MAX_PAYLOAD_BYTES]
    __attribute__((aligned(32)));
static wand1_header_t wand_header;
static uint8_t wand_blank_frame[SK9822_FRAME_BYTES];

/** SD/FatFs/WAND preload result. Zero means WAND.POV is ready in RAM. */
volatile int32_t wand_storage_status = WAND_STORAGE_ERROR_ARGUMENT;
/** Nonzero after a complete, CRC-checked WAND.POV has been preloaded. */
volatile uint8_t wand_loaded = 0U;
/** Current wand angle: -90 deg counterclockwise, 0 upright, +90 clockwise. */
volatile float wand_angle_deg = 0.0f;
/** Nearest frame currently selected from the preloaded WAND1 payload. */
volatile uint32_t wand_sample_index = 0U;
/** Number of runtime SK9822 DMA frames completed successfully. */
volatile uint32_t wand_frame_update_count = 0U;
/** Number of SK9822 transmissions attempted, including the boot self-test. */
volatile uint32_t wand_frame_attempt_count = 0U;
/** Number of SK9822 hardware failures; normal HAL_BUSY deferrals are excluded. */
volatile uint32_t wand_spi_error_count = 0U;
/** Most recent HAL SPI result returned while updating the strip. */
volatile int32_t wand_spi_status = HAL_OK;
/** Detailed HAL SPI error flags for the most recent transmission. */
volatile uint32_t wand_spi_hal_error = HAL_SPI_ERROR_NONE;
/** Nonzero while the one permitted SPI1 TX DMA transfer is active. */
volatile uint8_t wand_dma_active = 0U;
/** Nonzero when a newer frame is waiting to replace the active frame. */
volatile uint8_t wand_dma_pending = 0U;
/** Number of requests deferred because DMA or callback bookkeeping was busy. */
volatile uint32_t wand_dma_busy_count = 0U;
/** Number of obsolete pending frames replaced by a newer request. */
volatile uint32_t wand_dma_coalesced_frame_count = 0U;
/** Number of SPI DMA completion callbacks serviced by the main loop. */
volatile uint32_t wand_dma_completion_count = 0U;
/** Number of SPI DMA error callbacks serviced by the main loop. */
volatile uint32_t wand_dma_error_callback_count = 0U;
/** Most recent sample whose full DMA transfer completed successfully. */
volatile uint32_t wand_dma_last_completed_sample_index = 0U;

#if defined(DEBUG)
/** Number of LEDs with nonzero RGB data in the currently selected frame. */
volatile uint32_t wand_selected_lit_led_count = 0U;
/** Set to zero in Live Expressions to stop the one-line-per-second output. */
volatile uint8_t wand_debug_periodic_output_enabled = 0U;
/** Set to zero at the main() breakpoint to skip the RGB boot self-test. */
volatile uint8_t wand_debug_led_self_test_on_boot = 0U;
static uint8_t wand_debug_console_ready = 0U;
static const char *wand_debug_boot_stage = "reset";
#endif

/** Last IMU initialization or read result: 0 means success. */
volatile int32_t imu_status = -1;

/** WHO_AM_I register value; an LSM6DSV320X returns 0x73. */
volatile uint8_t imu_who_am_i = 0U;

/** Active 8-bit I2C address; selected automatically from the SDO/SA0 pin. */
volatile uint8_t imu_i2c_address = LSM6DSV320X_I2C_ADD_L;

/** Number of polling cycles in which at least one new sensor value was read. */
volatile uint32_t imu_sample_count = 0U;
/** Number of combined 12-byte gyro + low-g accelerometer I2C bursts. */
volatile uint32_t imu_gyro_accel_burst_count = 0U;
/** Number of fallback single-sensor reads when only one 1920 Hz DRDY was set. */
volatile uint32_t imu_gyro_accel_split_read_count = 0U;
/** Number of separately serviced 480 Hz high-g samples. */
volatile uint32_t imu_high_g_read_count = 0U;
/** Number of throttled temperature reads (nominally about 15 Hz). */
volatile uint32_t imu_temperature_read_count = 0U;

/** Low-g accelerometer X-axis register value (raw signed counts). */
volatile int16_t imu_accel_x_raw = 0;
/** Low-g accelerometer Y-axis register value (raw signed counts). */
volatile int16_t imu_accel_y_raw = 0;
/** Low-g accelerometer Z-axis register value (raw signed counts). */
volatile int16_t imu_accel_z_raw = 0;
/** Low-g X-axis acceleration in milligravity (mg), configured for +/-4 g. */
volatile float imu_accel_x_mg = 0.0f;
/** Low-g Y-axis acceleration in milligravity (mg), configured for +/-4 g. */
volatile float imu_accel_y_mg = 0.0f;
/** Low-g Z-axis acceleration in milligravity (mg), configured for +/-4 g. */
volatile float imu_accel_z_mg = 0.0f;
/** Calibrated low-g X-axis acceleration in milligravity (mg). */
volatile float imu_accel_x_calibrated_mg = 0.0f;
/** Calibrated low-g Y-axis acceleration in milligravity (mg). */
volatile float imu_accel_y_calibrated_mg = 0.0f;
/** Calibrated low-g Z-axis acceleration in milligravity (mg). */
volatile float imu_accel_z_calibrated_mg = 0.0f;

/** Gyroscope X-axis register value (raw signed counts). */
volatile int16_t imu_gyro_x_raw = 0;
/** Gyroscope Y-axis register value (raw signed counts). */
volatile int16_t imu_gyro_y_raw = 0;
/** Gyroscope Z-axis register value (raw signed counts). */
volatile int16_t imu_gyro_z_raw = 0;
/** X-axis angular rate in degrees per second, configured for +/-4000 dps. */
volatile float imu_gyro_x_dps = 0.0f;
/** Y-axis angular rate in degrees per second, configured for +/-4000 dps. */
volatile float imu_gyro_y_dps = 0.0f;
/** Z-axis angular rate in degrees per second, configured for +/-4000 dps. */
volatile float imu_gyro_z_dps = 0.0f;
/** Calibrated X-axis angular rate in degrees per second. */
volatile float imu_gyro_x_calibrated_dps = 0.0f;
/** Calibrated Y-axis angular rate in degrees per second. */
volatile float imu_gyro_y_calibrated_dps = 0.0f;
/** Calibrated Z-axis angular rate in degrees per second. */
volatile float imu_gyro_z_calibrated_dps = 0.0f;

/** High-g accelerometer X-axis register value (raw signed counts). */
volatile int16_t imu_high_g_x_raw = 0;
/** High-g accelerometer Y-axis register value (raw signed counts). */
volatile int16_t imu_high_g_y_raw = 0;
/** High-g accelerometer Z-axis register value (raw signed counts). */
volatile int16_t imu_high_g_z_raw = 0;
/** High-g X-axis acceleration in milligravity (mg), configured for +/-32 g. */
volatile float imu_high_g_x_mg = 0.0f;
/** High-g Y-axis acceleration in milligravity (mg), configured for +/-32 g. */
volatile float imu_high_g_y_mg = 0.0f;
/** High-g Z-axis acceleration in milligravity (mg), configured for +/-32 g. */
volatile float imu_high_g_z_mg = 0.0f;
/** Calibrated high-g X-axis acceleration in milligravity (mg). */
volatile float imu_high_g_x_calibrated_mg = 0.0f;
/** Calibrated high-g Y-axis acceleration in milligravity (mg). */
volatile float imu_high_g_y_calibrated_mg = 0.0f;
/** Calibrated high-g Z-axis acceleration in milligravity (mg). */
volatile float imu_high_g_z_calibrated_mg = 0.0f;

/** Temperature output register value (raw signed counts). */
volatile int16_t imu_temperature_raw = 0;
/** IMU die temperature in degrees Celsius. */
volatile float imu_temperature_c = 0.0f;

/**
 * Set this to 1 in Live Expressions while stopped at main() to collect a new
 * calibration.  It defaults to 0 so normal boots immediately use the constants
 * compiled into imu_calibration.c.
 */
volatile uint8_t imu_run_calibration_on_boot = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static int32_t IMU_Init(void);
static int32_t IMU_ReadAll(void);
static int32_t IMU_ReadSensors(imu_calibration_measurement_t *measurement);
static void WAND_PlayCurrentAngle(void);
static void WAND_ServiceLedDma(void);
static HAL_StatusTypeDef WAND_RequestDmaFrame(const uint8_t *frame,
                                              uint32_t sample_index);
static HAL_StatusTypeDef WAND_TransmitTracked(const uint8_t *frame);
#if defined(DEBUG)
static uint32_t WAND_CountLitLeds(const uint8_t *frame);
static void WAND_DebugPrintStorageStatus(void);
static void WAND_DebugRunLedSelfTest(void);
static void WAND_DebugPoll(void);
extern void initialise_monitor_handles(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *buffer, uint16_t length)
{
  HAL_StatusTypeDef result;

  result = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)handle,
                             imu_i2c_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)buffer,
                             length,
                             5U);

  return (result == HAL_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *buffer, uint16_t length)
{
  HAL_StatusTypeDef result;

  result = HAL_I2C_Mem_Read((I2C_HandleTypeDef *)handle,
                            imu_i2c_address,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            length,
                            5U);

  return (result == HAL_OK) ? 0 : -1;
}

static void platform_delay(uint32_t milliseconds)
{
  HAL_Delay(milliseconds);
}

static int32_t IMU_Init(void)
{
  uint8_t who_am_i = 0U;

  imu_ctx.write_reg = platform_write;
  imu_ctx.read_reg = platform_read;
  imu_ctx.mdelay = platform_delay;
  imu_ctx.handle = &hi2c1;
  imu_ctx.priv_data = NULL;

  HAL_Delay(20U);

  /* SDO/SA0 selects one of two I2C addresses, so try both. */
  imu_i2c_address = LSM6DSV320X_I2C_ADD_L;
  if ((lsm6dsv320x_device_id_get(&imu_ctx, &who_am_i) != 0) ||
      (who_am_i != LSM6DSV320X_ID))
  {
    imu_i2c_address = LSM6DSV320X_I2C_ADD_H;
    if ((lsm6dsv320x_device_id_get(&imu_ctx, &who_am_i) != 0) ||
        (who_am_i != LSM6DSV320X_ID))
    {
      imu_who_am_i = who_am_i;
      return -1;
    }
  }

  imu_who_am_i = who_am_i;

  if (lsm6dsv320x_sw_por(&imu_ctx) != 0)
    return -2;

  HAL_Delay(10U);

  if (lsm6dsv320x_block_data_update_set(&imu_ctx, PROPERTY_ENABLE) != 0)
    return -3;

  if (lsm6dsv320x_auto_increment_set(&imu_ctx, PROPERTY_ENABLE) != 0)
    return -4;

  if (lsm6dsv320x_xl_full_scale_set(&imu_ctx, LSM6DSV320X_4g) != 0)
    return -5;

  if (lsm6dsv320x_gy_full_scale_set(&imu_ctx, LSM6DSV320X_4000dps) != 0)
    return -6;

  if (lsm6dsv320x_hg_xl_full_scale_set(&imu_ctx, LSM6DSV320X_32g) != 0)
    return -7;

  /* 1.92 kHz is the highest practical polling rate over 1 MHz I2C while
   * leaving enough bandwidth for status, acceleration, and gyro transfers. */
  if (lsm6dsv320x_xl_setup(&imu_ctx,
                           LSM6DSV320X_ODR_AT_1920Hz,
                           LSM6DSV320X_XL_HIGH_PERFORMANCE_MD) != 0)
  {
    return -8;
  }

  /* The +/-4000 dps range avoids clipping a 10 Hz, +/-35 degree wave. */
  if (lsm6dsv320x_gy_setup(&imu_ctx,
                           LSM6DSV320X_ODR_AT_1920Hz,
                           LSM6DSV320X_GY_HIGH_PERFORMANCE_MD) != 0)
  {
    return -9;
  }

  /* High-g accelerometer: keep at 480 samples/second. */
  if (lsm6dsv320x_hg_xl_data_rate_set(
          &imu_ctx,
          LSM6DSV320X_HG_XL_ODR_AT_480Hz,
          PROPERTY_ENABLE) != 0)
  {
    return -10;
  }

  /* Use LPF1 directly.  Bypassing LPF2 removes the former medium-bandwidth
   * filter delay; XL_ULTRA_LIGHT is retained for a regeneration-safe setup. */
  if (lsm6dsv320x_filt_xl_setup(
          &imu_ctx,
          LSM6DSV320X_XL_FILT_LP_LPF1,
          LSM6DSV320X_XL_ULTRA_LIGHT,
          0U) != 0)
  {
    return -11;
  }

  /* Minimum LPF1 filtering gives the lowest sensor-side gyro latency. */
  if (lsm6dsv320x_filt_gy_lp1_bandwidth_set(
          &imu_ctx,
          LSM6DSV320X_GY_ULTRA_LIGHT) != 0)
  {
    return -12;
  }

  if (lsm6dsv320x_filt_gy_lp1_set(
          &imu_ctx,
          PROPERTY_ENABLE) != 0)
  {
    return -13;
  }

  /* Allow the newly enabled high-performance paths to settle. */
  HAL_Delay(50U);
  return 0;
}

static int16_t IMU_DecodeLittleEndianInt16(const uint8_t *bytes)
{
  return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void IMU_UpdateLowG(const int16_t raw_axes[3],
                           imu_calibration_measurement_t *measurement)
{
  const imu_vector3_t raw_mg = {
      lsm6dsv320x_from_fs4_to_mg(raw_axes[0]),
      lsm6dsv320x_from_fs4_to_mg(raw_axes[1]),
      lsm6dsv320x_from_fs4_to_mg(raw_axes[2])};
  imu_vector3_t calibrated_mg;

  imu_accel_x_raw = raw_axes[0];
  imu_accel_y_raw = raw_axes[1];
  imu_accel_z_raw = raw_axes[2];
  imu_accel_x_mg = raw_mg.x;
  imu_accel_y_mg = raw_mg.y;
  imu_accel_z_mg = raw_mg.z;
  IMU_AccelApplyCalibration(&imu_low_g_calibration,
                            &raw_mg, &calibrated_mg);
  imu_accel_x_calibrated_mg = calibrated_mg.x;
  imu_accel_y_calibrated_mg = calibrated_mg.y;
  imu_accel_z_calibrated_mg = calibrated_mg.z;

  if (measurement != NULL)
  {
    measurement->low_g_mg = raw_mg;
    measurement->ready_mask |= IMU_SAMPLE_LOW_G_READY;
  }
}

static void IMU_UpdateGyro(const int16_t raw_axes[3],
                           imu_calibration_measurement_t *measurement)
{
  const imu_vector3_t raw_dps = {
      lsm6dsv320x_from_fs4000_to_mdps(raw_axes[0]) / 1000.0f,
      lsm6dsv320x_from_fs4000_to_mdps(raw_axes[1]) / 1000.0f,
      lsm6dsv320x_from_fs4000_to_mdps(raw_axes[2]) / 1000.0f};
  imu_vector3_t calibrated_dps;

  imu_gyro_x_raw = raw_axes[0];
  imu_gyro_y_raw = raw_axes[1];
  imu_gyro_z_raw = raw_axes[2];
  imu_gyro_x_dps = raw_dps.x;
  imu_gyro_y_dps = raw_dps.y;
  imu_gyro_z_dps = raw_dps.z;
  IMU_GyroApplyCalibration(&imu_gyro_calibration,
                           &raw_dps, &calibrated_dps);
  imu_gyro_x_calibrated_dps = calibrated_dps.x;
  imu_gyro_y_calibrated_dps = calibrated_dps.y;
  imu_gyro_z_calibrated_dps = calibrated_dps.z;

  if (measurement != NULL)
  {
    measurement->gyro_dps = raw_dps;
    measurement->ready_mask |= IMU_SAMPLE_GYRO_READY;
  }
}

static void IMU_UpdateHighG(const int16_t raw_axes[3],
                            imu_calibration_measurement_t *measurement)
{
  const imu_vector3_t raw_mg = {
      lsm6dsv320x_from_fs32_to_mg(raw_axes[0]),
      lsm6dsv320x_from_fs32_to_mg(raw_axes[1]),
      lsm6dsv320x_from_fs32_to_mg(raw_axes[2])};
  imu_vector3_t calibrated_mg;

  imu_high_g_x_raw = raw_axes[0];
  imu_high_g_y_raw = raw_axes[1];
  imu_high_g_z_raw = raw_axes[2];
  imu_high_g_x_mg = raw_mg.x;
  imu_high_g_y_mg = raw_mg.y;
  imu_high_g_z_mg = raw_mg.z;
  IMU_AccelApplyCalibration(&imu_high_g_calibration,
                            &raw_mg, &calibrated_mg);
  imu_high_g_x_calibrated_mg = calibrated_mg.x;
  imu_high_g_y_calibrated_mg = calibrated_mg.y;
  imu_high_g_z_calibrated_mg = calibrated_mg.z;

  if (measurement != NULL)
  {
    measurement->high_g_mg = raw_mg;
    measurement->ready_mask |= IMU_SAMPLE_HIGH_G_READY;
  }
}

static void IMU_UpdateOrientation(
    const imu_calibration_measurement_t *measurement)
{
  imu_orientation_pending_mask |=
      measurement->ready_mask &
      (IMU_SAMPLE_LOW_G_READY | IMU_SAMPLE_GYRO_READY);

  if ((imu_orientation_pending_mask &
       (IMU_SAMPLE_LOW_G_READY | IMU_SAMPLE_GYRO_READY)) ==
      (IMU_SAMPLE_LOW_G_READY | IMU_SAMPLE_GYRO_READY))
  {
    const uint32_t timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);
    float sample_period_s = 1.0f / IMU_OUTPUT_DATA_RATE_HZ;

    if (imu_orientation_timestamp_valid != 0U)
    {
      /* Unsigned subtraction also handles the 32-bit timer wrapping. */
      sample_period_s =
          (float)(timestamp_us - imu_orientation_last_timestamp_us) *
          IMU_ORIENTATION_TIMER_SECONDS_PER_TICK;
    }

    imu_orientation_last_timestamp_us = timestamp_us;
    imu_orientation_timestamp_valid = 1U;
    imu_orientation_pending_mask = 0U;

    IMU_OrientationUpdate(imu_gyro_x_calibrated_dps,
                          imu_gyro_y_calibrated_dps,
                          imu_gyro_z_calibrated_dps,
                          imu_accel_x_calibrated_mg,
                          imu_accel_y_calibrated_mg,
                          imu_accel_z_calibrated_mg,
                          sample_period_s);
  }
}

/* Calibration deliberately retains independent sensor reads.  Its acquisition
 * callback needs each source and is kept separate from runtime scheduling. */
static int32_t IMU_ReadSensors(imu_calibration_measurement_t *measurement)
{
  lsm6dsv320x_data_ready_t data_ready = {0};
  int16_t raw_axes[3];
  int16_t raw_temperature;
  uint8_t updated = 0U;

  if (measurement != NULL)
    *measurement = (imu_calibration_measurement_t){0};

  if (lsm6dsv320x_flag_data_ready_get(&imu_ctx, &data_ready) != 0)
    return -20;

  if (data_ready.drdy_xl != 0U)
  {
    if (lsm6dsv320x_acceleration_raw_get(&imu_ctx, raw_axes) != 0)
      return -21;
    IMU_UpdateLowG(raw_axes, measurement);
    updated = 1U;
  }

  if (data_ready.drdy_gy != 0U)
  {
    if (lsm6dsv320x_angular_rate_raw_get(&imu_ctx, raw_axes) != 0)
      return -22;
    IMU_UpdateGyro(raw_axes, measurement);
    updated = 1U;
  }

  if (data_ready.drdy_hgxl != 0U)
  {
    if (lsm6dsv320x_hg_acceleration_raw_get(&imu_ctx, raw_axes) != 0)
      return -23;
    IMU_UpdateHighG(raw_axes, measurement);
    updated = 1U;
  }

  if (data_ready.drdy_temp != 0U)
  {
    if (lsm6dsv320x_temperature_raw_get(&imu_ctx, &raw_temperature) != 0)
      return -24;
    imu_temperature_raw = raw_temperature;
    imu_temperature_c = lsm6dsv320x_from_lsb_to_celsius(raw_temperature);
    updated = 1U;
  }

  if (updated != 0U)
    imu_sample_count++;

  return 0;
}

static int32_t IMU_ReadAll(void)
{
  lsm6dsv320x_data_ready_t data_ready = {0};
  imu_calibration_measurement_t measurement = {0};
  uint8_t gyro_accel_bytes[IMU_GYRO_ACCEL_BURST_BYTES];
  int16_t raw_accel[3];
  int16_t raw_gyro[3];
  int16_t raw_axes[3];
  int16_t raw_temperature;
  uint8_t updated = 0U;

  if (lsm6dsv320x_flag_data_ready_get(&imu_ctx, &data_ready) != 0)
    return -20;

  if ((data_ready.drdy_gy != 0U) && (data_ready.drdy_xl != 0U))
  {
    /* OUTX_L_G..OUTZ_H_A are contiguous and the device auto-increment is on.
     * Byte order matches the ST driver's individual raw-get functions. */
    if (lsm6dsv320x_read_reg(&imu_ctx, LSM6DSV320X_OUTX_L_G,
                             gyro_accel_bytes,
                             IMU_GYRO_ACCEL_BURST_BYTES) != 0)
    {
      return -25;
    }

    raw_gyro[0] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[0]);
    raw_gyro[1] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[2]);
    raw_gyro[2] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[4]);
    raw_accel[0] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[6]);
    raw_accel[1] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[8]);
    raw_accel[2] = IMU_DecodeLittleEndianInt16(&gyro_accel_bytes[10]);
    IMU_UpdateGyro(raw_gyro, &measurement);
    IMU_UpdateLowG(raw_accel, &measurement);
    imu_gyro_accel_burst_count++;
    updated = 1U;
  }
  else
  {
    /* Preserve data if the two same-rate DRDY bits are briefly out of phase. */
    if (data_ready.drdy_xl != 0U)
    {
      if (lsm6dsv320x_acceleration_raw_get(&imu_ctx, raw_axes) != 0)
        return -21;
      IMU_UpdateLowG(raw_axes, &measurement);
      imu_gyro_accel_split_read_count++;
      updated = 1U;
    }

    if (data_ready.drdy_gy != 0U)
    {
      if (lsm6dsv320x_angular_rate_raw_get(&imu_ctx, raw_axes) != 0)
        return -22;
      IMU_UpdateGyro(raw_axes, &measurement);
      imu_gyro_accel_split_read_count++;
      updated = 1U;
    }
  }

  /* Update Fusion immediately; lower-rate auxiliary reads happen afterwards. */
  IMU_UpdateOrientation(&measurement);

  if (data_ready.drdy_hgxl != 0U)
  {
    if (lsm6dsv320x_hg_acceleration_raw_get(&imu_ctx, raw_axes) != 0)
      return -23;
    IMU_UpdateHighG(raw_axes, NULL);
    imu_high_g_read_count++;
    updated = 1U;
  }

  if ((data_ready.drdy_temp != 0U) &&
      ((uint32_t)(HAL_GetTick() - imu_temperature_last_read_ms) >=
       IMU_TEMPERATURE_UPDATE_PERIOD_MS))
  {
    if (lsm6dsv320x_temperature_raw_get(&imu_ctx, &raw_temperature) != 0)
      return -24;
    imu_temperature_raw = raw_temperature;
    imu_temperature_c = lsm6dsv320x_from_lsb_to_celsius(raw_temperature);
    imu_temperature_last_read_ms = HAL_GetTick();
    imu_temperature_read_count++;
    updated = 1U;
  }

  if (updated != 0U)
    imu_sample_count++;

  return 0;
}

#if defined(DEBUG)
static uint32_t WAND_CountLitLeds(const uint8_t *frame)
{
  uint32_t led;
  uint32_t lit_leds = 0U;

  if (frame == NULL)
    return 0U;

  for (led = 0U; led < SK9822_LED_COUNT; led++)
  {
    const uint32_t offset = SK9822_START_FRAME_BYTES +
                            led * SK9822_BYTES_PER_LED;
    const uint8_t brightness = frame[offset] & 0x1FU;

    if ((brightness != 0U) &&
        ((frame[offset + 1U] != 0U) ||
         (frame[offset + 2U] != 0U) ||
         (frame[offset + 3U] != 0U)))
    {
      lit_leds++;
    }
  }

  return lit_leds;
}
#endif

static HAL_StatusTypeDef WAND_TransmitTracked(const uint8_t *frame)
{
  HAL_StatusTypeDef status;

  wand_frame_attempt_count++;
  status = SK9822_TransmitFrame(&hspi1, frame, SK9822_FRAME_BYTES);
  wand_spi_status = (int32_t)status;
  wand_spi_hal_error = HAL_SPI_GetError(&hspi1);
  if ((status != HAL_OK) && (status != HAL_BUSY))
    wand_spi_error_count++;

  return status;
}

static void WAND_StoreLatestPending(const uint8_t *frame,
                                    uint32_t sample_index)
{
  if (wand_dma_pending != 0U)
    wand_dma_coalesced_frame_count++;

  wand_dma_pending_frame = frame;
  wand_dma_pending_sample_index = sample_index;
  wand_dma_pending = 1U;
}

static HAL_StatusTypeDef WAND_RequestDmaFrame(const uint8_t *frame,
                                              uint32_t sample_index)
{
  HAL_StatusTypeDef status;
  uint32_t primask;

  if (frame == NULL)
    return HAL_ERROR;

  primask = __get_PRIMASK();
  __disable_irq();
  if ((wand_dma_active != 0U) ||
      (wand_dma_completion_pending != 0U) ||
      (wand_dma_error_pending != 0U))
  {
    WAND_StoreLatestPending(frame, sample_index);
    wand_dma_busy_count++;
    if (primask == 0U)
      __enable_irq();
    wand_spi_status = HAL_BUSY;
    return HAL_BUSY;
  }

  /* Reserve the sole DMA slot before interrupts are restored. */
  wand_dma_active_sample_index = sample_index;
  wand_dma_active = 1U;
  if (primask == 0U)
    __enable_irq();

  wand_frame_attempt_count++;
  status = SK9822_TransmitFrameDma(&hspi1, frame, SK9822_FRAME_BYTES);
  wand_spi_status = (int32_t)status;
  wand_spi_hal_error = HAL_SPI_GetError(&hspi1);
  if (status == HAL_OK)
    return status;

  primask = __get_PRIMASK();
  __disable_irq();
  wand_dma_active = 0U;
  if (status == HAL_BUSY)
  {
    /* HAL can still report busy during a narrow state transition.  Retain the
     * frame for a main-loop retry and do not classify this as a failure. */
    WAND_StoreLatestPending(frame, sample_index);
    wand_dma_busy_count++;
  }
  if (primask == 0U)
    __enable_irq();

  if (status != HAL_BUSY)
    wand_spi_error_count++;

  return status;
}

static void WAND_ServiceLedDma(void)
{
  const uint8_t *pending_frame = NULL;
  uint32_t pending_sample_index = 0U;
  uint32_t completed_sample_index = 0U;
  uint32_t callback_error = HAL_SPI_ERROR_NONE;
  uint32_t primask;
  uint8_t completed;
  uint8_t failed;

  primask = __get_PRIMASK();
  __disable_irq();
  completed = wand_dma_completion_pending;
  failed = wand_dma_error_pending;
  if (completed != 0U)
  {
    completed_sample_index = wand_dma_completed_sample_index;
    wand_dma_completion_pending = 0U;
  }
  if (failed != 0U)
  {
    callback_error = wand_dma_callback_error;
    wand_dma_error_pending = 0U;
  }
  if (primask == 0U)
    __enable_irq();

  if (completed != 0U)
  {
    wand_dma_completion_count++;
    wand_frame_update_count++;
    wand_dma_last_completed_sample_index = completed_sample_index;
    wand_spi_status = HAL_OK;
    wand_spi_hal_error = HAL_SPI_ERROR_NONE;
  }
  if (failed != 0U)
  {
    wand_dma_error_callback_count++;
    wand_spi_error_count++;
    wand_spi_status = HAL_ERROR;
    wand_spi_hal_error = callback_error;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if ((wand_dma_active == 0U) &&
      (wand_dma_completion_pending == 0U) &&
      (wand_dma_error_pending == 0U) &&
      (wand_dma_pending != 0U))
  {
    pending_frame = wand_dma_pending_frame;
    pending_sample_index = wand_dma_pending_sample_index;
    wand_dma_pending = 0U;
  }
  if (primask == 0U)
    __enable_irq();

  if (pending_frame != NULL)
    (void)WAND_RequestDmaFrame(pending_frame, pending_sample_index);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    /* ISR work is intentionally limited to publishing completion state. */
    wand_dma_completed_sample_index = wand_dma_active_sample_index;
    wand_dma_active = 0U;
    wand_dma_completion_pending = 1U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    /* Defer counters, diagnostics, retries, and all printing to main context. */
    wand_dma_callback_error = hspi->ErrorCode;
    wand_dma_active = 0U;
    wand_dma_error_pending = 1U;
  }
}

#if defined(DEBUG)
static const char *WAND_DebugStoragePhaseName(wand_storage_phase_t phase)
{
  switch (phase)
  {
    case WAND_STORAGE_PHASE_ARGUMENT: return "argument";
    case WAND_STORAGE_PHASE_MOUNT: return "mount";
    case WAND_STORAGE_PHASE_OPEN: return "open";
    case WAND_STORAGE_PHASE_HEADER_READ: return "header read";
    case WAND_STORAGE_PHASE_HEADER_VALIDATE: return "header validation";
    case WAND_STORAGE_PHASE_FILE_SIZE: return "file size";
    case WAND_STORAGE_PHASE_CAPACITY: return "RAM capacity";
    case WAND_STORAGE_PHASE_PAYLOAD_READ: return "payload read";
    case WAND_STORAGE_PHASE_PAYLOAD_VALIDATE: return "payload validation";
    case WAND_STORAGE_PHASE_CLOSE: return "close";
    case WAND_STORAGE_PHASE_COMPLETE: return "complete";
    default: return "not started";
  }
}

static const char *WAND_DebugFatFsResultName(int32_t result)
{
  switch ((FRESULT)result)
  {
    case FR_OK: return "FR_OK";
    case FR_DISK_ERR: return "FR_DISK_ERR";
    case FR_INT_ERR: return "FR_INT_ERR";
    case FR_NOT_READY: return "FR_NOT_READY";
    case FR_NO_FILE: return "FR_NO_FILE";
    case FR_NO_PATH: return "FR_NO_PATH";
    case FR_INVALID_NAME: return "FR_INVALID_NAME";
    case FR_DENIED: return "FR_DENIED";
    case FR_EXIST: return "FR_EXIST";
    case FR_INVALID_OBJECT: return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED: return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE: return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED: return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM: return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED: return "FR_MKFS_ABORTED";
    case FR_TIMEOUT: return "FR_TIMEOUT";
    case FR_LOCKED: return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE: return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER: return "FR_INVALID_PARAMETER";
    default: return "unknown FRESULT";
  }
}

static const char *WAND_DebugWandResultName(wand_result_t result)
{
  switch (result)
  {
    case WAND_OK: return "WAND_OK";
    case WAND_ERROR_ARGUMENT: return "WAND_ERROR_ARGUMENT";
    case WAND_ERROR_SHORT_HEADER: return "WAND_ERROR_SHORT_HEADER";
    case WAND_ERROR_MAGIC: return "WAND_ERROR_MAGIC";
    case WAND_ERROR_VERSION: return "WAND_ERROR_VERSION";
    case WAND_ERROR_FORMAT: return "WAND_ERROR_FORMAT";
    case WAND_ERROR_HEADER_CRC: return "WAND_ERROR_HEADER_CRC";
    case WAND_ERROR_PAYLOAD_LENGTH: return "WAND_ERROR_PAYLOAD_LENGTH";
    case WAND_ERROR_PAYLOAD_CRC: return "WAND_ERROR_PAYLOAD_CRC";
    case WAND_ERROR_SK9822_FRAME: return "WAND_ERROR_SK9822_FRAME";
    default: return "unknown WAND result";
  }
}

static const char *WAND_DebugHalStatusName(int32_t status)
{
  switch ((HAL_StatusTypeDef)status)
  {
    case HAL_OK: return "HAL_OK";
    case HAL_ERROR: return "HAL_ERROR";
    case HAL_BUSY: return "HAL_BUSY";
    case HAL_TIMEOUT: return "HAL_TIMEOUT";
    default: return "unknown HAL status";
  }
}

static void WAND_DebugPrintStorageStatus(void)
{
  wand_storage_diagnostics_t diagnostics;
  uint32_t sample;
  uint32_t lit_frames = 0U;
  uint32_t lit_led_commands = 0U;

  WAND_StorageGetDiagnostics(&diagnostics);
  if (wand_storage_status != WAND_STORAGE_OK)
  {
    (void)printf(
        "[ERROR][SD] %s load failed: status=%ld, phase=%s, "
        "FatFs=%ld (%s), WAND=%ld (%s).\r\n",
        WAND_STORAGE_FILENAME,
        (long)wand_storage_status,
        WAND_DebugStoragePhaseName(diagnostics.phase),
        (long)diagnostics.fatfs_result,
        WAND_DebugFatFsResultName(diagnostics.fatfs_result),
        (long)diagnostics.wand_result,
        WAND_DebugWandResultName(diagnostics.wand_result));
    (void)printf(
        "[ERROR][SD] bytes_read=%lu, file_bytes=%lu, expected=%lu.\r\n",
        (unsigned long)diagnostics.bytes_read,
        (unsigned long)diagnostics.file_bytes,
        (unsigned long)diagnostics.expected_file_bytes);

    if (wand_storage_status == WAND_STORAGE_ERROR_MOUNT)
    {
      (void)printf(
          "[HINT][SD] Insert a FAT32 card and check PC8-PC12/PD2 plus a "
          "common ground. FR_NO_FILESYSTEM means reformat as FAT32.\r\n");
    }
    else if (wand_storage_status == WAND_STORAGE_ERROR_OPEN)
    {
      (void)printf(
          "[HINT][SD] Copy the binary export to the card root with the exact "
          "8.3 name %s (not WAND.json or WAND.POV.POV).\r\n",
          WAND_STORAGE_FILENAME);
    }
    else
    {
      (void)printf(
          "[HINT][SD] Re-export %s from the current converter and copy it "
          "again; the file was unreadable or failed WAND1 validation.\r\n",
          WAND_STORAGE_FILENAME);
    }
    return;
  }

  for (sample = 0U; sample < wand_header.sample_count; sample++)
  {
    const uint8_t *frame =
        &wand_payload[sample * wand_header.frame_bytes];
    const uint32_t lit_leds = WAND_CountLitLeds(frame);

    lit_led_commands += lit_leds;
    if (lit_leds != 0U)
      lit_frames++;
  }

  (void)printf(
      "[OK][SD] %s loaded and CRC-checked: %lu bytes, %lu frames, "
      "%u LEDs, step=%lu microdegrees.\r\n",
      WAND_STORAGE_FILENAME,
      (unsigned long)wand_header.file_bytes,
      (unsigned long)wand_header.sample_count,
      (unsigned int)wand_header.led_count,
      (unsigned long)wand_header.angle_step_udeg);
  (void)printf(
      "[OK][SD] Image content: %lu/%lu frames and %lu LED commands are "
      "non-black.\r\n",
      (unsigned long)lit_frames,
      (unsigned long)wand_header.sample_count,
      (unsigned long)lit_led_commands);
  if (lit_frames == 0U)
  {
    (void)printf(
        "[WARN][SD] Every stored LED command is black or has brightness "
        "zero; regenerate the image with visible pixels and brightness.\r\n");
  }
}

static void WAND_DebugRunLedSelfTest(void)
{
  uint32_t led;
  HAL_StatusTypeDef status;

  if (wand_debug_led_self_test_on_boot == 0U)
  {
    (void)printf("[SKIP][LED] RGB boot self-test disabled.\r\n");
    return;
  }

  SK9822_MakeBlankFrame(wand_blank_frame, sizeof(wand_blank_frame));
  for (led = 0U; led < SK9822_LED_COUNT; led++)
  {
    const uint32_t offset = SK9822_START_FRAME_BYTES +
                            led * SK9822_BYTES_PER_LED;
    wand_blank_frame[offset] = 0xE1U; /* 1/31 global brightness. */
    if (led < 12U)
      wand_blank_frame[offset + 2U] = 0xFFU; /* Red. */
    else if (led < 24U)
      wand_blank_frame[offset + 1U] = 0xFFU; /* Green. */
    else
      wand_blank_frame[offset + 3U] = 0xFFU; /* Blue. */
  }

  (void)printf(
      "[TEST][LED] Showing low-brightness red/green/blue segments for "
      "1 second.\r\n");
  status = WAND_TransmitTracked(wand_blank_frame);
  (void)printf(
      "[TEST][LED] SPI result=%s, HAL error=0x%08lX.\r\n",
      WAND_DebugHalStatusName((int32_t)status),
      (unsigned long)wand_spi_hal_error);
  if (status == HAL_OK)
  {
    (void)printf(
        "[TEST][LED] If no colors are visible now, inspect 5 V power, "
        "common ground, strip input direction, PA5 clock, PA7 data, and "
        "3.3 V logic compatibility.\r\n");
  }
  HAL_Delay(1000U);

  SK9822_MakeBlankFrame(wand_blank_frame, sizeof(wand_blank_frame));
  status = WAND_TransmitTracked(wand_blank_frame);
  (void)printf(
      "[%s][LED] Boot self-test ended; blank-frame result=%s.\r\n",
      (status == HAL_OK) ? "OK" : "ERROR",
      WAND_DebugHalStatusName((int32_t)status));
}

static void WAND_DebugPoll(void)
{
  static uint32_t last_tick_ms = 0U;
  static uint32_t last_orientation_count = 0U;
  static uint32_t last_blank_sample = UINT32_MAX;
  const uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms;
  uint32_t orientation_delta;
  uint32_t orientation_rate_hz;
  int32_t roll_mdeg;
  int32_t angle_mdeg;

  if (wand_debug_periodic_output_enabled == 0U)
    return;

  if (last_tick_ms == 0U)
  {
    last_tick_ms = now_ms;
    last_orientation_count = imu_orientation_update_count;
    return;
  }

  elapsed_ms = now_ms - last_tick_ms;
  if (elapsed_ms < WAND_DEBUG_STATUS_PERIOD_MS)
    return;

  orientation_delta =
      imu_orientation_update_count - last_orientation_count;
  orientation_rate_hz =
      (orientation_delta * 1000U + elapsed_ms / 2U) / elapsed_ms;
  roll_mdeg = (int32_t)lroundf(imu_orientation_roll_deg * 1000.0f);
  angle_mdeg = (int32_t)lroundf(wand_angle_deg * 1000.0f);

  (void)printf(
      "[RUN] loaded=%u imu=%ld orient=%lu Hz startup=%u roll=%ld mdeg "
      "wand=%ld mdeg frame=%lu/%lu lit=%lu tx_ok=%lu attempts=%lu "
      "dma=%u pending=%u coalesced=%lu busy=%lu spi=%s errors=%lu "
      "hal=0x%08lX burst=%lu split=%lu hg=%lu temp=%lu.\r\n",
      (unsigned int)wand_loaded,
      (long)imu_status,
      (unsigned long)orientation_rate_hz,
      (unsigned int)imu_orientation_startup,
      (long)roll_mdeg,
      (long)angle_mdeg,
      (unsigned long)wand_sample_index,
      (unsigned long)((wand_loaded != 0U) ? wand_header.sample_count : 0U),
      (unsigned long)wand_selected_lit_led_count,
      (unsigned long)wand_frame_update_count,
      (unsigned long)wand_frame_attempt_count,
      (unsigned int)wand_dma_active,
      (unsigned int)wand_dma_pending,
      (unsigned long)wand_dma_coalesced_frame_count,
      (unsigned long)wand_dma_busy_count,
      WAND_DebugHalStatusName(wand_spi_status),
      (unsigned long)wand_spi_error_count,
      (unsigned long)wand_spi_hal_error,
      (unsigned long)imu_gyro_accel_burst_count,
      (unsigned long)imu_gyro_accel_split_read_count,
      (unsigned long)imu_high_g_read_count,
      (unsigned long)imu_temperature_read_count);

  if (orientation_delta == 0U)
  {
    (void)printf(
        "[WARN][IMU] No synchronized accelerometer/gyro orientation update "
        "arrived during this interval.\r\n");
  }
  if ((imu_orientation_startup != 0U) && (now_ms >= 5000U))
  {
    (void)printf(
        "[WAIT][IMU] Fusion is still in startup; playback intentionally "
        "keeps the LEDs blank until startup becomes 0. Hold the wand still.\r\n");
  }
  if ((wand_loaded != 0U) && (imu_orientation_startup == 0U) &&
      (wand_selected_lit_led_count == 0U) &&
      (wand_sample_index != last_blank_sample))
  {
    (void)printf(
        "[INFO][LED] Selected frame %lu contains no illuminated LEDs; "
        "move the wand or inspect that angle in WAND.json.\r\n",
        (unsigned long)wand_sample_index);
    last_blank_sample = wand_sample_index;
  }
  if ((wand_spi_status != HAL_OK) && (wand_spi_status != HAL_BUSY))
  {
    (void)printf(
        "[ERROR][LED] SPI transmission is failing. HAL status=%s, "
        "error=0x%08lX.\r\n",
        WAND_DebugHalStatusName(wand_spi_status),
        (unsigned long)wand_spi_hal_error);
  }

  last_tick_ms = now_ms;
  last_orientation_count = imu_orientation_update_count;
}
#endif

static void WAND_PlayCurrentAngle(void)
{
  uint32_t sample_index;
  int32_t roll_mdeg;
  int32_t angle_mdeg;
  const uint8_t *frame;

  if (imu_orientation_update_count == wand_last_orientation_update_count)
    return;

  wand_last_orientation_update_count = imu_orientation_update_count;
  roll_mdeg = (int32_t)lroundf(imu_orientation_roll_deg * 1000.0f);
  angle_mdeg = WAND_RollToAngleMdeg(roll_mdeg);
  wand_angle_deg = (float)angle_mdeg / 1000.0f;

  if ((wand_loaded == 0U) || (imu_orientation_startup != 0U))
    return;

  sample_index = WAND_AngleToSample(&wand_header, angle_mdeg);
  wand_sample_index = sample_index;

  /* The sensor runs faster than most exported angle grids.  Avoid spending
   * SPI bandwidth when consecutive orientation estimates select one frame. */
  if ((wand_requested_sample_valid != 0U) &&
      (sample_index == wand_requested_sample_index))
  {
    return;
  }

  frame = &wand_payload[sample_index * wand_header.frame_bytes];
#if defined(DEBUG)
  wand_selected_lit_led_count = WAND_CountLitLeds(frame);
#endif
  /* The payload is immutable after startup, so both active and pending DMA
   * pointers remain valid.  Updating this marker before queuing also prevents
   * duplicate requests for the same angle while a transfer is in flight. */
  wand_requested_sample_index = sample_index;
  wand_requested_sample_valid = 1U;
  wand_spi_status = (int32_t)WAND_RequestDmaFrame(frame, sample_index);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if defined(DEBUG)
  /* Use the same librdimon semihosting path as imu_calibration_session.c. */
  initialise_monitor_handles();
  (void)setvbuf(stdout, NULL, _IONBF, 0);
  wand_debug_console_ready = 1U;
  wand_debug_boot_stage = "peripheral initialization";
  (void)printf(
      "\r\n[BOOT] STM32 IMU POV wand diagnostics started (Debug build).\r\n");
  (void)printf(
      "[BOOT] Console uses ST-LINK GDB Server semihosting; keep the debugger "
      "connected and resumed.\r\n");
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* Keep the strip dark until both the command file and orientation filter
   * are ready.  A missing or invalid SD file remains a visible diagnostic
   * status and does not prevent IMU testing. */
  SK9822_MakeBlankFrame(wand_blank_frame, sizeof(wand_blank_frame));
  wand_spi_status = (int32_t)WAND_TransmitTracked(wand_blank_frame);
#if defined(DEBUG)
  (void)printf(
      "[%s][LED] Initial blank-frame SPI result=%s, HAL error=0x%08lX.\r\n",
      (wand_spi_status == HAL_OK) ? "OK" : "ERROR",
      WAND_DebugHalStatusName(wand_spi_status),
      (unsigned long)wand_spi_hal_error);
  WAND_DebugRunLedSelfTest();
#endif

  wand_storage_status = (int32_t)WAND_StorageLoad(
      wand_payload, sizeof(wand_payload), &wand_header);
  wand_loaded = (wand_storage_status == WAND_STORAGE_OK) ? 1U : 0U;
#if defined(DEBUG)
  WAND_DebugPrintStorageStatus();
#endif

  imu_status = IMU_Init();
#if defined(DEBUG)
  (void)printf(
      "[%s][IMU] initialization status=%ld, WHO_AM_I=0x%02X "
      "(expected 0x%02X), HAL I2C address=0x%02X.\r\n",
      (imu_status == 0) ? "OK" : "ERROR",
      (long)imu_status,
      (unsigned int)imu_who_am_i,
      (unsigned int)LSM6DSV320X_ID,
      (unsigned int)imu_i2c_address);
#endif

  /* Stop here on initialization failure so imu_status stays visible. */
  if (imu_status != 0)
  {
#if defined(DEBUG)
    (void)printf(
        "[FATAL][IMU] Initialization failed at step %ld. Playback cannot "
        "start; inspect the status, WHO_AM_I, wiring, CS high, and SA0.\r\n",
        (long)imu_status);
#endif
    while (1)
    {
      HAL_Delay(100U);
    }
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    imu_status = -30;
#if defined(DEBUG)
    (void)printf("[FATAL][TIM2] Timer start failed; imu_status=-30.\r\n");
#endif
    while (1)
    {
      HAL_Delay(100U);
    }
  }

  IMU_OrientationInitialise(IMU_OUTPUT_DATA_RATE_HZ);
#if defined(DEBUG)
  (void)printf(
      "[OK][IMU] Orientation filter initialized at %lu Hz. LEDs remain "
      "blank while fusion startup=1 (normally about 3 seconds).\r\n",
      (unsigned long)((uint32_t)IMU_OUTPUT_DATA_RATE_HZ));
#endif

  if (imu_run_calibration_on_boot != 0U)
  {
#if defined(DEBUG)
    (void)printf("[BOOT] Entering the requested IMU calibration session.\r\n");
#endif
    imu_status = IMU_RunCalibrationSession(IMU_ReadSensors, HAL_Delay);

    /* Keep the generated constants and status available in Live Expressions. */
    while (1)
    {
      HAL_Delay(100U);
    }
  }
#if defined(DEBUG)
  (void)printf(
      "[BOOT] Entering playback loop: loaded=%u. Runtime status follows once "
      "per second.\r\n",
      (unsigned int)wand_loaded);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    imu_status = IMU_ReadAll();
    WAND_PlayCurrentAngle();
    WAND_ServiceLedDma();
#if defined(DEBUG)
    WAND_DebugPoll();
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00401242;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB6);
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB7);
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 12;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
#if defined(DEBUG)
  if (wand_debug_console_ready != 0U)
  {
    (void)printf(
        "[FATAL][HAL] Error_Handler reached during %s at tick %lu ms. "
        "I2C error=0x%08lX, SPI error=0x%08lX, SD error=0x%08lX.\r\n",
        wand_debug_boot_stage,
        (unsigned long)HAL_GetTick(),
        (unsigned long)HAL_I2C_GetError(&hi2c1),
        (unsigned long)HAL_SPI_GetError(&hspi1),
        (unsigned long)hsd1.ErrorCode);
    (void)fflush(stdout);
  }
#endif
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
#if defined(DEBUG)
  if (wand_debug_console_ready != 0U)
  {
    (void)printf("[FATAL][ASSERT] %s:%lu.\r\n",
                 (const char *)file,
                 (unsigned long)line);
    (void)fflush(stdout);
  }
#else
  (void)file;
  (void)line;
#endif
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

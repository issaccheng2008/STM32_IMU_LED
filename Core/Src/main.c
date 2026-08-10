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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_OUTPUT_DATA_RATE_HZ 1920.0f
#define IMU_ORIENTATION_TIMER_SECONDS_PER_TICK 1.0e-6f

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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static stmdev_ctx_t imu_ctx;
static uint8_t imu_orientation_pending_mask = 0U;
static uint8_t imu_orientation_timestamp_valid = 0U;
static uint32_t imu_orientation_last_timestamp_us = 0U;
static uint32_t wand_last_orientation_update_count = 0U;
static uint32_t wand_last_sample_index = 0U;
static uint8_t wand_last_sample_valid = 0U;

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
/** Number of new SK9822 frames sent by the playback loop. */
volatile uint32_t wand_frame_update_count = 0U;
/** Most recent HAL SPI result returned while updating the strip. */
volatile int32_t wand_spi_status = HAL_OK;

/** Last IMU initialization or read result: 0 means success. */
volatile int32_t imu_status = -1;

/** WHO_AM_I register value; an LSM6DSV320X returns 0x73. */
volatile uint8_t imu_who_am_i = 0U;

/** Active 8-bit I2C address; selected automatically from the SDO/SA0 pin. */
volatile uint8_t imu_i2c_address = LSM6DSV320X_I2C_ADD_L;

/** Number of polling cycles in which at least one new sensor value was read. */
volatile uint32_t imu_sample_count = 0U;

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
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static int32_t IMU_Init(void);
static int32_t IMU_ReadAll(void);
static int32_t IMU_ReadSensors(imu_calibration_measurement_t *measurement);
static void WAND_PlayCurrentAngle(void);
#if defined(DEBUG)
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

    imu_accel_x_raw = raw_axes[0];
    imu_accel_y_raw = raw_axes[1];
    imu_accel_z_raw = raw_axes[2];
    imu_accel_x_mg = lsm6dsv320x_from_fs4_to_mg(raw_axes[0]);
    imu_accel_y_mg = lsm6dsv320x_from_fs4_to_mg(raw_axes[1]);
    imu_accel_z_mg = lsm6dsv320x_from_fs4_to_mg(raw_axes[2]);
    {
      const imu_vector3_t raw_mg = {
          imu_accel_x_mg, imu_accel_y_mg, imu_accel_z_mg};
      imu_vector3_t calibrated_mg;

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
    updated = 1U;
  }

  if (data_ready.drdy_gy != 0U)
  {
    if (lsm6dsv320x_angular_rate_raw_get(&imu_ctx, raw_axes) != 0)
      return -22;

    imu_gyro_x_raw = raw_axes[0];
    imu_gyro_y_raw = raw_axes[1];
    imu_gyro_z_raw = raw_axes[2];
    imu_gyro_x_dps = lsm6dsv320x_from_fs4000_to_mdps(raw_axes[0]) / 1000.0f;
    imu_gyro_y_dps = lsm6dsv320x_from_fs4000_to_mdps(raw_axes[1]) / 1000.0f;
    imu_gyro_z_dps = lsm6dsv320x_from_fs4000_to_mdps(raw_axes[2]) / 1000.0f;
    {
      const imu_vector3_t raw_dps = {
          imu_gyro_x_dps, imu_gyro_y_dps, imu_gyro_z_dps};
      imu_vector3_t calibrated_dps;

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
    updated = 1U;
  }

  if (data_ready.drdy_hgxl != 0U)
  {
    if (lsm6dsv320x_hg_acceleration_raw_get(&imu_ctx, raw_axes) != 0)
      return -23;

    imu_high_g_x_raw = raw_axes[0];
    imu_high_g_y_raw = raw_axes[1];
    imu_high_g_z_raw = raw_axes[2];
    imu_high_g_x_mg = lsm6dsv320x_from_fs32_to_mg(raw_axes[0]);
    imu_high_g_y_mg = lsm6dsv320x_from_fs32_to_mg(raw_axes[1]);
    imu_high_g_z_mg = lsm6dsv320x_from_fs32_to_mg(raw_axes[2]);
    {
      const imu_vector3_t raw_mg = {
          imu_high_g_x_mg, imu_high_g_y_mg, imu_high_g_z_mg};
      imu_vector3_t calibrated_mg;

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
  imu_calibration_measurement_t measurement;
  int32_t status;

  status = IMU_ReadSensors(&measurement);
  if (status != 0)
    return status;

  imu_orientation_pending_mask |=
      measurement.ready_mask &
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

  return 0;
}

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
  if ((wand_last_sample_valid != 0U) &&
      (sample_index == wand_last_sample_index))
  {
    return;
  }

  frame = &wand_payload[sample_index * wand_header.frame_bytes];
  wand_spi_status = (int32_t)SK9822_TransmitFrame(
      &hspi1, frame, wand_header.frame_bytes);
  if (wand_spi_status == HAL_OK)
  {
    wand_last_sample_index = sample_index;
    wand_last_sample_valid = 1U;
    wand_frame_update_count++;
  }
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Keep the strip dark until both the command file and orientation filter
   * are ready.  A missing or invalid SD file remains a visible diagnostic
   * status and does not prevent IMU testing. */
  SK9822_MakeBlankFrame(wand_blank_frame, sizeof(wand_blank_frame));
  wand_spi_status = (int32_t)SK9822_TransmitFrame(
      &hspi1, wand_blank_frame, sizeof(wand_blank_frame));

  wand_storage_status = (int32_t)WAND_StorageLoad(
      wand_payload, sizeof(wand_payload), &wand_header);
  wand_loaded = (wand_storage_status == WAND_STORAGE_OK) ? 1U : 0U;

  imu_status = IMU_Init();

  /* Stop here on initialization failure so imu_status stays visible. */
  if (imu_status != 0)
  {
    while (1)
    {
      HAL_Delay(100U);
    }
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    imu_status = -30;
    while (1)
    {
      HAL_Delay(100U);
    }
  }

  IMU_OrientationInitialise(IMU_OUTPUT_DATA_RATE_HZ);

  if (imu_run_calibration_on_boot != 0U)
  {
#if defined(DEBUG)
    /* Connect standard printf/fopen calls to the debugger via librdimon. */
    initialise_monitor_handles();
#endif
    imu_status = IMU_RunCalibrationSession(IMU_ReadSensors, HAL_Delay);

    /* Keep the generated constants and status available in Live Expressions. */
    while (1)
    {
      HAL_Delay(100U);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    imu_status = IMU_ReadAll();
    WAND_PlayCurrentAngle();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  /* Keep the CPU on the 64 MHz HSI while using a 75 MHz PLL1 Q output for
   * SPI1 and SDMMC1.  This is the clock setup proven by LED_strip_test. */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /* 1 MHz Fast-mode Plus with a 64 MHz I2C kernel clock. */
  hi2c1.Init.Timing = 0x00610E1A;
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
  /* Card initialization is deliberately deferred to FatFs disk_initialize().
   * That lets a missing card produce WAND_STORAGE_ERROR_MOUNT instead of
   * trapping the entire IMU application in Error_Handler(). */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  /* PLL1 Q = 75 MHz; /8 gives a 9.375 MHz SK9822 clock. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness =
      SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 63;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

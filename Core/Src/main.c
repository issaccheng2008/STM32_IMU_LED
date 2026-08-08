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
#include "lsm6dsv320x_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static stmdev_ctx_t imu_ctx;

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
/** X-axis angular rate in degrees per second, configured for +/-500 dps. */
volatile float imu_gyro_x_dps = 0.0f;
/** Y-axis angular rate in degrees per second, configured for +/-500 dps. */
volatile float imu_gyro_y_dps = 0.0f;
/** Z-axis angular rate in degrees per second, configured for +/-500 dps. */
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
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static int32_t IMU_Init(void);
static int32_t IMU_ReadAll(void);
static int32_t IMU_ReadSensors(imu_calibration_measurement_t *measurement);
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
                             100U);

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
                            100U);

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

  if (lsm6dsv320x_gy_full_scale_set(&imu_ctx, LSM6DSV320X_500dps) != 0)
    return -6;

  if (lsm6dsv320x_hg_xl_full_scale_set(&imu_ctx, LSM6DSV320X_32g) != 0)
    return -7;

  if (lsm6dsv320x_xl_setup(&imu_ctx,
                           LSM6DSV320X_ODR_AT_120Hz,
                           LSM6DSV320X_XL_HIGH_PERFORMANCE_MD) != 0)
    return -8;

  if (lsm6dsv320x_gy_setup(&imu_ctx,
                           LSM6DSV320X_ODR_AT_120Hz,
                           LSM6DSV320X_GY_HIGH_PERFORMANCE_MD) != 0)
    return -9;

  if (lsm6dsv320x_hg_xl_data_rate_set(&imu_ctx,
                                      LSM6DSV320X_HG_XL_ODR_AT_480Hz,
                                      PROPERTY_ENABLE) != 0)
    return -10;

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
    imu_gyro_x_dps = lsm6dsv320x_from_fs500_to_mdps(raw_axes[0]) / 1000.0f;
    imu_gyro_y_dps = lsm6dsv320x_from_fs500_to_mdps(raw_axes[1]) / 1000.0f;
    imu_gyro_z_dps = lsm6dsv320x_from_fs500_to_mdps(raw_axes[2]) / 1000.0f;
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
  return IMU_ReadSensors(NULL);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  imu_status = IMU_Init();

  /* Stop here on initialization failure so imu_status stays visible. */
  if (imu_status != 0)
  {
    while (1)
    {
      HAL_Delay(100U);
    }
  }

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
    HAL_Delay(1U);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
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
  hi2c1.Init.Timing = 0x00707CBB;
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

  /* USER CODE END I2C1_Init 2 */

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

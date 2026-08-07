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
#include "lsm6dsv320x_reg.h"
#include <stdint.h>
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

/* Easy to observe in CubeIDE Live Expressions */
volatile int32_t imu_status = -100;
volatile uint8_t imu_whoami = 0;

volatile float acc_x_ms2 = 0.0f;
volatile float acc_y_ms2 = 0.0f;
volatile float acc_z_ms2 = 0.0f;

volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

volatile uint32_t imu_sample_count = 0;

/* Later used by position estimator */
volatile float velocity_x_ms = 0.0f;
volatile float velocity_y_ms = 0.0f;
volatile float velocity_z_ms = 0.0f;

volatile float position_x_m = 0.0f;
volatile float position_y_m = 0.0f;
volatile float position_z_m = 0.0f;

static float accel_zero_x = 0.0f;
static float accel_zero_y = 0.0f;
static float accel_zero_z = 0.0f;

static uint32_t previous_time_us = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Button_State;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int32_t platform_write(void *handle,
                              uint8_t reg,
                              const uint8_t *bufp,
                              uint16_t len)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Write(
            (I2C_HandleTypeDef *)handle,
            LSM6DSV320X_I2C_ADD_L,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            (uint8_t *)bufp,
            len,
            100);

    return (ret == HAL_OK) ? 0 : -1;
}


static int32_t platform_read(void *handle,
                             uint8_t reg,
                             uint8_t *bufp,
                             uint16_t len)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(
            (I2C_HandleTypeDef *)handle,
            LSM6DSV320X_I2C_ADD_L,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            bufp,
            len,
            100);

    return (ret == HAL_OK) ? 0 : -1;
}


static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

static int32_t IMU_Init(void)
{
    int32_t ret;

    imu_ctx.write_reg = platform_write;
    imu_ctx.read_reg  = platform_read;
    imu_ctx.mdelay    = platform_delay;
    imu_ctx.handle    = &hi2c1;
    imu_ctx.priv_data = NULL;

    HAL_Delay(20);

    /*
     * First test communication BEFORE changing any registers.
     */
    ret = lsm6dsv320x_device_id_get(&imu_ctx,
                                    (uint8_t *)&imu_whoami);

    if (ret != 0)
        return -1;      /* I2C communication error */

    if (imu_whoami != LSM6DSV320X_ID)
        return -2;      /* Wrong device / wrong address */

    /*
     * Reset configuration.
     */
    ret = lsm6dsv320x_sw_por(&imu_ctx);

    if (ret != 0)
        return -3;

    HAL_Delay(10);

    /*
     * Protect 16-bit data while reading.
     */
    ret = lsm6dsv320x_block_data_update_set(
            &imu_ctx,
            PROPERTY_ENABLE);

    if (ret != 0)
        return -4;

    /*
     * Enable register auto-increment.
     */
    ret = lsm6dsv320x_auto_increment_set(
            &imu_ctx,
            PROPERTY_ENABLE);

    if (ret != 0)
        return -5;

    /*
     * Low-g accelerometer:
     * +/-4 g
     */
    ret = lsm6dsv320x_xl_full_scale_set(
            &imu_ctx,
            LSM6DSV320X_4g);

    if (ret != 0)
        return -6;

    /*
     * Gyroscope:
     * +/-500 degrees/sec
     */
    ret = lsm6dsv320x_gy_full_scale_set(
            &imu_ctx,
            LSM6DSV320X_500dps);

    if (ret != 0)
        return -7;

    /*
     * Accelerometer:
     * 120 Hz
     * High-performance mode
     */
    ret = lsm6dsv320x_xl_setup(
            &imu_ctx,
            LSM6DSV320X_ODR_AT_120Hz,
            LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);

    if (ret != 0)
        return -8;

    /*
     * Gyroscope:
     * 120 Hz
     * High-performance mode
     */
    ret = lsm6dsv320x_gy_setup(
            &imu_ctx,
            LSM6DSV320X_ODR_AT_120Hz,
            LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);

    if (ret != 0)
        return -9;

    HAL_Delay(50);

    return 0;
}

static int32_t IMU_Read(void)
{
    lsm6dsv320x_data_ready_t drdy;
    int16_t raw[3];

    int32_t ret;

    ret = lsm6dsv320x_flag_data_ready_get(
            &imu_ctx,
            &drdy);

    if (ret != 0)
        return -1;

    /*
     * Accelerometer
     */
    if (drdy.drdy_xl)
    {
        ret = lsm6dsv320x_acceleration_raw_get(
                &imu_ctx,
                raw);

        if (ret != 0)
            return -2;

        /*
         * Driver converts raw values -> mg.
         *
         * 1 mg = 0.00980665 m/s^2
         */
        acc_x_ms2 =
            lsm6dsv320x_from_fs4_to_mg(raw[0])
            * 0.00980665f;

        acc_y_ms2 =
            lsm6dsv320x_from_fs4_to_mg(raw[1])
            * 0.00980665f;

        acc_z_ms2 =
            lsm6dsv320x_from_fs4_to_mg(raw[2])
            * 0.00980665f;

        imu_sample_count++;
    }

    /*
     * Gyroscope
     */
    if (drdy.drdy_gy)
    {
        ret = lsm6dsv320x_angular_rate_raw_get(
                &imu_ctx,
                raw);

        if (ret != 0)
            return -3;

        /*
         * Driver returns mdps.
         * Convert mdps -> deg/s.
         */
        gyro_x_dps =
            lsm6dsv320x_from_fs500_to_mdps(raw[0])
            / 1000.0f;

        gyro_y_dps =
            lsm6dsv320x_from_fs500_to_mdps(raw[1])
            / 1000.0f;

        gyro_z_dps =
            lsm6dsv320x_from_fs500_to_mdps(raw[2])
            / 1000.0f;
    }

    return 0;
}

static void IMU_Calibrate(void)
{
    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    uint32_t samples = 0;

    while (samples < 200)
    {
        uint32_t before = imu_sample_count;

        IMU_Read();

        if (imu_sample_count != before)
        {
            sx += acc_x_ms2;
            sy += acc_y_ms2;
            sz += acc_z_ms2;

            samples++;
        }
    }

    accel_zero_x = sx / 200.0f;
    accel_zero_y = sy / 200.0f;
    accel_zero_z = sz / 200.0f;

    velocity_x_ms = 0.0f;
    velocity_y_ms = 0.0f;
    velocity_z_ms = 0.0f;

    position_x_m = 0.0f;
    position_y_m = 0.0f;
    position_z_m = 0.0f;

    previous_time_us = __HAL_TIM_GET_COUNTER(&htim2);
}

static void Position_Update(void)
{
    uint32_t old_sample;
    uint32_t now_us;
    uint32_t elapsed_us;

    float dt;

    float ax;
    float ay;
    float az;

    old_sample = imu_sample_count;

    if (IMU_Read() != 0)
        return;

    /*
     * No new accelerometer sample.
     */
    if (imu_sample_count == old_sample)
        return;

    now_us = __HAL_TIM_GET_COUNTER(&htim2);

    /*
     * Unsigned subtraction handles TIM2 wrap-around.
     */
    elapsed_us = now_us - previous_time_us;
    previous_time_us = now_us;

    dt = ((float)elapsed_us) * 0.000001f;

    /*
     * Reject unreasonable timing gaps caused by breakpoints etc.
     */
    if ((dt <= 0.0f) || (dt > 0.05f))
        return;

    /*
     * Remove initial stationary acceleration.
     * For a fixed-orientation experiment, this also removes gravity.
     */
    ax = acc_x_ms2 - accel_zero_x;
    ay = acc_y_ms2 - accel_zero_y;
    az = acc_z_ms2 - accel_zero_z;


    /*
     * Simple stationary detector.
     */
    if ((ax > -0.08f) && (ax < 0.08f) &&
        (ay > -0.08f) && (ay < 0.08f) &&
        (az > -0.08f) && (az < 0.08f) &&

        (gyro_x_dps > -1.5f) && (gyro_x_dps < 1.5f) &&
        (gyro_y_dps > -1.5f) && (gyro_y_dps < 1.5f) &&
        (gyro_z_dps > -1.5f) && (gyro_z_dps < 1.5f))
    {
        /*
         * Zero-velocity update.
         */
        velocity_x_ms = 0.0f;
        velocity_y_ms = 0.0f;
        velocity_z_ms = 0.0f;
    }
    else
    {
        /*
         * Integrate acceleration -> position and velocity.
         */
        position_x_m +=
            velocity_x_ms * dt +
            0.5f * ax * dt * dt;

        position_y_m +=
            velocity_y_ms * dt +
            0.5f * ay * dt * dt;

        position_z_m +=
            velocity_z_ms * dt +
            0.5f * az * dt * dt;

        velocity_x_ms += ax * dt;
        velocity_y_ms += ay * dt;
        velocity_z_ms += az * dt;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  imu_status = IMU_Init();

  if (imu_status != 0)
  {
      while (1)
      {
      }
  }

  IMU_Calibrate();

  /*
   * If initialization failed, stay here so we can inspect
   * imu_status and imu_whoami using the debugger.
   */
  if (imu_status != 0)
  {
      while (1)
      {
          HAL_Delay(100);
      }
  }

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Position_Update();
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

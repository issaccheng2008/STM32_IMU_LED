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
#define MS2_PER_MG                              0.00980665f
#define CALIBRATION_SAMPLES                     200U
#define CALIBRATION_TIMEOUT_MS                  6000U
#define STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2   0.12f
#define STATIONARY_GYRO_THRESHOLD_DPS           1.5f
#define STATIONARY_DWELL_SAMPLES                24U
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

volatile float gravity_x_ms2 = 0.0f;
volatile float gravity_y_ms2 = 0.0f;
volatile float gravity_z_ms2 = 0.0f;

volatile float linear_acc_x_ms2 = 0.0f;
volatile float linear_acc_y_ms2 = 0.0f;
volatile float linear_acc_z_ms2 = 0.0f;

volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

volatile float quaternion_w = 1.0f;
volatile float quaternion_x = 0.0f;
volatile float quaternion_y = 0.0f;
volatile float quaternion_z = 0.0f;

volatile uint32_t imu_sample_count = 0;

volatile float velocity_x_ms = 0.0f;
volatile float velocity_y_ms = 0.0f;
volatile float velocity_z_ms = 0.0f;

volatile float position_x_m = 0.0f;
volatile float position_y_m = 0.0f;
volatile float position_z_m = 0.0f;

static lsm6dsv320x_quaternion_t orientation_quaternion =
{
    .quat_w = 1.0f,
    .quat_x = 0.0f,
    .quat_y = 0.0f,
    .quat_z = 0.0f
};

static lsm6dsv320x_quaternion_t initial_orientation_quaternion =
{
    .quat_w = 1.0f,
    .quat_x = 0.0f,
    .quat_y = 0.0f,
    .quat_z = 0.0f
};

static float accel_bias_body_x = 0.0f;
static float accel_bias_body_y = 0.0f;
static float accel_bias_body_z = 0.0f;

static uint16_t stationary_sample_count = 0U;
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


static lsm6dsv320x_quaternion_t Quaternion_Multiply(
        const lsm6dsv320x_quaternion_t *a,
        const lsm6dsv320x_quaternion_t *b)
{
    lsm6dsv320x_quaternion_t result;

    result.quat_w =
        a->quat_w * b->quat_w -
        a->quat_x * b->quat_x -
        a->quat_y * b->quat_y -
        a->quat_z * b->quat_z;

    result.quat_x =
        a->quat_w * b->quat_x +
        a->quat_x * b->quat_w +
        a->quat_y * b->quat_z -
        a->quat_z * b->quat_y;

    result.quat_y =
        a->quat_w * b->quat_y -
        a->quat_x * b->quat_z +
        a->quat_y * b->quat_w +
        a->quat_z * b->quat_x;

    result.quat_z =
        a->quat_w * b->quat_z +
        a->quat_x * b->quat_y -
        a->quat_y * b->quat_x +
        a->quat_z * b->quat_w;

    return result;
}


static int32_t Quaternion_Rotate_Vector(
        const lsm6dsv320x_quaternion_t *q,
        float vx,
        float vy,
        float vz,
        float *rx,
        float *ry,
        float *rz)
{
    float norm_squared;
    float ww;
    float xx;
    float yy;
    float zz;

    norm_squared =
        q->quat_w * q->quat_w +
        q->quat_x * q->quat_x +
        q->quat_y * q->quat_y +
        q->quat_z * q->quat_z;

    if ((norm_squared < 0.25f) || (norm_squared > 2.25f))
        return -1;

    ww = q->quat_w * q->quat_w;
    xx = q->quat_x * q->quat_x;
    yy = q->quat_y * q->quat_y;
    zz = q->quat_z * q->quat_z;

    /*
     * The SFLP game-rotation quaternion rotates sensor-frame vectors into
     * its fixed reference frame. Divide by |q|^2 so half-float rounding
     * cannot scale the rotated acceleration.
     */
    *rx =
        ((ww + xx - yy - zz) * vx +
         2.0f * (q->quat_x * q->quat_y -
                 q->quat_w * q->quat_z) * vy +
         2.0f * (q->quat_x * q->quat_z +
                 q->quat_w * q->quat_y) * vz) / norm_squared;

    *ry =
        (2.0f * (q->quat_x * q->quat_y +
                 q->quat_w * q->quat_z) * vx +
         (ww - xx + yy - zz) * vy +
         2.0f * (q->quat_y * q->quat_z -
                 q->quat_w * q->quat_x) * vz) / norm_squared;

    *rz =
        (2.0f * (q->quat_x * q->quat_z -
                 q->quat_w * q->quat_y) * vx +
         2.0f * (q->quat_y * q->quat_z +
                 q->quat_w * q->quat_x) * vy +
         (ww - xx - yy + zz) * vz) / norm_squared;

    return 0;
}


static int32_t Body_To_Initial_Frame(float body_x,
                                     float body_y,
                                     float body_z,
                                     float *initial_x,
                                     float *initial_y,
                                     float *initial_z)
{
    lsm6dsv320x_quaternion_t initial_conjugate;
    lsm6dsv320x_quaternion_t relative_orientation;

    initial_conjugate.quat_w = initial_orientation_quaternion.quat_w;
    initial_conjugate.quat_x = -initial_orientation_quaternion.quat_x;
    initial_conjugate.quat_y = -initial_orientation_quaternion.quat_y;
    initial_conjugate.quat_z = -initial_orientation_quaternion.quat_z;

    /*
     * q_relative = conjugate(q_at_reset) * q_now.
     * This makes the estimator axes match the IMU axes at reset.
     */
    relative_orientation =
        Quaternion_Multiply(&initial_conjugate, &orientation_quaternion);

    return Quaternion_Rotate_Vector(&relative_orientation,
                                    body_x,
                                    body_y,
                                    body_z,
                                    initial_x,
                                    initial_y,
                                    initial_z);
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

    ret = lsm6dsv320x_device_id_get(&imu_ctx,
                                    (uint8_t *)&imu_whoami);

    if (ret != 0)
        return -1;

    if (imu_whoami != LSM6DSV320X_ID)
        return -2;

    ret = lsm6dsv320x_sw_por(&imu_ctx);

    if (ret != 0)
        return -3;

    HAL_Delay(10);

    ret = lsm6dsv320x_block_data_update_set(
            &imu_ctx,
            PROPERTY_ENABLE);

    if (ret != 0)
        return -4;

    ret = lsm6dsv320x_auto_increment_set(
            &imu_ctx,
            PROPERTY_ENABLE);

    if (ret != 0)
        return -5;

    ret = lsm6dsv320x_xl_full_scale_set(
            &imu_ctx,
            LSM6DSV320X_4g);

    if (ret != 0)
        return -6;

    ret = lsm6dsv320x_gy_full_scale_set(
            &imu_ctx,
            LSM6DSV320X_500dps);

    if (ret != 0)
        return -7;

    ret = lsm6dsv320x_xl_setup(
            &imu_ctx,
            LSM6DSV320X_ODR_AT_120Hz,
            LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);

    if (ret != 0)
        return -8;

    ret = lsm6dsv320x_gy_setup(
            &imu_ctx,
            LSM6DSV320X_ODR_AT_120Hz,
            LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);

    if (ret != 0)
        return -9;

    /*
     * SFLP estimates orientation, gravity, and gyroscope bias from the
     * low-g accelerometer and gyroscope. Its ODR must not exceed theirs.
     */
    ret = lsm6dsv320x_sflp_data_rate_set(
            &imu_ctx,
            LSM6DSV320X_SFLP_120Hz);

    if (ret != 0)
        return -10;

    ret = lsm6dsv320x_sflp_game_rotation_set(
            &imu_ctx,
            PROPERTY_ENABLE);

    if (ret != 0)
        return -11;

    /* Allow filters to start before calibration begins. */
    HAL_Delay(250);

    return 0;
}


static int32_t IMU_Read(void)
{
    lsm6dsv320x_data_ready_t drdy;
    lsm6dsv320x_quaternion_t quaternion;
    int16_t acceleration_raw[3];
    int16_t gravity_raw[3];
    int16_t angular_rate_raw[3];
    float quaternion_norm_squared;
    int32_t ret;

    ret = lsm6dsv320x_flag_data_ready_get(
            &imu_ctx,
            &drdy);

    if (ret != 0)
        return -1;

    if (drdy.drdy_xl)
    {
        ret = lsm6dsv320x_acceleration_raw_get(
                &imu_ctx,
                acceleration_raw);

        if (ret != 0)
            return -2;

        ret = lsm6dsv320x_sflp_gravity_raw_get(
                &imu_ctx,
                gravity_raw);

        if (ret != 0)
            return -3;

        ret = lsm6dsv320x_sflp_quaternion_get(
                &imu_ctx,
                &quaternion);

        if (ret != 0)
            return -4;

        quaternion_norm_squared =
            quaternion.quat_w * quaternion.quat_w +
            quaternion.quat_x * quaternion.quat_x +
            quaternion.quat_y * quaternion.quat_y +
            quaternion.quat_z * quaternion.quat_z;

        /*
         * SFLP output is zero while starting. Do not publish a sample until
         * it contains a plausible unit quaternion.
         */
        if ((quaternion_norm_squared < 0.25f) ||
            (quaternion_norm_squared > 2.25f))
        {
            return -5;
        }

        acc_x_ms2 =
            lsm6dsv320x_from_fs4_to_mg(acceleration_raw[0])
            * MS2_PER_MG;

        acc_y_ms2 =
            lsm6dsv320x_from_fs4_to_mg(acceleration_raw[1])
            * MS2_PER_MG;

        acc_z_ms2 =
            lsm6dsv320x_from_fs4_to_mg(acceleration_raw[2])
            * MS2_PER_MG;

        gravity_x_ms2 =
            lsm6dsv320x_from_sflp_to_mg(gravity_raw[0])
            * MS2_PER_MG;

        gravity_y_ms2 =
            lsm6dsv320x_from_sflp_to_mg(gravity_raw[1])
            * MS2_PER_MG;

        gravity_z_ms2 =
            lsm6dsv320x_from_sflp_to_mg(gravity_raw[2])
            * MS2_PER_MG;

        orientation_quaternion = quaternion;

        quaternion_w = quaternion.quat_w;
        quaternion_x = quaternion.quat_x;
        quaternion_y = quaternion.quat_y;
        quaternion_z = quaternion.quat_z;

        imu_sample_count++;
    }

    if (drdy.drdy_gy)
    {
        ret = lsm6dsv320x_angular_rate_raw_get(
                &imu_ctx,
                angular_rate_raw);

        if (ret != 0)
            return -6;

        gyro_x_dps =
            lsm6dsv320x_from_fs500_to_mdps(angular_rate_raw[0])
            / 1000.0f;

        gyro_y_dps =
            lsm6dsv320x_from_fs500_to_mdps(angular_rate_raw[1])
            / 1000.0f;

        gyro_z_dps =
            lsm6dsv320x_from_fs500_to_mdps(angular_rate_raw[2])
            / 1000.0f;
    }

    return 0;
}


static int32_t IMU_Calibrate(void)
{
    float bias_sum_x = 0.0f;
    float bias_sum_y = 0.0f;
    float bias_sum_z = 0.0f;
    float quaternion_sum_w = 0.0f;
    float quaternion_sum_x = 0.0f;
    float quaternion_sum_y = 0.0f;
    float quaternion_sum_z = 0.0f;
    lsm6dsv320x_quaternion_t reference_quaternion;
    lsm6dsv320x_quaternion_t sample_quaternion;
    uint32_t calibration_start_ms;
    uint32_t samples = 0U;
    uint8_t have_reference = 0U;

    calibration_start_ms = HAL_GetTick();

    while (samples < CALIBRATION_SAMPLES)
    {
        uint32_t before = imu_sample_count;
        float sign = 1.0f;

        if ((HAL_GetTick() - calibration_start_ms) >
            CALIBRATION_TIMEOUT_MS)
        {
            return -1;
        }

        if (IMU_Read() != 0)
        {
            HAL_Delay(1);
            continue;
        }

        if (imu_sample_count == before)
            continue;

        /*
         * SFLP gravity is in the sensor frame, so the remaining stationary
         * acceleration is the sensor residual bias in that same frame.
         */
        bias_sum_x += acc_x_ms2 - gravity_x_ms2;
        bias_sum_y += acc_y_ms2 - gravity_y_ms2;
        bias_sum_z += acc_z_ms2 - gravity_z_ms2;

        sample_quaternion = orientation_quaternion;

        if (!have_reference)
        {
            reference_quaternion = sample_quaternion;
            have_reference = 1U;
        }
        else if ((sample_quaternion.quat_w * reference_quaternion.quat_w +
                  sample_quaternion.quat_x * reference_quaternion.quat_x +
                  sample_quaternion.quat_y * reference_quaternion.quat_y +
                  sample_quaternion.quat_z * reference_quaternion.quat_z) < 0.0f)
        {
            /* q and -q represent the same orientation; align before averaging. */
            sign = -1.0f;
        }

        quaternion_sum_w += sign * sample_quaternion.quat_w;
        quaternion_sum_x += sign * sample_quaternion.quat_x;
        quaternion_sum_y += sign * sample_quaternion.quat_y;
        quaternion_sum_z += sign * sample_quaternion.quat_z;

        samples++;
    }

    accel_bias_body_x = bias_sum_x / (float)CALIBRATION_SAMPLES;
    accel_bias_body_y = bias_sum_y / (float)CALIBRATION_SAMPLES;
    accel_bias_body_z = bias_sum_z / (float)CALIBRATION_SAMPLES;

    initial_orientation_quaternion.quat_w =
        quaternion_sum_w / (float)CALIBRATION_SAMPLES;
    initial_orientation_quaternion.quat_x =
        quaternion_sum_x / (float)CALIBRATION_SAMPLES;
    initial_orientation_quaternion.quat_y =
        quaternion_sum_y / (float)CALIBRATION_SAMPLES;
    initial_orientation_quaternion.quat_z =
        quaternion_sum_z / (float)CALIBRATION_SAMPLES;

    velocity_x_ms = 0.0f;
    velocity_y_ms = 0.0f;
    velocity_z_ms = 0.0f;

    position_x_m = 0.0f;
    position_y_m = 0.0f;
    position_z_m = 0.0f;

    linear_acc_x_ms2 = 0.0f;
    linear_acc_y_ms2 = 0.0f;
    linear_acc_z_ms2 = 0.0f;

    stationary_sample_count = 0U;
    previous_time_us = __HAL_TIM_GET_COUNTER(&htim2);

    return 0;
}


static uint8_t Is_Stationary(float ax, float ay, float az)
{
    return
        (ax > -STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (ax <  STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (ay > -STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (ay <  STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (az > -STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (az <  STATIONARY_LINEAR_ACCEL_THRESHOLD_MS2) &&
        (gyro_x_dps > -STATIONARY_GYRO_THRESHOLD_DPS) &&
        (gyro_x_dps <  STATIONARY_GYRO_THRESHOLD_DPS) &&
        (gyro_y_dps > -STATIONARY_GYRO_THRESHOLD_DPS) &&
        (gyro_y_dps <  STATIONARY_GYRO_THRESHOLD_DPS) &&
        (gyro_z_dps > -STATIONARY_GYRO_THRESHOLD_DPS) &&
        (gyro_z_dps <  STATIONARY_GYRO_THRESHOLD_DPS);
}


static void Position_Update(void)
{
    uint32_t old_sample;
    uint32_t now_us;
    uint32_t elapsed_us;
    float dt;
    float body_ax;
    float body_ay;
    float body_az;
    float ax;
    float ay;
    float az;

    old_sample = imu_sample_count;

    if (IMU_Read() != 0)
        return;

    if (imu_sample_count == old_sample)
        return;

    now_us = __HAL_TIM_GET_COUNTER(&htim2);

    /* Unsigned subtraction handles TIM2 wrap-around. */
    elapsed_us = now_us - previous_time_us;
    previous_time_us = now_us;

    dt = ((float)elapsed_us) * 0.000001f;

    /* Reject timing gaps caused by breakpoints or temporary stalls. */
    if ((dt <= 0.0f) || (dt > 0.05f))
        return;

    /*
     * Remove the SFLP gravity estimate in the sensor frame. Unlike the old
     * reset-time offset, this vector follows the IMU when it is tilted.
     */
    body_ax = acc_x_ms2 - gravity_x_ms2 - accel_bias_body_x;
    body_ay = acc_y_ms2 - gravity_y_ms2 - accel_bias_body_y;
    body_az = acc_z_ms2 - gravity_z_ms2 - accel_bias_body_z;

    /*
     * Rotate linear acceleration into the frame that existed at reset so
     * velocity and position axes do not rotate with the board.
     */
    if (Body_To_Initial_Frame(body_ax,
                              body_ay,
                              body_az,
                              &ax,
                              &ay,
                              &az) != 0)
    {
        return;
    }

    linear_acc_x_ms2 = ax;
    linear_acc_y_ms2 = ay;
    linear_acc_z_ms2 = az;

    if (Is_Stationary(ax, ay, az))
    {
        if (stationary_sample_count < STATIONARY_DWELL_SAMPLES)
            stationary_sample_count++;

        if (stationary_sample_count >= STATIONARY_DWELL_SAMPLES)
        {
            /*
             * Zero-velocity update. This bounds drift whenever the IMU is
             * placed still, independent of its orientation.
             */
            velocity_x_ms = 0.0f;
            velocity_y_ms = 0.0f;
            velocity_z_ms = 0.0f;

            linear_acc_x_ms2 = 0.0f;
            linear_acc_y_ms2 = 0.0f;
            linear_acc_z_ms2 = 0.0f;
            return;
        }
    }
    else
    {
        stationary_sample_count = 0U;
    }

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
          HAL_Delay(100);
      }
  }

  /*
   * Keep the IMU completely still during calibration. The SFLP also uses
   * this interval to converge its internal gyroscope-bias estimate.
   */
  imu_status = IMU_Calibrate();

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

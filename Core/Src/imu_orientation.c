#include "imu_orientation.h"

#include "FusionAhrs.h"

#include <math.h>

#define IMU_ORIENTATION_DEFAULT_SAMPLE_RATE_HZ 480.0f
#define IMU_ORIENTATION_GAIN 0.5f
#define IMU_ORIENTATION_GYROSCOPE_RANGE_DPS 500.0f
#define IMU_ORIENTATION_ACCELERATION_REJECTION_DEG 10.0f
#define IMU_ORIENTATION_REJECTION_TIMEOUT_S 5.0f
#define MG_PER_G 1000.0f

static FusionAhrs orientation_ahrs;
static float orientation_nominal_sample_period_s =
    1.0f / IMU_ORIENTATION_DEFAULT_SAMPLE_RATE_HZ;

volatile float imu_orientation_roll_deg = 0.0f;
volatile float imu_orientation_pitch_deg = 0.0f;
volatile float imu_orientation_yaw_deg = 0.0f;

volatile float imu_orientation_q_w = 1.0f;
volatile float imu_orientation_q_x = 0.0f;
volatile float imu_orientation_q_y = 0.0f;
volatile float imu_orientation_q_z = 0.0f;

volatile float imu_orientation_sample_period_s =
    1.0f / IMU_ORIENTATION_DEFAULT_SAMPLE_RATE_HZ;
volatile float imu_orientation_sample_rate_hz =
    IMU_ORIENTATION_DEFAULT_SAMPLE_RATE_HZ;
volatile float imu_orientation_acceleration_error_deg = 0.0f;
volatile uint8_t imu_orientation_startup = 1U;
volatile uint8_t imu_orientation_accelerometer_ignored = 0U;
volatile uint8_t imu_orientation_acceleration_recovery = 0U;
volatile uint32_t imu_orientation_update_count = 0U;

void IMU_OrientationInitialise(float sample_rate_hz)
{
  FusionAhrsSettings settings;

  if ((!isfinite(sample_rate_hz)) || (sample_rate_hz <= 0.0f))
    sample_rate_hz = IMU_ORIENTATION_DEFAULT_SAMPLE_RATE_HZ;

  orientation_nominal_sample_period_s = 1.0f / sample_rate_hz;

  FusionAhrsInitialise(&orientation_ahrs);
  settings = (FusionAhrsSettings){
      .sampleRate = sample_rate_hz,
      .convention = FusionConventionNwu,
      .gain = IMU_ORIENTATION_GAIN,
      .gyroscopeRange = IMU_ORIENTATION_GYROSCOPE_RANGE_DPS,
      .accelerationRejection =
          IMU_ORIENTATION_ACCELERATION_REJECTION_DEG,
      .magneticRejection = 0.0f,
      .rejectionTimeout = IMU_ORIENTATION_REJECTION_TIMEOUT_S,
  };
  FusionAhrsSetSettings(&orientation_ahrs, &settings);
  FusionAhrsRestart(&orientation_ahrs);

  imu_orientation_roll_deg = 0.0f;
  imu_orientation_pitch_deg = 0.0f;
  imu_orientation_yaw_deg = 0.0f;
  imu_orientation_q_w = 1.0f;
  imu_orientation_q_x = 0.0f;
  imu_orientation_q_y = 0.0f;
  imu_orientation_q_z = 0.0f;
  imu_orientation_sample_period_s = orientation_nominal_sample_period_s;
  imu_orientation_sample_rate_hz = sample_rate_hz;
  imu_orientation_acceleration_error_deg = 0.0f;
  imu_orientation_startup = 1U;
  imu_orientation_accelerometer_ignored = 0U;
  imu_orientation_acceleration_recovery = 0U;
  imu_orientation_update_count = 0U;
}

void IMU_OrientationUpdate(float gyro_x_dps,
                           float gyro_y_dps,
                           float gyro_z_dps,
                           float accel_x_mg,
                           float accel_y_mg,
                           float accel_z_mg,
                           float sample_period_s)
{
  FusionQuaternion quaternion;
  FusionEuler euler;
  FusionAhrsInternalStates internal_states;
  FusionAhrsFlags flags;
  const FusionVector gyroscope = {
      .axis = {gyro_x_dps, gyro_y_dps, gyro_z_dps}};
  const FusionVector accelerometer = {
      .axis = {accel_x_mg / MG_PER_G,
               accel_y_mg / MG_PER_G,
               accel_z_mg / MG_PER_G}};

  /* Ignore debugger pauses or corrupt timestamps instead of integrating a
   * single gyro sample across an unrealistic interval. */
  if ((!isfinite(sample_period_s)) ||
      (sample_period_s < (0.25f * orientation_nominal_sample_period_s)) ||
      (sample_period_s > (5.0f * orientation_nominal_sample_period_s)))
  {
    sample_period_s = orientation_nominal_sample_period_s;
  }

  FusionAhrsSetSamplePeriod(&orientation_ahrs, sample_period_s);
  FusionAhrsUpdateNoMagnetometer(&orientation_ahrs,
                                 gyroscope,
                                 accelerometer);

  quaternion = FusionAhrsGetQuaternion(&orientation_ahrs);
  euler = FusionQuaternionToEuler(quaternion);
  internal_states = FusionAhrsGetInternalStates(&orientation_ahrs);
  flags = FusionAhrsGetFlags(&orientation_ahrs);

  imu_orientation_q_w = quaternion.element.w;
  imu_orientation_q_x = quaternion.element.x;
  imu_orientation_q_y = quaternion.element.y;
  imu_orientation_q_z = quaternion.element.z;
  imu_orientation_roll_deg = euler.angle.roll;
  imu_orientation_pitch_deg = euler.angle.pitch;
  imu_orientation_yaw_deg = euler.angle.yaw;
  imu_orientation_sample_period_s = sample_period_s;
  imu_orientation_sample_rate_hz = 1.0f / sample_period_s;
  imu_orientation_acceleration_error_deg =
      internal_states.accelerationError;
  imu_orientation_startup = flags.startup ? 1U : 0U;
  imu_orientation_accelerometer_ignored =
      internal_states.accelerometerIgnored ? 1U : 0U;
  imu_orientation_acceleration_recovery =
      flags.accelerationRecovery ? 1U : 0U;
  imu_orientation_update_count++;
}

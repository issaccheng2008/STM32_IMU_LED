#include "imu_orientation.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define SAMPLE_RATE_HZ 480.0f
#define SAMPLE_PERIOD_S (1.0f / SAMPLE_RATE_HZ)

static void update_for_seconds(float seconds,
                               float gx,
                               float gy,
                               float gz,
                               float ax_mg,
                               float ay_mg,
                               float az_mg)
{
  const int updates = (int)(seconds * SAMPLE_RATE_HZ);

  for (int index = 0; index < updates; index++)
  {
    IMU_OrientationUpdate(gx, gy, gz,
                          ax_mg, ay_mg, az_mg,
                          SAMPLE_PERIOD_S);
  }
}

int main(void)
{
  IMU_OrientationInitialise(SAMPLE_RATE_HZ);
  update_for_seconds(4.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 1000.0f);
  assert(fabsf(imu_orientation_roll_deg) < 0.25f);
  assert(fabsf(imu_orientation_pitch_deg) < 0.25f);
  assert(imu_orientation_startup == 0U);

  for (int index = 0; index < (int)SAMPLE_RATE_HZ; index++)
  {
    const float angle_deg = 30.0f * (float)(index + 1) / SAMPLE_RATE_HZ;
    const float angle_rad = angle_deg * 3.14159265358979323846f / 180.0f;

    IMU_OrientationUpdate(30.0f, 0.0f, 0.0f,
                          0.0f,
                          1000.0f * sinf(angle_rad),
                          1000.0f * cosf(angle_rad),
                          SAMPLE_PERIOD_S);
  }
  update_for_seconds(2.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 500.0f, 866.0254f);
  assert(imu_orientation_roll_deg > 29.0f);
  assert(imu_orientation_roll_deg < 31.0f);
  assert(fabsf(imu_orientation_pitch_deg) < 0.5f);

  IMU_OrientationInitialise(SAMPLE_RATE_HZ);
  update_for_seconds(4.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 1000.0f);
  for (int index = 0; index < (int)SAMPLE_RATE_HZ; index++)
  {
    const float angle_deg = 30.0f * (float)(index + 1) / SAMPLE_RATE_HZ;
    const float angle_rad = angle_deg * 3.14159265358979323846f / 180.0f;

    IMU_OrientationUpdate(0.0f, 30.0f, 0.0f,
                          -1000.0f * sinf(angle_rad),
                          0.0f,
                          1000.0f * cosf(angle_rad),
                          SAMPLE_PERIOD_S);
  }
  update_for_seconds(2.0f, 0.0f, 0.0f, 0.0f,
                     -500.0f, 0.0f, 866.0254f);
  assert(fabsf(imu_orientation_roll_deg) < 0.5f);
  assert(imu_orientation_pitch_deg > 29.0f);
  assert(imu_orientation_pitch_deg < 31.0f);

  IMU_OrientationInitialise(SAMPLE_RATE_HZ);
  update_for_seconds(4.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 1000.0f);
  update_for_seconds(1.0f, 0.0f, 0.0f, 0.0f,
                     1000.0f, 0.0f, 1000.0f);
  assert(fabsf(imu_orientation_roll_deg) < 0.5f);
  assert(fabsf(imu_orientation_pitch_deg) < 0.5f);
  assert(imu_orientation_accelerometer_ignored == 1U);

  update_for_seconds(1.0f, 0.0f, 0.0f, 90.0f,
                     0.0f, 0.0f, 1000.0f);
  assert(imu_orientation_yaw_deg > 88.0f);
  assert(imu_orientation_yaw_deg < 92.0f);

  puts("orientation tests passed");
  return 0;
}

#include "imu_calibration_session.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

static uint32_t read_count;

static int32_t fake_read(imu_calibration_measurement_t *measurement)
{
  const uint32_t orientation =
      read_count / IMU_CALIBRATION_SAMPLES_PER_ORIENTATION;

  *measurement = (imu_calibration_measurement_t){0};
  if (orientation < IMU_CALIBRATION_ORIENTATION_COUNT)
  {
    const double z = 1.0 -
                     2.0 * ((double)orientation + 0.5) /
                         IMU_CALIBRATION_ORIENTATION_COUNT;
    const double radius = sqrt(1.0 - z * z);
    const double angle = PI * (3.0 - sqrt(5.0)) * (double)orientation;

    measurement->low_g_mg.x = (float)(18.0 + 985.0 * radius * cos(angle));
    measurement->low_g_mg.y = (float)(-11.0 + 1015.0 * radius * sin(angle));
    measurement->low_g_mg.z = (float)(27.0 + 995.0 * z);
    measurement->high_g_mg.x = (float)(36.0 + 970.0 * radius * cos(angle));
    measurement->high_g_mg.y = (float)(-22.0 + 1030.0 * radius * sin(angle));
    measurement->high_g_mg.z = (float)(54.0 + 990.0 * z);
    measurement->ready_mask = IMU_SAMPLE_LOW_G_READY |
                              IMU_SAMPLE_HIGH_G_READY;
  }
  else
  {
    measurement->gyro_dps = (imu_vector3_t){0.35f, -0.42f, 0.18f};
    measurement->ready_mask = IMU_SAMPLE_GYRO_READY;
  }

  read_count++;
  return 0;
}

static void fake_delay(uint32_t milliseconds)
{
  (void)milliseconds;
}

int main(void)
{
  if (IMU_RunCalibrationSession(fake_read, fake_delay) != 0)
  {
    fprintf(stderr, "session status: %ld\n",
            (long)imu_calibration_session_status);
    return EXIT_FAILURE;
  }

  if ((imu_calibration_orientation_progress !=
       IMU_CALIBRATION_ORIENTATION_COUNT) ||
      (fabs(imu_calibration_last_gyro.gyro_offset_x - 0.35) > 1.0e-6) ||
      (fabs(imu_calibration_last_gyro.gyro_offset_y + 0.42) > 1.0e-6) ||
      (fabs(imu_calibration_last_gyro.gyro_offset_z - 0.18) > 1.0e-6))
  {
    fputs("session results did not match the synthetic input\n", stderr);
    return EXIT_FAILURE;
  }

  puts("imu_calibration full session test passed");
  return EXIT_SUCCESS;
}

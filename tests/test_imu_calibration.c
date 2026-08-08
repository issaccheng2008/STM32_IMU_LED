#include "imu_calibration.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_POINT_COUNT 80U
#define PI 3.14159265358979323846

static int invert_3(const double input[3][3], double inverse[3][3])
{
  const double determinant =
      input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1]) -
      input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0]) +
      input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);

  if (fabs(determinant) < 1.0e-12)
    return -1;

  inverse[0][0] = (input[1][1] * input[2][2] - input[1][2] * input[2][1]) / determinant;
  inverse[0][1] = (input[0][2] * input[2][1] - input[0][1] * input[2][2]) / determinant;
  inverse[0][2] = (input[0][1] * input[1][2] - input[0][2] * input[1][1]) / determinant;
  inverse[1][0] = (input[1][2] * input[2][0] - input[1][0] * input[2][2]) / determinant;
  inverse[1][1] = (input[0][0] * input[2][2] - input[0][2] * input[2][0]) / determinant;
  inverse[1][2] = (input[0][2] * input[1][0] - input[0][0] * input[1][2]) / determinant;
  inverse[2][0] = (input[1][0] * input[2][1] - input[1][1] * input[2][0]) / determinant;
  inverse[2][1] = (input[0][1] * input[2][0] - input[0][0] * input[2][1]) / determinant;
  inverse[2][2] = (input[0][0] * input[1][1] - input[0][1] * input[1][0]) / determinant;
  return 0;
}

static int check_close(const char *name, double actual, double expected, double tolerance)
{
  if (fabs(actual - expected) <= tolerance)
    return 0;

  fprintf(stderr, "%s: actual %.12f, expected %.12f\n", name, actual, expected);
  return -1;
}

int main(void)
{
  const double expected_matrix[3][3] = {
      {1.020, 0.015, -0.008},
      {0.015, 0.980, 0.012},
      {-0.008, 0.012, 1.010},
  };
  const double expected_offset[3] = {23.0, -17.0, 31.0};
  double inverse_matrix[3][3];
  imu_vector3_t points[TEST_POINT_COUNT];
  imu_accel_calibration_t calibration;
  float rms_error_mg;

  if (invert_3(expected_matrix, inverse_matrix) != 0)
    return EXIT_FAILURE;

  /* Deterministic Fibonacci-sphere points provide full three-dimensional coverage. */
  for (uint32_t point = 0U; point < TEST_POINT_COUNT; point++)
  {
    const double z = 1.0 - 2.0 * ((double)point + 0.5) / TEST_POINT_COUNT;
    const double radius = sqrt(1.0 - z * z);
    const double angle = PI * (3.0 - sqrt(5.0)) * (double)point;
    const double truth[3] = {
        1000.0 * radius * cos(angle),
        1000.0 * radius * sin(angle),
        1000.0 * z,
    };
    double raw[3] = {expected_offset[0], expected_offset[1], expected_offset[2]};

    for (uint32_t row = 0U; row < 3U; row++)
    {
      for (uint32_t column = 0U; column < 3U; column++)
        raw[row] += inverse_matrix[row][column] * truth[column];
    }
    points[point] = (imu_vector3_t){(float)raw[0], (float)raw[1], (float)raw[2]};
  }

  if (IMU_AccelFitRotatedEllipsoid(points, TEST_POINT_COUNT,
                                   &calibration, &rms_error_mg) !=
      IMU_CALIBRATION_OK)
  {
    fputs("fit returned an error\n", stderr);
    return EXIT_FAILURE;
  }

  if ((check_close("offset x", calibration.offset_x_mg, expected_offset[0], 0.001) != 0) ||
      (check_close("offset y", calibration.offset_y_mg, expected_offset[1], 0.001) != 0) ||
      (check_close("offset z", calibration.offset_z_mg, expected_offset[2], 0.001) != 0) ||
      (check_close("x gain", calibration.x_gain, expected_matrix[0][0], 1.0e-5) != 0) ||
      (check_close("y to x", calibration.y_to_x, expected_matrix[0][1], 1.0e-5) != 0) ||
      (check_close("z to x", calibration.z_to_x, expected_matrix[0][2], 1.0e-5) != 0) ||
      (check_close("x to y", calibration.x_to_y, expected_matrix[1][0], 1.0e-5) != 0) ||
      (check_close("y gain", calibration.y_gain, expected_matrix[1][1], 1.0e-5) != 0) ||
      (check_close("z to y", calibration.z_to_y, expected_matrix[1][2], 1.0e-5) != 0) ||
      (check_close("x to z", calibration.x_to_z, expected_matrix[2][0], 1.0e-5) != 0) ||
      (check_close("y to z", calibration.y_to_z, expected_matrix[2][1], 1.0e-5) != 0) ||
      (check_close("z gain", calibration.z_gain, expected_matrix[2][2], 1.0e-5) != 0) ||
      (check_close("RMS error", rms_error_mg, 0.0, 0.001) != 0))
    return EXIT_FAILURE;

  puts("imu_calibration synthetic ellipsoid test passed");
  return EXIT_SUCCESS;
}

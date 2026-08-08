#include "imu_calibration.h"

#include <math.h>
#include <stddef.h>

#define IMU_ELLIPSOID_PARAMETER_COUNT 9U
#define IMU_GRAVITY_MG 1000.0
#define IMU_SOLVER_EPSILON 1.0e-12
#define IMU_JACOBI_ITERATIONS 32U

/*
 * Persistent calibration values.  These safe identity/zero defaults leave
 * sensor values unchanged.  Replace them with the paste-ready values printed
 * by the calibration session for this specific board.
 */
const imu_accel_calibration_t imu_low_g_calibration = {
    .offset_x_mg = 0.0f,
    .offset_y_mg = 0.0f,
    .offset_z_mg = 0.0f,
    .x_gain = 1.0f,
    .y_to_x = 0.0f,
    .z_to_x = 0.0f,
    .x_to_y = 0.0f,
    .y_gain = 1.0f,
    .z_to_y = 0.0f,
    .x_to_z = 0.0f,
    .y_to_z = 0.0f,
    .z_gain = 1.0f,
};

const imu_accel_calibration_t imu_high_g_calibration = {
    .offset_x_mg = 0.0f,
    .offset_y_mg = 0.0f,
    .offset_z_mg = 0.0f,
    .x_gain = 1.0f,
    .y_to_x = 0.0f,
    .z_to_x = 0.0f,
    .x_to_y = 0.0f,
    .y_gain = 1.0f,
    .z_to_y = 0.0f,
    .x_to_z = 0.0f,
    .y_to_z = 0.0f,
    .z_gain = 1.0f,
};

const imu_gyro_calibration_t imu_gyro_calibration = {
    .gyro_offset_x = 0.0f,
    .gyro_offset_y = 0.0f,
    .gyro_offset_z = 0.0f,
};

static int32_t solve_linear_9(double matrix[IMU_ELLIPSOID_PARAMETER_COUNT]
                                           [IMU_ELLIPSOID_PARAMETER_COUNT],
                              double vector[IMU_ELLIPSOID_PARAMETER_COUNT],
                              double solution[IMU_ELLIPSOID_PARAMETER_COUNT])
{
  double augmented[IMU_ELLIPSOID_PARAMETER_COUNT]
                  [IMU_ELLIPSOID_PARAMETER_COUNT + 1U];
  uint32_t row;
  uint32_t column;

  for (row = 0U; row < IMU_ELLIPSOID_PARAMETER_COUNT; row++)
  {
    for (column = 0U; column < IMU_ELLIPSOID_PARAMETER_COUNT; column++)
      augmented[row][column] = matrix[row][column];
    augmented[row][IMU_ELLIPSOID_PARAMETER_COUNT] = vector[row];
  }

  for (column = 0U; column < IMU_ELLIPSOID_PARAMETER_COUNT; column++)
  {
    uint32_t pivot_row = column;
    double pivot_magnitude = fabs(augmented[column][column]);

    for (row = column + 1U; row < IMU_ELLIPSOID_PARAMETER_COUNT; row++)
    {
      const double magnitude = fabs(augmented[row][column]);
      if (magnitude > pivot_magnitude)
      {
        pivot_magnitude = magnitude;
        pivot_row = row;
      }
    }

    if (pivot_magnitude < IMU_SOLVER_EPSILON)
      return -1;

    if (pivot_row != column)
    {
      for (uint32_t item = column;
           item <= IMU_ELLIPSOID_PARAMETER_COUNT;
           item++)
      {
        const double temporary = augmented[column][item];
        augmented[column][item] = augmented[pivot_row][item];
        augmented[pivot_row][item] = temporary;
      }
    }

    {
      const double pivot = augmented[column][column];
      for (uint32_t item = column;
           item <= IMU_ELLIPSOID_PARAMETER_COUNT;
           item++)
        augmented[column][item] /= pivot;
    }

    for (row = 0U; row < IMU_ELLIPSOID_PARAMETER_COUNT; row++)
    {
      double factor;

      if (row == column)
        continue;

      factor = augmented[row][column];
      for (uint32_t item = column;
           item <= IMU_ELLIPSOID_PARAMETER_COUNT;
           item++)
        augmented[row][item] -= factor * augmented[column][item];
    }
  }

  for (row = 0U; row < IMU_ELLIPSOID_PARAMETER_COUNT; row++)
    solution[row] = augmented[row][IMU_ELLIPSOID_PARAMETER_COUNT];

  return 0;
}

static int32_t solve_linear_3(double matrix[3][3],
                              double vector[3],
                              double solution[3])
{
  double augmented[3][4];

  for (uint32_t row = 0U; row < 3U; row++)
  {
    for (uint32_t column = 0U; column < 3U; column++)
      augmented[row][column] = matrix[row][column];
    augmented[row][3] = vector[row];
  }

  for (uint32_t column = 0U; column < 3U; column++)
  {
    uint32_t pivot_row = column;
    double pivot_magnitude = fabs(augmented[column][column]);

    for (uint32_t row = column + 1U; row < 3U; row++)
    {
      const double magnitude = fabs(augmented[row][column]);
      if (magnitude > pivot_magnitude)
      {
        pivot_magnitude = magnitude;
        pivot_row = row;
      }
    }

    if (pivot_magnitude < IMU_SOLVER_EPSILON)
      return -1;

    if (pivot_row != column)
    {
      for (uint32_t item = column; item < 4U; item++)
      {
        const double temporary = augmented[column][item];
        augmented[column][item] = augmented[pivot_row][item];
        augmented[pivot_row][item] = temporary;
      }
    }

    {
      const double pivot = augmented[column][column];
      for (uint32_t item = column; item < 4U; item++)
        augmented[column][item] /= pivot;
    }

    for (uint32_t row = 0U; row < 3U; row++)
    {
      const double factor = augmented[row][column];
      if (row == column)
        continue;
      for (uint32_t item = column; item < 4U; item++)
        augmented[row][item] -= factor * augmented[column][item];
    }
  }

  for (uint32_t row = 0U; row < 3U; row++)
    solution[row] = augmented[row][3];

  return 0;
}

static void symmetric_eigen_3(double input[3][3],
                              double eigenvectors[3][3],
                              double eigenvalues[3])
{
  double matrix[3][3];

  for (uint32_t row = 0U; row < 3U; row++)
  {
    for (uint32_t column = 0U; column < 3U; column++)
    {
      matrix[row][column] = input[row][column];
      eigenvectors[row][column] = (row == column) ? 1.0 : 0.0;
    }
  }

  for (uint32_t iteration = 0U; iteration < IMU_JACOBI_ITERATIONS;
       iteration++)
  {
    uint32_t p = 0U;
    uint32_t q = 1U;
    double largest = fabs(matrix[0][1]);

    if (fabs(matrix[0][2]) > largest)
    {
      p = 0U;
      q = 2U;
      largest = fabs(matrix[0][2]);
    }
    if (fabs(matrix[1][2]) > largest)
    {
      p = 1U;
      q = 2U;
      largest = fabs(matrix[1][2]);
    }

    if (largest < IMU_SOLVER_EPSILON)
      break;

    {
      const double app = matrix[p][p];
      const double aqq = matrix[q][q];
      const double apq = matrix[p][q];
      const double tau = (aqq - app) / (2.0 * apq);
      const double t = ((tau >= 0.0) ? 1.0 : -1.0) /
                       (fabs(tau) + sqrt(1.0 + tau * tau));
      const double cosine = 1.0 / sqrt(1.0 + t * t);
      const double sine = t * cosine;

      for (uint32_t index = 0U; index < 3U; index++)
      {
        if ((index != p) && (index != q))
        {
          const double aip = matrix[index][p];
          const double aiq = matrix[index][q];
          matrix[index][p] = cosine * aip - sine * aiq;
          matrix[p][index] = matrix[index][p];
          matrix[index][q] = sine * aip + cosine * aiq;
          matrix[q][index] = matrix[index][q];
        }
      }

      matrix[p][p] = cosine * cosine * app -
                     2.0 * sine * cosine * apq + sine * sine * aqq;
      matrix[q][q] = sine * sine * app +
                     2.0 * sine * cosine * apq + cosine * cosine * aqq;
      matrix[p][q] = 0.0;
      matrix[q][p] = 0.0;

      for (uint32_t row = 0U; row < 3U; row++)
      {
        const double vip = eigenvectors[row][p];
        const double viq = eigenvectors[row][q];
        eigenvectors[row][p] = cosine * vip - sine * viq;
        eigenvectors[row][q] = sine * vip + cosine * viq;
      }
    }
  }

  for (uint32_t index = 0U; index < 3U; index++)
    eigenvalues[index] = matrix[index][index];
}

void IMU_AccelApplyCalibration(const imu_accel_calibration_t *calibration,
                               const imu_vector3_t *raw_mg,
                               imu_vector3_t *calibrated_mg)
{
  const float x = raw_mg->x - calibration->offset_x_mg;
  const float y = raw_mg->y - calibration->offset_y_mg;
  const float z = raw_mg->z - calibration->offset_z_mg;

  calibrated_mg->x = calibration->x_gain * x +
                     calibration->y_to_x * y +
                     calibration->z_to_x * z;
  calibrated_mg->y = calibration->x_to_y * x +
                     calibration->y_gain * y +
                     calibration->z_to_y * z;
  calibrated_mg->z = calibration->x_to_z * x +
                     calibration->y_to_z * y +
                     calibration->z_gain * z;
}

void IMU_GyroApplyCalibration(const imu_gyro_calibration_t *calibration,
                              const imu_vector3_t *raw_dps,
                              imu_vector3_t *calibrated_dps)
{
  calibrated_dps->x = raw_dps->x - calibration->gyro_offset_x;
  calibrated_dps->y = raw_dps->y - calibration->gyro_offset_y;
  calibrated_dps->z = raw_dps->z - calibration->gyro_offset_z;
}

imu_calibration_status_t IMU_AccelFitRotatedEllipsoid(
    const imu_vector3_t *points_mg,
    uint32_t point_count,
    imu_accel_calibration_t *calibration,
    float *rms_magnitude_error_mg)
{
  double normal_matrix[IMU_ELLIPSOID_PARAMETER_COUNT]
                      [IMU_ELLIPSOID_PARAMETER_COUNT] = {{0.0}};
  double normal_vector[IMU_ELLIPSOID_PARAMETER_COUNT] = {0.0};
  double alternative_solution[IMU_ELLIPSOID_PARAMETER_COUNT];
  double coefficients[IMU_ELLIPSOID_PARAMETER_COUNT];
  double quadratic[3][3];
  double linear[3];
  double center_rhs[3];
  double center[3];
  double translated_quadratic[3][3];
  double eigenvectors[3][3];
  double eigenvalues[3];
  double correction[3][3] = {{0.0}};
  double centered_scale;
  double sum_squared_error = 0.0;

  if ((points_mg == NULL) || (calibration == NULL))
    return IMU_CALIBRATION_INVALID_ARGUMENT;
  if (point_count < IMU_ELLIPSOID_PARAMETER_COUNT)
    return IMU_CALIBRATION_NOT_ENOUGH_POINTS;

  /*
   * DT0059 alternate formulation:
   * D = [x2+y2-2z2, x2-2y2+z2, 4xy, 2xz, 2yz,
   *      2x, 2y, 2z, 1], E = x2+y2+z2.
   * Coordinates are normalized to g so all columns stay near unity.
   */
  for (uint32_t point = 0U; point < point_count; point++)
  {
    const double x = (double)points_mg[point].x / IMU_GRAVITY_MG;
    const double y = (double)points_mg[point].y / IMU_GRAVITY_MG;
    const double z = (double)points_mg[point].z / IMU_GRAVITY_MG;
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;
    const double row[IMU_ELLIPSOID_PARAMETER_COUNT] = {
        x2 + y2 - 2.0 * z2,
        x2 - 2.0 * y2 + z2,
        4.0 * x * y,
        2.0 * x * z,
        2.0 * y * z,
        2.0 * x,
        2.0 * y,
        2.0 * z,
        1.0,
    };
    const double rhs = x2 + y2 + z2;

    for (uint32_t row_index = 0U;
         row_index < IMU_ELLIPSOID_PARAMETER_COUNT;
         row_index++)
    {
      normal_vector[row_index] += row[row_index] * rhs;
      for (uint32_t column_index = 0U;
           column_index < IMU_ELLIPSOID_PARAMETER_COUNT;
           column_index++)
      {
        normal_matrix[row_index][column_index] +=
            row[row_index] * row[column_index];
      }
    }
  }

  if (solve_linear_9(normal_matrix, normal_vector,
                     alternative_solution) != 0)
    return IMU_CALIBRATION_SINGULAR_FIT;

  {
    const double transformed[10] = {
        -1.0 + alternative_solution[0] + alternative_solution[1],
        -1.0 + alternative_solution[0] - 2.0 * alternative_solution[1],
        -1.0 - 2.0 * alternative_solution[0] + alternative_solution[1],
        2.0 * alternative_solution[2],
        alternative_solution[3],
        alternative_solution[4],
        alternative_solution[5],
        alternative_solution[6],
        alternative_solution[7],
        alternative_solution[8],
    };

    if (fabs(transformed[9]) < IMU_SOLVER_EPSILON)
      return IMU_CALIBRATION_SINGULAR_FIT;

    for (uint32_t index = 0U;
         index < IMU_ELLIPSOID_PARAMETER_COUNT;
         index++)
      coefficients[index] = -transformed[index] / transformed[9];
  }

  quadratic[0][0] = coefficients[0];
  quadratic[0][1] = coefficients[3];
  quadratic[0][2] = coefficients[4];
  quadratic[1][0] = coefficients[3];
  quadratic[1][1] = coefficients[1];
  quadratic[1][2] = coefficients[5];
  quadratic[2][0] = coefficients[4];
  quadratic[2][1] = coefficients[5];
  quadratic[2][2] = coefficients[2];
  linear[0] = coefficients[6];
  linear[1] = coefficients[7];
  linear[2] = coefficients[8];
  center_rhs[0] = -linear[0];
  center_rhs[1] = -linear[1];
  center_rhs[2] = -linear[2];

  if (solve_linear_3(quadratic, center_rhs, center) != 0)
    return IMU_CALIBRATION_SINGULAR_FIT;

  centered_scale = 1.0;
  for (uint32_t row = 0U; row < 3U; row++)
  {
    for (uint32_t column = 0U; column < 3U; column++)
      centered_scale += center[row] * quadratic[row][column] * center[column];
  }

  if (centered_scale <= IMU_SOLVER_EPSILON)
    return IMU_CALIBRATION_NOT_AN_ELLIPSOID;

  for (uint32_t row = 0U; row < 3U; row++)
  {
    for (uint32_t column = 0U; column < 3U; column++)
      translated_quadratic[row][column] =
          quadratic[row][column] / centered_scale;
  }

  symmetric_eigen_3(translated_quadratic, eigenvectors, eigenvalues);
  for (uint32_t index = 0U; index < 3U; index++)
  {
    if (eigenvalues[index] <= IMU_SOLVER_EPSILON)
      return IMU_CALIBRATION_NOT_AN_ELLIPSOID;
  }

  /*
   * The symmetric positive-definite square root maps the ellipsoid to a
   * sphere while choosing the least-rotation solution.  This avoids arbitrary
   * eigenvector swaps/sign flips and yields a directly usable correction
   * matrix whose diagonal and off-diagonal terms are the requested gains.
   */
  for (uint32_t row = 0U; row < 3U; row++)
  {
    for (uint32_t column = 0U; column < 3U; column++)
    {
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        correction[row][column] += eigenvectors[row][axis] *
                                   sqrt(eigenvalues[axis]) *
                                   eigenvectors[column][axis];
      }
    }
  }

  calibration->offset_x_mg = (float)(center[0] * IMU_GRAVITY_MG);
  calibration->offset_y_mg = (float)(center[1] * IMU_GRAVITY_MG);
  calibration->offset_z_mg = (float)(center[2] * IMU_GRAVITY_MG);
  calibration->x_gain = (float)correction[0][0];
  calibration->y_to_x = (float)correction[0][1];
  calibration->z_to_x = (float)correction[0][2];
  calibration->x_to_y = (float)correction[1][0];
  calibration->y_gain = (float)correction[1][1];
  calibration->z_to_y = (float)correction[1][2];
  calibration->x_to_z = (float)correction[2][0];
  calibration->y_to_z = (float)correction[2][1];
  calibration->z_gain = (float)correction[2][2];

  for (uint32_t point = 0U; point < point_count; point++)
  {
    imu_vector3_t corrected;
    double magnitude_error;

    IMU_AccelApplyCalibration(calibration, &points_mg[point], &corrected);
    magnitude_error = sqrt((double)corrected.x * corrected.x +
                           (double)corrected.y * corrected.y +
                           (double)corrected.z * corrected.z) -
                      IMU_GRAVITY_MG;
    sum_squared_error += magnitude_error * magnitude_error;
  }

  if (rms_magnitude_error_mg != NULL)
    *rms_magnitude_error_mg =
        (float)sqrt(sum_squared_error / (double)point_count);

  return IMU_CALIBRATION_OK;
}

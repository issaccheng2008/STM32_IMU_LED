#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** A three-axis value expressed in the units documented by the caller. */
typedef struct
{
  float x;
  float y;
  float z;
} imu_vector3_t;

/**
 * Accelerometer offset and 3x3 correction matrix.
 *
 * Matrix rows are calibrated output axes and columns are offset-corrected
 * input axes.  For example:
 *
 *   calibrated_x = x_gain * (raw_x - offset_x_mg)
 *                + y_to_x * (raw_y - offset_y_mg)
 *                + z_to_x * (raw_z - offset_z_mg)
 *
 * The three diagonal terms are axis scale gains.  The six off-diagonal terms
 * compensate cross-axis coupling.  Raw and calibrated acceleration are in mg.
 */
typedef struct
{
  float offset_x_mg;
  float offset_y_mg;
  float offset_z_mg;

  float x_gain;
  float y_to_x;
  float z_to_x;

  float x_to_y;
  float y_gain;
  float z_to_y;

  float x_to_z;
  float y_to_z;
  float z_gain;
} imu_accel_calibration_t;

/** Gyroscope zero-rate offsets in degrees per second. */
typedef struct
{
  float gyro_offset_x;
  float gyro_offset_y;
  float gyro_offset_z;
} imu_gyro_calibration_t;

typedef enum
{
  IMU_CALIBRATION_OK = 0,
  IMU_CALIBRATION_NOT_ENOUGH_POINTS = -1,
  IMU_CALIBRATION_SINGULAR_FIT = -2,
  IMU_CALIBRATION_NOT_AN_ELLIPSOID = -3,
  IMU_CALIBRATION_INVALID_ARGUMENT = -4
} imu_calibration_status_t;

/*
 * Paste the calibration session's generated initializers over these three
 * definitions in Core/Src/imu_calibration.c after calibrating the board.
 */
extern const imu_accel_calibration_t imu_low_g_calibration;
extern const imu_accel_calibration_t imu_high_g_calibration;
extern const imu_gyro_calibration_t imu_gyro_calibration;

/** Apply offset, axis-gain, and cross-axis compensation to an accelerometer. */
void IMU_AccelApplyCalibration(const imu_accel_calibration_t *calibration,
                               const imu_vector3_t *raw_mg,
                               imu_vector3_t *calibrated_mg);

/** Apply zero-rate offset compensation to the gyroscope. */
void IMU_GyroApplyCalibration(const imu_gyro_calibration_t *calibration,
                              const imu_vector3_t *raw_dps,
                              imu_vector3_t *calibrated_dps);

/**
 * Fit DT0059's alternate rotated-ellipsoid system for near-spherical data.
 * Each input point is one stationary, averaged acceleration measurement in mg.
 * rms_magnitude_error_mg may be NULL.
 */
imu_calibration_status_t IMU_AccelFitRotatedEllipsoid(
    const imu_vector3_t *points_mg,
    uint32_t point_count,
    imu_accel_calibration_t *calibration,
    float *rms_magnitude_error_mg);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */

#ifndef IMU_CALIBRATION_SESSION_H
#define IMU_CALIBRATION_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_calibration.h"

#include <stdint.h>

/** Number of stationary orientations collected over the full sphere. */
#define IMU_CALIBRATION_ORIENTATION_COUNT 40U

/** Fresh samples averaged into each accelerometer calibration point. */
#define IMU_CALIBRATION_SAMPLES_PER_ORIENTATION 256U

/** Fresh stationary gyroscope samples used for its zero-rate mean. */
#define IMU_CALIBRATION_GYRO_SAMPLE_COUNT 1200U

/** Reject an orientation if low-g three-axis RMS noise exceeds this limit. */
#define IMU_CALIBRATION_MAX_STATIONARY_RMS_MG 12.0

/** Semihosted path relative to the CubeIDE launch working directory. */
#define IMU_CALIBRATION_SAMPLE_FILE "calibration/imu_calibration_samples.csv"

/** Fallback filename if a debugger ignores the configured working directory. */
#define IMU_CALIBRATION_SAMPLE_FILE_FALLBACK "imu_calibration_samples.csv"

#define IMU_SAMPLE_LOW_G_READY (1U << 0)
#define IMU_SAMPLE_HIGH_G_READY (1U << 1)
#define IMU_SAMPLE_GYRO_READY (1U << 2)

/** One polling result; ready_mask says which fields contain a fresh sample. */
typedef struct
{
  imu_vector3_t low_g_mg;
  imu_vector3_t high_g_mg;
  imu_vector3_t gyro_dps;
  uint8_t ready_mask;
} imu_calibration_measurement_t;

typedef int32_t (*imu_calibration_read_fn_t)(
    imu_calibration_measurement_t *measurement);
typedef void (*imu_calibration_delay_fn_t)(uint32_t milliseconds);

/** Live Expressions: accepted accelerometer orientations, from 0 to 40. */
extern volatile uint32_t imu_calibration_orientation_progress;

/** Live Expressions: final status; 0 indicates a successful calibration. */
extern volatile int32_t imu_calibration_session_status;

/** Most recent low-g fit, also printed as a paste-ready C initializer. */
extern volatile imu_accel_calibration_t imu_calibration_last_low_g;

/** Most recent high-g fit, also printed as a paste-ready C initializer. */
extern volatile imu_accel_calibration_t imu_calibration_last_high_g;

/** Most recent gyro mean, also printed as a paste-ready C initializer. */
extern volatile imu_gyro_calibration_t imu_calibration_last_gyro;

/**
 * Run the interactive, debugger-hosted calibration session.
 *
 * CubeIDE semihosting must be enabled.  The function prints countdowns and
 * results in the debugger console and writes all averaged points to the CSV
 * file above.  It returns after printing the constants.
 */
int32_t IMU_RunCalibrationSession(imu_calibration_read_fn_t read_measurement,
                                  imu_calibration_delay_fn_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_SESSION_H */

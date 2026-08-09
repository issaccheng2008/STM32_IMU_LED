#ifndef IMU_ORIENTATION_H
#define IMU_ORIENTATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * CubeMonitor outputs.  Roll, pitch, and yaw use the ZYX convention and are
 * expressed in degrees.  Roll is rotation about sensor X, pitch about sensor
 * Y, and yaw about sensor Z.  Without a magnetometer, yaw is relative to boot
 * and will drift; roll and pitch remain referenced to gravity.
 */
extern volatile float imu_orientation_roll_deg;
extern volatile float imu_orientation_pitch_deg;
extern volatile float imu_orientation_yaw_deg;

/** Orientation quaternion in scalar-first (w, x, y, z) order. */
extern volatile float imu_orientation_q_w;
extern volatile float imu_orientation_q_x;
extern volatile float imu_orientation_q_y;
extern volatile float imu_orientation_q_z;

/** Most recent accepted filter step and its reciprocal. */
extern volatile float imu_orientation_sample_period_s;
extern volatile float imu_orientation_sample_rate_hz;

/** Angle between measured acceleration and the filter's gravity estimate. */
extern volatile float imu_orientation_acceleration_error_deg;

/** Nonzero while the first three-second high-gain convergence is active. */
extern volatile uint8_t imu_orientation_startup;

/** Nonzero when motion acceleration is being rejected for this update. */
extern volatile uint8_t imu_orientation_accelerometer_ignored;

/** Nonzero during automatic recovery from prolonged acceleration rejection. */
extern volatile uint8_t imu_orientation_acceleration_recovery;

/** Number of complete, synchronized accel/gyro samples processed. */
extern volatile uint32_t imu_orientation_update_count;

void IMU_OrientationInitialise(float sample_rate_hz);

void IMU_OrientationUpdate(float gyro_x_dps,
                           float gyro_y_dps,
                           float gyro_z_dps,
                           float accel_x_mg,
                           float accel_y_mg,
                           float accel_z_mg,
                           float sample_period_s);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ORIENTATION_H */

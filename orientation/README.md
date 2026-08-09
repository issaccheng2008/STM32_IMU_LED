# IMU orientation output

This firmware estimates orientation relative to gravity with the current
[x-io Fusion](https://github.com/xioTechnologies/Fusion) AHRS algorithm, the
successor to Sebastian Madgwick's original 2010 gradient-descent filter.  It
uses the calibrated LSM6DSV320X low-g accelerometer and gyroscope.  No
magnetometer is required.

## CubeMonitor variables

Add these symbols to STM32CubeMonitor:

| Variable | Meaning |
| --- | --- |
| `imu_orientation_roll_deg` | Rotation about sensor X, in degrees |
| `imu_orientation_pitch_deg` | Rotation about sensor Y, in degrees |
| `imu_orientation_yaw_deg` | Gyro-integrated rotation about sensor Z; it will drift |
| `imu_orientation_q_w` ... `imu_orientation_q_z` | Scalar-first orientation quaternion |
| `imu_orientation_sample_rate_hz` | Instantaneous rate of complete accel/gyro updates |
| `imu_orientation_acceleration_error_deg` | Difference between measured and estimated gravity directions |
| `imu_orientation_accelerometer_ignored` | `1` while motion acceleration is rejected |
| `imu_orientation_startup` | `1` during the initial three-second convergence |
| `imu_orientation_update_count` | Number of complete filter updates |

Keep the board still for the first three seconds after reset.  Roll and pitch
are zero when sensor +Z points upward and X/Y lie in the ground plane.  If the
IMU module is mounted in another orientation, remap its axes before calling
`IMU_OrientationUpdate`.

## Configuration

The settings are in `Core/Src/imu_orientation.c`:

- 480 Hz nominal sample rate, matching the configured sensor ODR
- gain 0.5
- +/-500 dps gyroscope range
- 10 degree acceleration-rejection threshold
- 5 second rejection-recovery timeout
- NWU (X north, Y west, Z up) Earth convention

TIM2 runs at 1 MHz and measures the actual time between synchronized
accelerometer/gyroscope samples.  Implausible intervals caused by a debugger
pause are replaced with the nominal interval instead of being integrated.

## Why Fusion instead of the supplied 2010 code

Both algorithms integrate a quaternion from the gyroscope and use gravity to
correct roll and pitch.  The original IMU code applies accelerometer correction
on every valid sample with a fixed `beta`.  Fusion uses a complementary
feedback gain, fast startup convergence, measured sample periods, gyroscope
overrange recovery, and acceleration rejection.  The last feature is the most
important upgrade here: linear acceleration that disagrees strongly with the
current gravity estimate is temporarily ignored, reducing false roll/pitch
changes during motion.

The lack of a magnetometer remains fundamental: gravity cannot observe rotation
about the vertical axis, so yaw has no absolute reference and will drift.

The vendored Fusion source is from upstream commit
`5c200f17579bb01335f070da4e3bf5c6b58387c8`.  Its license is stored at
`LICENSES/Fusion-LICENSE.md`.

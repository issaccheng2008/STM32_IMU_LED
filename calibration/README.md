# LSM6DSV320X calibration

This project calibrates the low-g and high-g accelerometers independently with
the numerically improved rotated-ellipsoid formulation in ST DT0059. It uses 40
stationary orientations without requiring known angles. The gyroscope receives
zero-rate offset calibration only.

## Collect calibration data in CubeIDE

1. Build the **Debug** configuration and start `LED-test Debug` with the
   ST-Link/V2. The committed launch configuration enables semihosting and uses
   the project directory as its working directory.
2. When the debugger stops at `main`, add `imu_run_calibration_on_boot` to Live
   Expressions and set it to `1` before resuming.
3. Follow the debugger-console prompts. For every orientation, place the board
   at a different angle, take your hands off it, and wait for the `3, 2, 1`
   countdown. The firmware averages 256 fresh samples from each accelerometer.
   Captures with more than 12 mg low-g RMS movement/noise are repeated.
4. Distribute the 40 orientations over the full sphere. Include each face,
   edge, and corner direction rather than collecting many nearly identical
   points.
5. After both ellipsoids are fitted, leave the board still through the 10-second
   gyro countdown and collection of 1200 fresh gyro samples.

The session writes every accepted pair of averaged accelerometer points and the
gyro mean to `calibration/imu_calibration_samples.csv`. It also prints three
paste-ready initializers. Replace the matching identity/zero definitions near
the top of `Core/Src/imu_calibration.c`, rebuild, and leave
`imu_run_calibration_on_boot` at its normal value of `0`.

If CubeIDE ignores the launch working directory, the firmware falls back to
`imu_calibration_samples.csv` in the debugger server's current directory and
prints that filename in the console.

The normal polling loop then exposes both raw physical readings and these
calibrated values in Live Expressions:

- `imu_accel_{x,y,z}_calibrated_mg`
- `imu_high_g_{x,y,z}_calibrated_mg`
- `imu_gyro_{x,y,z}_calibrated_dps`

## Refit without collecting again

The CSV is intentionally kept in the repository. After changing calibration
math, rerun the fit on the same captured points with:

```sh
python calibration/fit_imu_calibration.py calibration/imu_calibration_samples.csv
```

The script uses only the Python standard library and prints replacement C
initializers. Keep a backup of the populated CSV before starting a new session,
because a new calibration opens it in overwrite mode.

## Interpretation and limits

Each accelerometer has its own offset and 3x3 correction matrix. Matrix diagonal
entries are X/Y/Z scale gains; off-diagonal entries are the six named cross-axis
terms. The fit uses gravity magnitude only, so it cannot identify an arbitrary
overall rigid rotation of the complete sensor coordinate frame. The symmetric
positive-definite square-root used here selects the least-rotation correction
and avoids eigenvector swaps or sign flips.

At +/-32 g, the high-g sensor measures 1 g over only a small part of its range
and has much higher noise than the low-g channel. Averaging and full-sphere
coverage help, but its gravity-only calibration will naturally be less precise.

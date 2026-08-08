#include "imu_calibration_session.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SEMIHOST_SYS_OPEN 0x01U
#define SEMIHOST_SYS_CLOSE 0x02U
#define SEMIHOST_SYS_WRITE0 0x04U
#define SEMIHOST_SYS_WRITE 0x05U
#define SEMIHOST_OPEN_WRITE_TEXT 4U

#define CALIBRATION_POLL_DELAY_MS 1U
#define CALIBRATION_MAX_IDLE_POLLS 30000U
#define CALIBRATION_TEXT_CAPACITY 1024U

typedef struct
{
  char text[CALIBRATION_TEXT_CAPACITY];
  uint32_t length;
} calibration_text_t;

volatile uint32_t imu_calibration_orientation_progress = 0U;
volatile int32_t imu_calibration_session_status = -1;
volatile imu_accel_calibration_t imu_calibration_last_low_g = {0};
volatile imu_accel_calibration_t imu_calibration_last_high_g = {0};
volatile imu_gyro_calibration_t imu_calibration_last_gyro = {0};

static int32_t semihost_call(uint32_t operation, const void *parameter)
{
#if defined(__arm__) || defined(__thumb__)
  register uint32_t register_r0 __asm("r0") = operation;
  register const void *register_r1 __asm("r1") = parameter;

  __asm volatile("bkpt 0xAB"
                 : "+r"(register_r0)
                 : "r"(register_r1)
                 : "memory");
  return (int32_t)register_r0;
#else
  (void)operation;
  (void)parameter;
  return -1;
#endif
}

static void debug_write(const char *text)
{
  (void)semihost_call(SEMIHOST_SYS_WRITE0, text);
}

static int32_t sample_file_open(const char *name, uint32_t name_length)
{
  struct
  {
    const char *name;
    uint32_t mode;
    uint32_t name_length;
  } arguments;

  arguments.name = name;
  arguments.mode = SEMIHOST_OPEN_WRITE_TEXT;
  arguments.name_length = name_length;
  return semihost_call(SEMIHOST_SYS_OPEN, &arguments);
}

static int32_t sample_file_write(int32_t handle,
                                 const char *text,
                                 uint32_t length)
{
  struct
  {
    int32_t handle;
    const char *buffer;
    uint32_t length;
  } arguments;

  arguments.handle = handle;
  arguments.buffer = text;
  arguments.length = length;
  return semihost_call(SEMIHOST_SYS_WRITE, &arguments);
}

static void sample_file_close(int32_t handle)
{
  (void)semihost_call(SEMIHOST_SYS_CLOSE, &handle);
}

static void text_reset(calibration_text_t *output)
{
  output->length = 0U;
  output->text[0] = '\0';
}

static void text_append_char(calibration_text_t *output, char character)
{
  if (output->length + 1U >= CALIBRATION_TEXT_CAPACITY)
    return;
  output->text[output->length++] = character;
  output->text[output->length] = '\0';
}

static void text_append(calibration_text_t *output, const char *text)
{
  while (*text != '\0')
  {
    text_append_char(output, *text);
    text++;
  }
}

static void text_append_u64(calibration_text_t *output, uint64_t value)
{
  char reversed[21];
  uint32_t count = 0U;

  do
  {
    reversed[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (count < sizeof(reversed)));

  while (count > 0U)
    text_append_char(output, reversed[--count]);
}

static void text_append_u32(calibration_text_t *output, uint32_t value)
{
  text_append_u64(output, value);
}

static void text_append_fixed(calibration_text_t *output,
                              double value,
                              uint32_t decimal_places)
{
  uint64_t scale = 1U;
  uint64_t scaled;

  if (!isfinite(value))
  {
    text_append(output, "invalid");
    return;
  }

  if (value < 0.0)
  {
    text_append_char(output, '-');
    value = -value;
  }

  for (uint32_t digit = 0U; digit < decimal_places; digit++)
    scale *= 10U;

  scaled = (uint64_t)(value * (double)scale + 0.5);
  text_append_u64(output, scaled / scale);

  if (decimal_places != 0U)
  {
    uint64_t fraction = scaled % scale;
    uint64_t divisor = scale / 10U;

    text_append_char(output, '.');
    while (divisor != 0U)
    {
      text_append_char(output,
                       (char)('0' + (fraction / divisor) % 10U));
      divisor /= 10U;
    }
  }
}

static void print_countdown(const char *message,
                            uint32_t seconds,
                            imu_calibration_delay_fn_t delay_ms)
{
  calibration_text_t output;

  debug_write(message);
  for (uint32_t remaining = seconds; remaining > 0U; remaining--)
  {
    text_reset(&output);
    text_append(&output, "  ");
    text_append_u32(&output, remaining);
    text_append(&output, "...\r\n");
    debug_write(output.text);
    delay_ms(1000U);
  }
}

static int32_t collect_orientation(imu_calibration_read_fn_t read_measurement,
                                   imu_calibration_delay_fn_t delay_ms,
                                   imu_vector3_t *low_g_point,
                                   imu_vector3_t *high_g_point,
                                   double *stationary_rms_mg)
{
  double low_sum[3] = {0.0, 0.0, 0.0};
  double low_sum_squared[3] = {0.0, 0.0, 0.0};
  double high_sum[3] = {0.0, 0.0, 0.0};
  uint32_t low_count = 0U;
  uint32_t high_count = 0U;
  uint32_t idle_polls = 0U;

  while ((low_count < IMU_CALIBRATION_SAMPLES_PER_ORIENTATION) ||
         (high_count < IMU_CALIBRATION_SAMPLES_PER_ORIENTATION))
  {
    imu_calibration_measurement_t measurement = {0};
    const int32_t status = read_measurement(&measurement);
    uint8_t progressed = 0U;

    if (status != 0)
      return status;

    if (((measurement.ready_mask & IMU_SAMPLE_LOW_G_READY) != 0U) &&
        (low_count < IMU_CALIBRATION_SAMPLES_PER_ORIENTATION))
    {
      const double values[3] = {
          measurement.low_g_mg.x,
          measurement.low_g_mg.y,
          measurement.low_g_mg.z,
      };

      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        low_sum[axis] += values[axis];
        low_sum_squared[axis] += values[axis] * values[axis];
      }
      low_count++;
      progressed = 1U;
    }

    if (((measurement.ready_mask & IMU_SAMPLE_HIGH_G_READY) != 0U) &&
        (high_count < IMU_CALIBRATION_SAMPLES_PER_ORIENTATION))
    {
      high_sum[0] += measurement.high_g_mg.x;
      high_sum[1] += measurement.high_g_mg.y;
      high_sum[2] += measurement.high_g_mg.z;
      high_count++;
      progressed = 1U;
    }

    if (progressed != 0U)
      idle_polls = 0U;
    else
      idle_polls++;
    if (idle_polls > CALIBRATION_MAX_IDLE_POLLS)
      return -100;

    delay_ms(CALIBRATION_POLL_DELAY_MS);
  }

  low_g_point->x = (float)(low_sum[0] / (double)low_count);
  low_g_point->y = (float)(low_sum[1] / (double)low_count);
  low_g_point->z = (float)(low_sum[2] / (double)low_count);
  high_g_point->x = (float)(high_sum[0] / (double)high_count);
  high_g_point->y = (float)(high_sum[1] / (double)high_count);
  high_g_point->z = (float)(high_sum[2] / (double)high_count);

  {
    double summed_variance = 0.0;
    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
      const double mean = low_sum[axis] / (double)low_count;
      double variance = low_sum_squared[axis] / (double)low_count -
                        mean * mean;
      if (variance < 0.0)
        variance = 0.0;
      summed_variance += variance;
    }
    *stationary_rms_mg = sqrt(summed_variance);
  }

  return 0;
}

static void append_accel_initializer(calibration_text_t *output,
                                     const char *name,
                                     const imu_accel_calibration_t *value)
{
#define APPEND_FIELD(field_name)                                                \
  do                                                                            \
  {                                                                             \
    text_append(output, "    ." #field_name " = ");                            \
    text_append_fixed(output, value->field_name, 9U);                           \
    text_append(output, "f,\r\n");                                             \
  } while (0)

  text_append(output, "const imu_accel_calibration_t ");
  text_append(output, name);
  text_append(output, " = {\r\n");
  APPEND_FIELD(offset_x_mg);
  APPEND_FIELD(offset_y_mg);
  APPEND_FIELD(offset_z_mg);
  APPEND_FIELD(x_gain);
  APPEND_FIELD(y_to_x);
  APPEND_FIELD(z_to_x);
  APPEND_FIELD(x_to_y);
  APPEND_FIELD(y_gain);
  APPEND_FIELD(z_to_y);
  APPEND_FIELD(x_to_z);
  APPEND_FIELD(y_to_z);
  APPEND_FIELD(z_gain);
  text_append(output, "};\r\n\r\n");

#undef APPEND_FIELD
}

static void print_accel_initializer(const char *name,
                                    const imu_accel_calibration_t *value)
{
  calibration_text_t output;

  text_reset(&output);
  append_accel_initializer(&output, name, value);
  debug_write(output.text);
}

static void print_gyro_initializer(const imu_gyro_calibration_t *value)
{
  calibration_text_t output;

  text_reset(&output);
  text_append(&output,
              "const imu_gyro_calibration_t imu_gyro_calibration = {\r\n"
              "    .gyro_offset_x = ");
  text_append_fixed(&output, value->gyro_offset_x, 9U);
  text_append(&output, "f,\r\n    .gyro_offset_y = ");
  text_append_fixed(&output, value->gyro_offset_y, 9U);
  text_append(&output, "f,\r\n    .gyro_offset_z = ");
  text_append_fixed(&output, value->gyro_offset_z, 9U);
  text_append(&output, "f,\r\n};\r\n");
  debug_write(output.text);
}

static void build_csv_point(calibration_text_t *output,
                            uint32_t index,
                            const imu_vector3_t *low_g,
                            const imu_vector3_t *high_g)
{
  text_reset(output);
  text_append_u32(output, index);
  text_append_char(output, ',');
  text_append_fixed(output, low_g->x, 6U);
  text_append_char(output, ',');
  text_append_fixed(output, low_g->y, 6U);
  text_append_char(output, ',');
  text_append_fixed(output, low_g->z, 6U);
  text_append_char(output, ',');
  text_append_fixed(output, high_g->x, 6U);
  text_append_char(output, ',');
  text_append_fixed(output, high_g->y, 6U);
  text_append_char(output, ',');
  text_append_fixed(output, high_g->z, 6U);
  text_append(output, "\r\n");
}

static int32_t collect_gyro(imu_calibration_read_fn_t read_measurement,
                            imu_calibration_delay_fn_t delay_ms,
                            imu_gyro_calibration_t *gyro)
{
  double sum[3] = {0.0, 0.0, 0.0};
  uint32_t count = 0U;
  uint32_t idle_polls = 0U;

  while (count < IMU_CALIBRATION_GYRO_SAMPLE_COUNT)
  {
    imu_calibration_measurement_t measurement = {0};
    const int32_t status = read_measurement(&measurement);

    if (status != 0)
      return status;

    if ((measurement.ready_mask & IMU_SAMPLE_GYRO_READY) != 0U)
    {
      sum[0] += measurement.gyro_dps.x;
      sum[1] += measurement.gyro_dps.y;
      sum[2] += measurement.gyro_dps.z;
      count++;
      idle_polls = 0U;
    }
    else
    {
      idle_polls++;
    }

    if (idle_polls > CALIBRATION_MAX_IDLE_POLLS)
      return -101;
    delay_ms(CALIBRATION_POLL_DELAY_MS);
  }

  gyro->gyro_offset_x = (float)(sum[0] / (double)count);
  gyro->gyro_offset_y = (float)(sum[1] / (double)count);
  gyro->gyro_offset_z = (float)(sum[2] / (double)count);
  return 0;
}

int32_t IMU_RunCalibrationSession(imu_calibration_read_fn_t read_measurement,
                                  imu_calibration_delay_fn_t delay_ms)
{
  imu_vector3_t low_g_points[IMU_CALIBRATION_ORIENTATION_COUNT];
  imu_vector3_t high_g_points[IMU_CALIBRATION_ORIENTATION_COUNT];
  imu_accel_calibration_t low_g_fit;
  imu_accel_calibration_t high_g_fit;
  imu_gyro_calibration_t gyro_fit;
  float low_g_rms = 0.0f;
  float high_g_rms = 0.0f;
  int32_t sample_file;
  const char *sample_file_path = IMU_CALIBRATION_SAMPLE_FILE;
  calibration_text_t output;
  int32_t status = 0;

  if ((read_measurement == NULL) || (delay_ms == NULL))
    return IMU_CALIBRATION_INVALID_ARGUMENT;

  imu_calibration_orientation_progress = 0U;
  imu_calibration_session_status = -1;
  debug_write("\r\n=== LSM6DSV320X calibration ===\r\n"
              "Keep the board completely stationary during each capture.\r\n"
              "Move it to a new orientation only after a point is accepted.\r\n");

  sample_file = sample_file_open(
      IMU_CALIBRATION_SAMPLE_FILE,
      (uint32_t)(sizeof(IMU_CALIBRATION_SAMPLE_FILE) - 1U));
  if (sample_file < 0)
  {
    sample_file_path = IMU_CALIBRATION_SAMPLE_FILE_FALLBACK;
    sample_file = sample_file_open(
        IMU_CALIBRATION_SAMPLE_FILE_FALLBACK,
        (uint32_t)(sizeof(IMU_CALIBRATION_SAMPLE_FILE_FALLBACK) - 1U));
  }
  if (sample_file < 0)
  {
    debug_write("WARNING: Could not open a CSV sample file. The fit will "
                "continue, but points will not be saved.\r\n");
  }
  else
  {
    static const char header[] =
        "index,low_g_x_mg,low_g_y_mg,low_g_z_mg,high_g_x_mg,"
        "high_g_y_mg,high_g_z_mg\r\n";
    (void)sample_file_write(sample_file, header,
                            (uint32_t)(sizeof(header) - 1U));
  }

  for (uint32_t orientation = 0U;
       orientation < IMU_CALIBRATION_ORIENTATION_COUNT;)
  {
    double stationary_rms_mg;

    text_reset(&output);
    text_append(&output, "\r\nOrientation ");
    text_append_u32(&output, orientation + 1U);
    text_append(&output, " of ");
    text_append_u32(&output, IMU_CALIBRATION_ORIENTATION_COUNT);
    text_append(&output, ": place the board, then hands off.\r\n");
    debug_write(output.text);
    print_countdown("Capture begins in:\r\n", 3U, delay_ms);

    status = collect_orientation(read_measurement, delay_ms,
                                 &low_g_points[orientation],
                                 &high_g_points[orientation],
                                 &stationary_rms_mg);
    if (status != 0)
      goto session_finished;

    if (stationary_rms_mg > IMU_CALIBRATION_MAX_STATIONARY_RMS_MG)
    {
      text_reset(&output);
      text_append(&output, "Movement detected (low-g RMS ");
      text_append_fixed(&output, stationary_rms_mg, 3U);
      text_append(&output, " mg). Repeating this orientation.\r\n");
      debug_write(output.text);
      continue;
    }

    build_csv_point(&output, orientation + 1U,
                    &low_g_points[orientation],
                    &high_g_points[orientation]);
    if (sample_file >= 0)
      (void)sample_file_write(sample_file, output.text, output.length);

    text_reset(&output);
    text_append(&output, "Accepted point ");
    text_append_u32(&output, orientation + 1U);
    text_append(&output, " (stationary RMS ");
    text_append_fixed(&output, stationary_rms_mg, 3U);
    text_append(&output, " mg).\r\n");
    debug_write(output.text);

    orientation++;
    imu_calibration_orientation_progress = orientation;
  }

  debug_write("\r\nFitting low-g and high-g rotated ellipsoids...\r\n");
  status = IMU_AccelFitRotatedEllipsoid(
      low_g_points, IMU_CALIBRATION_ORIENTATION_COUNT,
      &low_g_fit, &low_g_rms);
  if (status != IMU_CALIBRATION_OK)
    goto session_finished;

  status = IMU_AccelFitRotatedEllipsoid(
      high_g_points, IMU_CALIBRATION_ORIENTATION_COUNT,
      &high_g_fit, &high_g_rms);
  if (status != IMU_CALIBRATION_OK)
    goto session_finished;

  print_countdown("\r\nAccelerometers fitted. Keep the board stationary.\r\n"
                  "Gyroscope zero-rate collection begins in:\r\n",
                  10U, delay_ms);
  status = collect_gyro(read_measurement, delay_ms, &gyro_fit);
  if (status != 0)
    goto session_finished;

  if (sample_file >= 0)
  {
    text_reset(&output);
    text_append(&output, "# gyro_offset_dps,");
    text_append_fixed(&output, gyro_fit.gyro_offset_x, 9U);
    text_append_char(&output, ',');
    text_append_fixed(&output, gyro_fit.gyro_offset_y, 9U);
    text_append_char(&output, ',');
    text_append_fixed(&output, gyro_fit.gyro_offset_z, 9U);
    text_append(&output, "\r\n");
    (void)sample_file_write(sample_file, output.text, output.length);
  }

  imu_calibration_last_low_g = low_g_fit;
  imu_calibration_last_high_g = high_g_fit;
  imu_calibration_last_gyro = gyro_fit;

  text_reset(&output);
  text_append(&output, "\r\nFit RMS magnitude error: low-g = ");
  text_append_fixed(&output, low_g_rms, 3U);
  text_append(&output, " mg, high-g = ");
  text_append_fixed(&output, high_g_rms, 3U);
  text_append(&output, " mg.\r\n");
  if (sample_file >= 0)
  {
    text_append(&output, "Saved averaged points to ");
    text_append(&output, sample_file_path);
  }
  else
  {
    text_append(&output, "WARNING: Averaged points were not saved");
  }
  text_append(&output,
              ".\r\n\r\n"
              "Paste these definitions into Core/Src/imu_calibration.c:\r\n\r\n");
  debug_write(output.text);
  print_accel_initializer("imu_low_g_calibration", &low_g_fit);
  print_accel_initializer("imu_high_g_calibration", &high_g_fit);
  print_gyro_initializer(&gyro_fit);
  debug_write("\r\nCalibration complete. Rebuild after pasting the constants.\r\n");
  status = 0;

session_finished:
  if (sample_file >= 0)
    sample_file_close(sample_file);

  if (status != 0)
  {
    text_reset(&output);
    text_append(&output, "\r\nCalibration stopped with status ");
    if (status < 0)
    {
      text_append_char(&output, '-');
      text_append_u32(&output, (uint32_t)(-status));
    }
    else
    {
      text_append_u32(&output, (uint32_t)status);
    }
    text_append(&output, ". Check imu_status and sensor wiring.\r\n");
    debug_write(output.text);
  }

  imu_calibration_session_status = status;
  return status;
}

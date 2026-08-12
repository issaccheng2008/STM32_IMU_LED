# WAND1 binary file format

WAND1 is the fixed-layout command format shared by the browser converter and
the STM32 IMU POV-wand firmware.  Its payload is ready for direct transmission
to a 35-pixel SK9822 strip.

## Conventions

- All multi-byte integers are little-endian.
- Every file covers -90° through +90°, including both endpoints.
- 0° is upright, negative is counterclockwise, and positive is clockwise.
- Frame order is increasing wand angle.
- Fine angles use microdegrees (`µdeg`); signed endpoints use millidegrees
  (`mdeg`).
- CRC32 is the IEEE reflected algorithm with polynomial `0xEDB88320`, initial
  value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`.

## Header: 48 bytes

| Offset | Size | Type | Field | Required value or meaning |
|---:|---:|---|---|---|
| 0 | 4 | char[4] | `magic` | ASCII `WAND` |
| 4 | 2 | uint16 | `version` | `1` |
| 6 | 2 | uint16 | `header_bytes` | `48` |
| 8 | 2 | uint16 | `led_count` | `35` |
| 10 | 2 | uint16 | `frame_bytes` | `148` |
| 12 | 4 | uint32 | `sample_count` | 5–1,801 angle frames |
| 16 | 4 | int32 | `min_angle_mdeg` | `-90000` |
| 20 | 4 | int32 | `max_angle_mdeg` | `90000` |
| 24 | 4 | uint32 | `angle_step_udeg` | `round(180000000 / (sample_count - 1))` |
| 28 | 4 | uint32 | `payload_bytes` | `sample_count × 148` |
| 32 | 4 | uint32 | `payload_crc32` | CRC32 over the complete payload |
| 36 | 1 | uint8 | `encoding` | `1` = complete SK9822 wire frames |
| 37 | 1 | uint8 | `reserved` | writer sets `0`; reader ignores |
| 38 | 2 | uint16 | `flags` | required bits are `0x0007` |
| 40 | 4 | uint32 | `file_bytes` | `48 + payload_bytes` |
| 44 | 4 | uint32 | `header_crc32` | CRC32 over bytes 0–43 |

### Required flag bits

| Bit | Meaning when set |
|---:|---|
| 0 | Each payload item is one complete SK9822 wire frame |
| 1 | Both -90° and +90° endpoints are stored |
| 2 | Increasing frame angle is clockwise from upright |
| 3–15 | Reserved; writers set zero and readers may ignore |

## Payload

The payload begins at byte 48 and contains `sample_count` adjacent frames.  A
frame is exactly 148 bytes:

| Frame offset | Size | Content |
|---:|---:|---|
| 0 | 4 | SK9822 start frame: `00 00 00 00` |
| 4 | 140 | 35 pixel commands in LED 0→34 order |
| 144 | 4 | SK9822 end frame: `FF FF FF FF` |

Each four-byte pixel command is already in wire order:

| Pixel offset | Meaning |
|---:|---|
| 0 | `111BBBBB`, where `BBBBB` is global brightness 0–31 |
| 1 | Blue, 0–255 |
| 2 | Green, 0–255 |
| 3 | Red, 0–255 |

The STM32 sends all 148 bytes without RGB conversion:

```c
frame = payload + sample_index * 148u;
HAL_SPI_Transmit(&hspi1, frame, 148u, timeout_ms);
```

## Angle lookup

Clamp the measured wand angle to the stored range, convert its offset from
-90° to microdegrees, and round to the nearest sample:

```c
angle_mdeg = clamp(angle_mdeg, -90000, 90000);
relative_udeg = (angle_mdeg - (-90000)) * 1000;
sample_index = (relative_udeg + angle_step_udeg / 2) /
               angle_step_udeg;
sample_index = min(sample_index, sample_count - 1);
```

For the physical IMU mounting used by this project, Fusion roll is converted
before lookup:

```text
wand angle = normalize(-90° - Fusion roll), then clamp to ±90°
```

This gives `roll -90° → wand 0°`, `roll 0° → wand -90°`, and
`roll -180° → wand +90°`.

## Example sizes

| Requested/actual interval | Frames | Payload | Complete file |
|---:|---:|---:|---:|
| 45° | 5 | 740 B | 788 B |
| 1° | 181 | 26,788 B | 26,836 B |
| 0.5° | 361 | 53,428 B | 53,476 B |
| 0.1° | 1,801 | 266,548 B | 266,596 B |

If the requested interval does not divide 180° exactly, the converter rounds
the interval count and stores the adjusted interval in the header.

## Firmware validation order

The included parser and loader perform these checks before playback:

1. Magic, version, 48-byte header, fixed 35-pixel/148-byte layout, encoding,
   flags, endpoints, sample-count range, and exact derived angle step.
2. Header CRC32.
3. Overflow-safe payload and complete-file lengths.
4. Exact file size on the FAT volume.
5. Full payload CRC32.
6. Four start bytes, four end bytes, and the `111` prefix of every pixel command
   in every frame.

The full payload is then kept in internal RAM, so no filesystem operation is
part of angle-to-LED playback.

## Human-readable translation

`WAND.json` is not a firmware input.  It records the source and geometry,
sampling and motion settings, and for every angle:

- `wire_frame_hex`: the exact 148 bytes in the binary payload;
- `global_brightness` and decoded `[red, green, blue]` values for each LED;
- `sk9822_bytes_hex`: the corresponding four-byte pixel command.

The JSON is intentionally redundant so a binary frame can be audited without
special tooling.

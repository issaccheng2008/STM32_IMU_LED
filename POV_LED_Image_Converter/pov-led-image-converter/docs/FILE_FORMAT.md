# POV1 binary file format

POV1 is a compact, little-endian, angle-major RGB format designed for direct indexed reads from a microSD card on an STM32.

## Conventions

- All multi-byte integers are **little-endian**.
- Distances use integer micrometres (`µm`).
- Fine angles use microdegrees (`µdeg`); placement angles use millidegrees (`mdeg`).
- The payload channel order is **R, G, B**.
- A frame contains all LEDs for one mechanical angle.
- Frames appear in increasing mechanical-angle-index order.
- CRC32 is the standard IEEE reflected algorithm: polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, final XOR `0xFFFFFFFF`.

## Header (80 bytes)

| Offset | Size | Type | Field | Meaning |
|---:|---:|---|---|---|
| 0 | 4 | char[4] | magic | ASCII `POV1` |
| 4 | 2 | uint16 | version | `1` |
| 6 | 2 | uint16 | header_bytes | `80` |
| 8 | 2 | uint16 | led_count | LEDs in every frame |
| 10 | 1 | uint8 | channels | `3` |
| 11 | 1 | uint8 | encoding | `0` = raw RGB8 |
| 12 | 4 | uint32 | sample_count | Angle frames in one 360° revolution |
| 16 | 4 | uint32 | angle_step_udeg | Mechanical interval in microdegrees |
| 20 | 4 | uint32 | payload_bytes | Must equal `sample_count × led_count × 3` |
| 24 | 4 | uint32 | payload_crc32 | CRC32 of the complete payload |
| 28 | 4 | int32 | strip_length_um | LED 0 center to last LED center |
| 32 | 4 | int32 | pivot_to_led0_um | Signed radius of LED 0 |
| 36 | 4 | int32 | image_center_x_um | Source placement metadata |
| 40 | 4 | int32 | image_center_y_um | Source placement metadata |
| 44 | 4 | uint32 | image_width_um | Source physical width metadata |
| 48 | 4 | uint32 | image_height_um | Source physical height metadata |
| 52 | 4 | int32 | image_rotation_mdeg | Clockwise rotation metadata |
| 56 | 4 | int32 | zero_angle_mdeg | Hardware 0° clockwise from up |
| 60 | 4 | uint32 | flags | See below |
| 64 | 4 | uint32 | led_pitch_um | Derived center-to-center pitch |
| 68 | 4 | int32 | led0_radius_um | Same physical quantity as pivot-to-LED-0; explicit runtime alias |
| 72 | 1 | uint8 | global_brightness | Suggested SK9822 brightness, 0–31 |
| 73 | 1 | uint8 | rgb_order | `0` = RGB |
| 74 | 2 | uint16 | reserved | `0`; ignore when reading |
| 76 | 4 | uint32 | header_crc32 | CRC32 over header bytes 0 through 75 |

### Flags

| Bit | Meaning when set |
|---:|---|
| 0 | Increasing frame angle is clockwise |
| 1 | Bilinear source sampling was used |
| 2 | RGB brightness scaling is already applied to the payload |
| 3 | Source alpha was composited over black |
| 4–31 | Reserved; readers must ignore |

## Payload

The raw payload starts at byte 80. Its byte index is:

```c
payload_index = ((angle_index * led_count) + led_index) * 3;
red   = payload[payload_index + 0];
green = payload[payload_index + 1];
blue  = payload[payload_index + 2];
```

A complete frame is `led_count × 3` bytes and can be read in one operation.

## Angle lookup

For an encoder angle expressed in microdegrees in the range `[0, 360000000)`:

```c
angle_index = ((angle_udeg + angle_step_udeg / 2) / angle_step_udeg) % sample_count;
```

This selects the nearest stored frame. If your sensor's angle convention is opposite to the exported direction, either change the converter's direction setting or reverse the runtime angle before lookup. Do not reverse both.

## Example sizes for 35 LEDs

| Interval | Frames | RGB payload | Complete file |
|---:|---:|---:|---:|
| 2° | 180 | 18,900 B | 18,980 B |
| 1° | 360 | 37,800 B | 37,880 B |
| 0.5° | 720 | 75,600 B | 75,680 B |
| 0.1° | 3,600 | 378,000 B | 378,080 B |

## Validation order

Recommended firmware startup validation:

1. Check `magic`, `version`, `header_bytes`, `channels`, and `encoding`.
2. Verify `header_crc32`.
3. Check `payload_bytes == sample_count × led_count × 3` with overflow protection.
4. Check the SD file size is `header_bytes + payload_bytes`.
5. Optionally verify the payload CRC once when loading the file.

Per-frame CRCs are intentionally omitted to keep frame lookup simple. If power-loss corruption is a concern, validate the full payload before starting the motor.


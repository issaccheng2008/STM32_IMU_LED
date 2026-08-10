# Firmware implementation

The former standalone POV1 RGB decoder was removed because its 360° fan format
and runtime RGB-to-SK9822 conversion no longer match this project.

The production WAND1 implementation is compiled directly by the STM32 project:

- `../../../Core/Inc/wand_file.h` and `../../../Core/Src/wand_file.c` — portable
  header/CRC validation, roll mapping, and nearest-angle lookup;
- `../../../Core/Inc/wand_storage.h` and `../../../Core/Src/wand_storage.c` —
  read-only FatFs preload of `WAND.POV`;
- `../../../Core/Inc/sk9822.h` and `../../../Core/Src/sk9822.c` — direct 148-byte
  frame transmission.

The converter's `npm test` command compiles that same `wand_file.c` on the host
and checks it against a fixture written by the JavaScript exporter.

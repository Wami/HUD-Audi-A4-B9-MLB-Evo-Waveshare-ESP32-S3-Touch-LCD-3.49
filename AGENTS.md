# AGENTS.md

## Project
ESP32-S3 Arduino firmware for a Waveshare ESP32-S3-Touch-LCD-3.49 V2 HUD (Audi MLB-Evo / VAG MQB).
It only listens to the car CAN bus via a BLE sniffer/gateway or its onboard TWAI transceiver.
**Never add CAN transmission** — the design is passive (TWAI `LISTEN_ONLY`, transceiver silent).

## Build and flash
- No CMake, PlatformIO or package.json. A prepared `arduino-cli` toolchain is at `/tmp/opencode/bin/arduino-cli`; core `esp32:esp32 3.3.10`; libraries in `~/Arduino/libraries` — LVGL 8.4.x, NimBLE-Arduino 2.5.2 (1.x callbacks differ), RTClib, Adafruit_BusIO. Pure-C modules also have host tests (see below).
- FQBN: `esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,CDCOnBoot=cdc`.
- Compile from `waveshare_hud_mockup/`:
  `arduino-cli compile --fqbn "<FQBN>" --build-path /tmp/opencode/hud-build-ble/build --output-dir /tmp/opencode/hud-build-ble/out .`
- Flash (substitute the serial port):
  `arduino-cli upload --fqbn "<FQBN>" --port /dev/ttyACM0 --input-dir /tmp/opencode/hud-build-ble/out .`
- Serial monitor at 115200. `arduino-cli` lives in `/tmp/opencode/bin`; add it to `PATH` or call it by absolute path.
- Partition scheme: `16M Flash (3MB APP/9.9MB FATFS)`. PSRAM: OPI — LVGL draw buffers and optional data files live there.
- BLE limits are baked into the core, not the sketch: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3` (verified in `esp32s3-libs/3.3.10/qio_*/include/sdkconfig.h`), all four roles enabled. NimBLE-Arduino 2.5.2 has no `setMaxConnections()` API. Central (CAN sniffer) and GATT peripheral can connect simultaneously within that budget.
- `lv_conf.h` goes beside the `lvgl` library folder, not inside it. `LV_USE_FONT_COMPRESSED 1` is mandatory or generated fonts silently render nothing.
- `secrets.h` is gitignored. Copy `secrets.example.h`; never commit real keys.

## Architecture
- `setup()` wires everything; `loop()` is intentionally idle.
- Data path: `hud_source.cpp` (BLE/TWAI/FAKE/AUTO) → `can_decode_frame()` in `can_decode.c` → `HudData` under `portMUX` → `hud_data_snapshot()` → LVGL timers in `hud_mockup.c`.
- Phone overlay: `hud_update_cb` calls `nav_bridge_apply()` on the local `HudData` copy; it maps G1→`man_*`, G5→`sign_raw`/`nav_bridge_speed_limit()`, G8→`dest_*`/`tte_*`, G9→`gps_*`. Fresh phone groups win over BAP/VZE/PSD; `mode=idle` leaves CAN/BAP visible. G6 is exposed by `nav_bridge_camera()` and used by `cams_update` only when the menu source is phone or both.
- `can_decode.c` is the single decoder for all sources. Add a signal in `hud_data.h` (`HudData` + `V_*`) and `can_decode.c`; for BLE also add its ID to the ACL in `ble_can_client.cpp`, otherwise it never arrives.
- **Never call LVGL outside its timers** (`hud_update_cb`, `anim_timer_cb`, menu/OTA timers). CAN/BLE/audio run on other tasks and cores; direct LVGL calls race.
- `.c` files are C, `.cpp`/`.ino` are C++. Headers shared by both need `extern "C"` guards (see `hud_data.h`).
- Platform differences use `#if HUD_PLATFORM == HUD_PLATFORM_MQB`. MQB lacks several MLB frames (ACC_12/14, 0x64F wheel, pre-sense), so check both paths when changing decoding.
- A BLE navigation channel from an Android phone (GATT server, parallel to `HUD_DATA_SOURCE`, data takes priority over BAP) is frozen in `../protocol/spec.md` (v1), with `../protocol/schema/protocol.json` and `../protocol/vectors/golden.json`. Implemented: `ble_nav_server.cpp` (GATT peripheral, `HUD_NAV_SERVER_ENABLE`), `nav_proto.c` (pure C decoder) and `nav_bridge.c` (overlay onto BAP/VZE fields: G1/G5/G8/G9; G6 cached for the camera block). `hud_update_cb` calls `nav_bridge_apply()` after `hud_data_snapshot()`, so phone data wins over BAP/VZE/PSD while fresh. Camera source is a menu option (`HUD_CAM_SRC` / NVS `camsrc`: local db / phone / both) handled in `cams_update`. Don't invent a different protocol or transport.

## Configuration traps
- `waveshare_hud_mockup/hud_config.h` is the source of truth. The README "Default" tables and comments have been out of sync with the actual defines (data source and `HUD_PLATFORM` have differed); verify before assuming.
- Menu values persist in NVS (`hud_settings.cpp`). `HUD_LANG`, `HUD_UNITS`, `HUD_PSD_*`, `HUD_ACCEL_BAR`, volume etc. are first-boot defaults only; changing them won't affect an already-configured unit.
- Display geometry/rotation/mirroring is in `user_config.h` (`Rotated = USER_DISP_ROT_90`, `HUD_LCD_MIRROR_X`). The mirror flips touch input too.
- Use `arduino_printf()` / `hud_log_write()` for logs; a bare `printf` can go to UART0 instead of USB CDC. Diagnostics are gated by `HUD_LOG_*` in `hud_config.h`.

## Generated files — do not hand-edit
- `hud_images.c/.h`, `hud_nav_images.c/.h`, `hud_font_*.c`, `hud_sounds.c/.h`.
- `nav_proto_generated.h` and `tools/nav_proto_vectors.h` are generated from `../protocol/`; run `python3 ../protocol/tools/gen.py` from the repo root.
- Run generators from the repo root; they write directly into `waveshare_hud_mockup/`:
  - `python3 tools/gen_hud_images.py` (Pillow)
  - `python3 tools/gen_nav_images.py` (Pillow)
  - `python3 tools/fonts/make_fonts.py` (fontTools + `lv_font_conv`)
  - `python3 tools/make_sounds.py` (ffmpeg)
- The font script inserts `#define LV_LVGL_H_INCLUDE_SIMPLE`; add it yourself if running `lv_font_conv` manually.
- `arrows/arrows.bin` is a prebuilt binary; regenerate with `tools/gen_arrow_anim.py` only when source frames are available.

## LVGL 8 graphics pitfalls
- Images are alpha masks; color comes from `img_recolor`.
- `img_recolor` is ignored unless `img_recolor_opa > 0`; create images via `car_img()` in `hud_mockup.c`.
- Generated fonts contain only the characters listed in `tools/fonts/make_fonts.py`; missing glyphs render as boxes.

## Data files and diagnostics
- `/cams.bin` and `/arrows.bin` are optional FATFS files loaded into PSRAM at boot. Upload them via BOOT hold → `http://192.168.4.1`. Missing files silently disable the feature.
- Camera DB format: `docs/cams_format.md`; build: `python3 tools/gen_cams.py ... -o cams.bin`.
- Verify camera logic on a PC:
  `gcc -O2 -o cam_replay tools/cam_replay.c waveshare_hud_mockup/cam_alert.c -Iwaveshare_hud_mockup -lm && ./cam_replay cams.bin can_log_0004.txt`
- Verify the phone protocol decoder on a PC:
  `gcc -O2 -std=c11 -Wall -Wextra -Werror -o /tmp/nav_proto_test tools/nav_proto_test.c waveshare_hud_mockup/nav_proto.c -Iwaveshare_hud_mockup -lm && /tmp/nav_proto_test`
- Verify decoder edge cases on a PC:
  `gcc -O2 -std=c11 -Wall -Wextra -Werror -o /tmp/nav_proto_edge_test tools/nav_proto_edge_test.c waveshare_hud_mockup/nav_proto.c -Iwaveshare_hud_mockup -lm && /tmp/nav_proto_edge_test`
- Validate the frozen contract and generated files on a PC:
  `python3 ../protocol/tools/check.py`
- Run the contract check and all pure-C host tests in one command:
  `bash ../protocol/tools/test.sh`
- Verify the phone → HudData bridge on a PC:
  `gcc -O2 -std=c11 -Wall -Wextra -Werror -o /tmp/nav_bridge_test tools/nav_bridge_test.c waveshare_hud_mockup/nav_bridge.c waveshare_hud_mockup/nav_proto.c -Iwaveshare_hud_mockup -lm && /tmp/nav_bridge_test`
- Inspect changing CAN bits:
  `python3 tools/can_bitdiff.py can_log_0001.txt.lzma 0x397 0x31E --ignore 0-11`
- `hud_diag_t` (NVS `hudd`, `HUD_DIAG_VER 3`) also stores phone-channel counters (`nav_snapshots`, `nav_positions`, `nav_crc`, `nav_unknown`, `nav_bad_len`, per-group `nav_groups[12]`) and the camera source; the OTA diagnostics page shows them.

## Conventions
- Code comments, log strings, and much documentation are Russian. `README.md` and `README_ru.md` are parallel; update both when behavior/docs change.
- HUD strings are RU/EN (`hud_font_menu` contains both).
- Leave `src/` alone (vendored Waveshare/Espressif BSP and codec) unless the task is about the BSP.
- Bump `HUD_FW_VERSION` in `hud_config.h` for releases; MAJOR for incompatible formats or a new platform.

# Firmware — ESP32-C3 + BNO085

Reads the BNO085's fused rotation vector + accelerometer over I2C and streams a 32-byte
packet over BLE notify (~50 Hz) to the visionOS app.

## Two sketches

| Sketch | What it is |
|--------|-----------|
| `esp32_tracker/esp32_tracker.ino` | The base tracker (no display). |
| `esp32_tracker_display/esp32_tracker_display.ino` | Same tracker + a 128x64 OLED showing hand, BLE state, calibration, quaternion, and accel — live, no serial monitor needed. Use this for untethered/battery boards. |

Both speak the **identical** BLE protocol (UUIDs + packet below), so the visionOS app and the PC
dashboard work with either. They live in separate folders because the Arduino IDE concatenates every
`.ino` in a sketch folder into one build — open each from its own folder.

## Wiring (I2C)

| BNO085 | ESP32-C3 SuperMini |
|--------|--------------------|
| VIN    | 3V3                |
| GND    | GND                |
| SDA    | GPIO0              |
| SCL    | GPIO1              |

```
   BNO085 breakout                 ESP32-C3 SuperMini
  ┌───────────────┐               ┌───────────────────┐
  │           VIN ●───────────────● 3V3                │
  │           GND ●───────────────● GND                │
  │           SDA ●───────────────● GPIO0  (SDA)       │
  │           SCL ●───────────────● GPIO1  (SCL)       │
  │                               │                    │
  │  RST INT P0 P1  ← unused      │  [USB-C]  ← to Mac │
  └───────────────┘               └───────────────────┘
```

Only these four wires. Leave the BNO085's RST / INT / P0 / P1 / PS pins unconnected — the firmware
uses plain I2C polling. Power from **3V3**, not 5V (the C3's GPIOs are 3.3 V logic). Adafruit/SparkFun
breakouts already have I2C pull-ups, so none to add.

GPIO0/1 are used (not GPIO8/9): on the SuperMini, GPIO8 is the onboard LED and GPIO8/9 are boot
strapping pins, so GPIO0/1 are the cleaner choice for I2C. Confirm against your board's silkscreen;
SuperMini clones vary. The pins are set in the sketch (`PIN_SDA` / `PIN_SCL`), so change them there if
your board differs.

### OLED (display sketch only)

The display sketch adds a 128x64 OLED (GME12864 / SSD1306) on the **same hardware I2C bus as the
BNO085** — they coexist by address (OLED `0x3C`, BNO `0x4B`), and the bus runs at 400 kHz:

| OLED | ESP32-C3 SuperMini |
|------|--------------------|
| VCC  | 3V3                |
| GND  | GND                |
| SDA  | GPIO0 (shared with BNO085 SDA) |
| SCL  | GPIO1 (shared with BNO085 SCL) |

> Don't use a separate software-I2C bus for the OLED: a bitbanged full-frame redraw takes hundreds
> of ms (~1 fps) and blocks the loop, stuttering the 50 Hz BLE stream. On hardware I2C @ 400 kHz a
> redraw is ~25 ms. (`Wire.setClock(400000)` + `display.setBusClock(400000)` — the latter stops U8g2
> from dropping the bus back to its 100 kHz default during display transfers.)

Both the BNO085 and the OLED share the 3V3 pin and a common GND (~35 mA total — well within the
regulator's budget). The screen shows `LEFT`/`RIGHT` + `ADV`/`CONN` + `C:n` (calibration) on top, then
quaternion w/x/y/z (left column) and accel x/y/z (right column), refreshed at 10 Hz.
Sanity check: board flat and still → `az` reads ≈ +9.8 (gravity).

If text appears garbled/shifted, the module is the 1.3" **SH1106** variant — swap `SSD1306` → `SH1106`
in the display constructor (noted in the sketch).

## Two boards: left + right hand

This firmware drives two identical trackers. One line near the top of the sketch picks which hand a
board is, which sets its advertised BLE name:

```cpp
#define IS_LEFT_HAND 1   // 1 = "Left Hand Tracker"   |   0 = "Right Hand Tracker"
```

Flash board 1 with `1`, change to `0` and flash board 2. Both boards share the **same** service and
characteristic UUIDs — only the name differs, which is how the app tells them apart. The name is sent
in the BLE **scan response** (it's too long to fit in the main advertising packet next to the 128-bit
service UUID), so a scanner shows the full "Left/Right Hand Tracker" name.

## Troubleshooting

These bit us during first bring-up on a real SuperMini — check here before suspecting hardware.

- **Serial monitor shows the ROM boot log but nothing from the sketch** (`BNO08x ready` etc. never
  appear). The SuperMini has only a *native USB* port (no UART bridge chip), so `Serial` must be routed
  over USB. `platformio.ini` sets `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` for this. Without
  those flags, `Serial.println()` goes to the physical UART0 pins and the USB monitor stays blank.
- **`BNO08x not found` even though the sensor is wired correctly.** Two causes, both handled in the
  sketch now:
  - **Address.** Many BNO085 breakouts sit at **0x4B**, not the library default 0x4A. `BNO_ADDR` is set
    to 0x4B; flip it back to 0x4A if a bus scan says so.
  - **Cold-boot NACK.** The BNO085 often ignores its *first* I2C transaction after power-up, so a single
    `begin_I2C()` probe fails on a perfectly good sensor. `setup()` adds a startup delay and retries
    `begin_I2C()` up to 5×.
- **Not sure if the sensor is even on the bus?** Drop a quick I2C scan into `setup()` after
  `Wire.begin(PIN_SDA, PIN_SCL)` to list responding addresses:
  ```cpp
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("I2C device at 0x%02X\n", a);
  }
  ```
  Nothing found → wiring/power (re-check the four joints, SDA/SCL not swapped). A device at 0x4A/0x4B →
  set `BNO_ADDR` to match.
- **Upload fails with "port is busy".** Close any open serial monitor (the PlatformIO 🔌 button, or a
  stray `pio device monitor`) — only one program can hold the port. After a reset the native-USB port
  may also rename (e.g. `usbmodemXXXX` → `usbmodem101`); PlatformIO auto-detects it.

## Build — PlatformIO (recommended)
```
cd firmware
pio run -t upload
pio device monitor      # 115200 baud
```

## Build — Arduino IDE
1. Install ESP32 board support (Boards Manager → "esp32" by Espressif, **version 2.0.17** — the
   3.x core crashes with NimBLE 1.4.x at BLE startup: `Guru Meditation` / `MEPC: 0x00000000`).
2. Library Manager → install **Adafruit BNO08x** and **NimBLE-Arduino** (use **1.4.x**, not 2.x —
   the server-callback signatures changed). For the display sketch, also install **U8g2** (by oliver).
3. Open `esp32_tracker/esp32_tracker.ino` (or `esp32_tracker_display/esp32_tracker_display.ino`),
   select board **ESP32C3 Dev Module**, set **USB CDC On Boot: Enabled** (this resets when you
   change cores!), set `IS_LEFT_HAND` for the board in hand, upload.
4. Keep the Arduino sketchbook on a plain local path (e.g. `C:\Arduino`) — a OneDrive-synced
   sketchbook causes random `Permission denied` errors on library headers during compile.

## Bring-up order (matches SPEC.md milestones)
1. **Serial first.** After flashing, the monitor should print `BNO08x ready` then
   `BLE advertising as Left Hand Tracker` (or `Right Hand Tracker`, per `IS_LEFT_HAND`). If you want to
   eyeball the quaternion, temporarily add a `Serial.printf` of `pkt.w/x/y/z` in `loop()`.
   (With the display sketch, the OLED shows the same bring-up states on the board itself —
   `Starting...` → `BLE advertising` → live data, or `BNO085 NOT FOUND` on a wiring fault.)
2. **Verify BLE with a scanner.** Use **nRF Connect** (iOS/Android) — confirm devices named
   `Left Hand Tracker` / `Right Hand Tracker` advertising service `4F7A0001-…`, subscribe to char
   `4F7A0002-…`, and watch 32-byte notifications arrive. Bytes 0–15 are the quaternion floats
   (little-endian). With both boards flashed, both names should appear.
3. **Verify both devices end-to-end with the PC dashboard** (`../pc_dashboard/`) — connects to both
   boards and shows two live 3D cubes + the numbers. Full test checklist in its README. Remember a
   board holds ONE central: disconnect nRF Connect / the dashboard before connecting anything else.
4. **Then connect the Vision Pro app.**

## Packet format (must match the app)
| Offset | Type     | Field                |
|--------|----------|----------------------|
| 0      | float32  | quaternion w         |
| 4      | float32  | quaternion x (i)     |
| 8      | float32  | quaternion y (j)     |
| 12     | float32  | quaternion z (k)     |
| 16     | float32  | accel x (m/s²)       |
| 20     | float32  | accel y              |
| 24     | float32  | accel z              |
| 28     | uint8    | calibration 0–3      |
| 29–31  | uint8[3] | padding              |

## Notes
- The **Rotation Vector** report is the magnetometer-fused, drift-corrected one. Calibrate by
  moving the sensor in a figure-8 a few times; watch the `calib` value climb toward 3.
- Both ESP32 and Apple silicon are little-endian, so floats go on the wire as-is (no byte swap).
- BLE only — this is why the project uses the C3, not the WiFi-only D1 Mini.

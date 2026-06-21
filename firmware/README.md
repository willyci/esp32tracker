# Firmware — ESP32-C3 + BNO085 (+ OLED + SoftPot)

Reads the BNO085's fused rotation vector + accelerometer over I2C and streams a 32-byte
packet over BLE notify (~50 Hz) to the visionOS app / PC dashboard. The full build adds a
status OLED and a SoftPot touch strip.

## Sketches

| Sketch | What it is |
|--------|-----------|
| `esp32_tracker/esp32_tracker.ino` | Base tracker — **BNO085 + BLE only**. Smallest, coolest, simplest. |
| `esp32_tracker_touch_display/esp32_tracker_touch_display.ino` | ⭐ **Full build** — base + status **OLED** + **SoftPot** touch strip (reports the slide start & current points). The recommended sketch. |

(`esp32_tracker_display/` and `esp32_tracker_test_display/` are earlier OLED iterations, superseded by
`esp32_tracker_touch_display`.)

All speak the **identical** BLE protocol (UUIDs + packet below), so the app and PC dashboard work with
any. Each lives in its own folder because the Arduino IDE concatenates every `.ino` in a folder into one
build — open each from its own folder.

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

## Full build — OLED + SoftPot (touch_display sketch)

The full sketch adds a **128x64 OLED** and a **SoftPot** linear touch strip. Both go on their *own*
pins — **not** the BNO's I2C bus.

### OLED — software I2C on GPIO5 / GPIO21

| OLED (GME12864 / SSD1306) | ESP32-C3 |
|------|------|
| VCC  | 3V3   |
| GND  | GND   |
| SDA  | **GPIO5**  |
| SCL  | **GPIO21** |

> ⚠️ **Why software I2C on its own pins, not the BNO's hardware bus?** We tried the OLED on the shared
> GPIO0/1 hardware bus — U8g2/U8x8's hardware-I2C would **not coexist** with the Adafruit BNO library
> (OLED stayed blank / `begin()` hung). Bit-banged **software** I2C on separate pins (GPIO5/21) never
> touches the BNO's hardware I2C peripheral, and just works. (The C3 has only one hardware I2C
> controller — an S3 with two would let both be hardware I2C; not needed here.)
>
> GPIO21 is free because `Serial` runs over USB-CDC, not the UART0 pins.

The screen shows `LEFT`/`RIGHT` + `ADV`/`CONN` + `C:n` (calib), quaternion w/x/y/z, and the SoftPot
`St`/`Cu` points. It **wakes for `SCREEN_ON_MS` (10 s) on power-up or a BOOT press, then auto-offs** —
keeping the slow software-I2C redraw from stuttering BLE the rest of the time. Sanity check: board flat
and still → `az ≈ +9.8` (gravity). If text is garbled/shifted, it's the 1.3" **SH1106** variant — swap
`SSD1306` → `SH1106` in the display constructor.

### SoftPot — analog on GPIO3 (+ 100k pulldown)

| SoftPot (Spectra, 3 solder tabs) | ESP32-C3 |
|------|------|
| Pin 1 (V+, end)     | 3V3 |
| Pin 2 (wiper, MIDDLE) | **GPIO3** |
| Pin 3 (GND, end)    | GND |
| **100 kΩ** | from **GPIO3 → GND** (pulldown) |

The **100k pulldown is required** — without it an untouched strip floats and reads noise. GPIO3 (not
GPIO2) because GPIO2 is a boot **strapping** pin, and the pulldown would hold it LOW at boot. The
firmware captures the **start** point on touch-down and tracks the **current** point as you slide
(reset on release) — the start↔current Δ is the slide/advance gesture.

### BOOT button (GPIO9)
Built into the board. A press wakes the OLED for 10 s. (Don't *hold* it during power-up — that's the
download-mode combo; a press while running is fine.)

### Full wiring diagram (touch_display)
```
   ESP32-C3 SuperMini
  ┌────────────────────┐
  │ 3V3 ●──┬─────────────────► BNO085 VIN
  │        ├─────────────────► OLED VCC
  │        └─────────────────► SoftPot Pin1 (V+)
  │ GND ●──┬─────────────────► BNO085 GND
  │        ├─────────────────► OLED GND
  │        └─────────────────► SoftPot Pin3 (GND)
  │GPIO0 ●───────────────────► BNO085 SDA  ┐ hardware I2C (0x4B)
  │GPIO1 ●───────────────────► BNO085 SCL  ┘
  │GPIO5 ●───────────────────► OLED SDA    ┐ software I2C (0x3C)
  │GPIO21●───────────────────► OLED SCL    ┘
  │GPIO3 ●──┬────────────────► SoftPot Pin2 (wiper)   ← ADC
  │         └──[100kΩ]──► GND  (pulldown on the wiper node)
  │GPIO9 ●  BOOT button (on-board) → wakes OLED
  │ [USB-C / 5V]                              battery → 5V pin (USB unplugged)
  └────────────────────┘
```
All three peripherals share the **3V3** rail and a common **GND**. The base sketch needs only the
BNO085 (the top four wires).

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
   the server-callback signatures changed). For the touch+display sketch, also install **U8g2** (by
   oliver — provides the U8x8 text mode used here).
3. Open `esp32_tracker/esp32_tracker.ino` (or `esp32_tracker_touch_display/esp32_tracker_touch_display.ino`),
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
| 29     | uint8    | SoftPot touchStart (0 = no touch)   |
| 30     | uint8    | SoftPot touchCurrent (0 = no touch) |
| 31     | uint8    | touchActive (1 = finger down)       |

(The base `esp32_tracker` sketch leaves bytes 29–31 as 0; consumers that don't use the SoftPot just
ignore them.)

## Notes
- The **Rotation Vector** report is the magnetometer-fused, drift-corrected one. Calibrate by
  moving the sensor in a figure-8 a few times; watch the `calib` value climb toward 3.
- Both ESP32 and Apple silicon are little-endian, so floats go on the wire as-is (no byte swap).
- BLE only — this is why the project uses the C3, not the WiFi-only D1 Mini.

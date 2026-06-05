# Firmware — ESP32-C3 + BNO085

Reads the BNO085's fused rotation vector + accelerometer over I2C and streams a 32-byte
packet over BLE notify (~50 Hz) to the visionOS app.

## Wiring (I2C)

| BNO085 | ESP32-C3 SuperMini |
|--------|--------------------|
| VIN    | 3V3                |
| GND    | GND                |
| SDA    | GPIO8              |
| SCL    | GPIO9              |

```
   BNO085 breakout                 ESP32-C3 SuperMini
  ┌───────────────┐               ┌───────────────────┐
  │           VIN ●───────────────● 3V3                │
  │           GND ●───────────────● GND                │
  │           SDA ●───────────────● GPIO8  (SDA)       │
  │           SCL ●───────────────● GPIO9  (SCL)       │
  │                               │                    │
  │  RST INT P0 P1  ← unused      │  [USB-C]  ← to Mac │
  └───────────────┘               └───────────────────┘
```

Only these four wires. Leave the BNO085's RST / INT / P0 / P1 / PS pins unconnected — the firmware
uses plain I2C polling. Power from **3V3**, not 5V (the C3's GPIOs are 3.3 V logic). Adafruit/SparkFun
breakouts already have I2C pull-ups, so none to add.

Confirm GPIO8/9 against your board's silkscreen; SuperMini clones vary. The pins are set in the sketch
(`PIN_SDA` / `PIN_SCL`), so change them there if your board differs.

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
1. Install ESP32 board support (Boards Manager → "esp32" by Espressif).
2. Library Manager → install **Adafruit BNO08x** and **NimBLE-Arduino** (use 1.4.x).
3. Open `esp32_tracker/esp32_tracker.ino`, select board **ESP32C3 Dev Module**, upload.

## Bring-up order (matches SPEC.md milestones)
1. **Serial first.** After flashing, the monitor should print `BNO08x ready` then
   `BLE advertising as ESP32-Tracker`. If you want to eyeball the quaternion, temporarily add a
   `Serial.printf` of `pkt.w/x/y/z` in `loop()`.
2. **Verify BLE with a scanner.** Use **nRF Connect** (iOS/Android) — confirm a device named
   `ESP32-Tracker` advertising service `4F7A0001-…`, subscribe to char `4F7A0002-…`, and watch
   32-byte notifications arrive. Bytes 0–15 are the quaternion floats (little-endian).
3. **Then connect the Vision Pro app.**

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

# ESP32 Orientation Tracker → Apple Vision Pro — Prototype Spec (v0)

## Goal

A minimal end-to-end prototype:

1. Connect the Vision Pro app to an ESP32-C3 + IMU over Bluetooth LE.
2. Stream the IMU's fused orientation (quaternion) + raw values.
3. Display the raw data as text in the scene.
4. Rotate a 3D object in a RealityKit scene to match the sensor's real-world orientation.

This is a "see it work" milestone. **Position tracking is explicitly out of scope** — the IMU
gives orientation only (see why in the project notes). The object sits at a fixed point and
only rotates.

---

## System architecture

```
┌──────────────────────┐      BLE GATT notify      ┌───────────────────────────┐
│  ESP32-C3 SuperMini  │  ───────────────────────► │   Apple Vision Pro app    │
│                      │   quaternion + raw data   │   (SwiftUI + RealityKit)  │
│  ┌────────────────┐  │      (~50 Hz)             │                           │
│  │ BNO08x / BNO055│  │                           │  CoreBluetooth ─► state   │
│  │  (I2C, fusion) │  │                           │  state ─► RealityView     │
│  └────────────────┘  │                           │  state ─► text overlay    │
└──────────────────────┘                           └───────────────────────────┘
```

- **Why BLE:** Vision Pro connects to peripherals via Core Bluetooth. ESP32-C3 has BLE.
  (The Wemos D1 Mini / ESP8266 is WiFi-only and cannot be used here.)
- **Why send a quaternion, not raw accel/gyro:** the BNO does sensor fusion onboard and emits a
  drift-corrected quaternion. The app stays dumb — no fusion math on visionOS.

---

## Hardware

| Item | Choice | Notes |
|------|--------|-------|
| MCU | ESP32-C3 SuperMini | Has BLE (required). |
| IMU | BNO085 (preferred) **or** BNO055 / DFRobot SEN0253 | Both output a fused quaternion. |
| Bus | I2C (SDA/SCL + 3V3 + GND) | BNO055 uses clock stretching → keep it on the C3's HW I2C, not ESP8266. |

I2C pins used: **SDA = GPIO0, SCL = GPIO1** (confirm against your board silk). GPIO8/9 are avoided —
GPIO8 is the SuperMini's onboard LED and GPIO8/9 are boot strapping pins.

> **Two-device update (current):** the prototype now runs **two** trackers — left and right hand —
> each rendered as its own 3D object. Both boards run the same firmware; `#define IS_LEFT_HAND` sets a
> board's hand and its advertised BLE name (`Left Hand Tracker` / `Right Hand Tracker`). Both share the
> same service/characteristic UUIDs; the app distinguishes them by name. The sections below describe the
> single-device v0; the multi-device wiring is the same per board, and the app changes are reflected in
> the file layout.

---

## BLE protocol (custom GATT service)

Keep it dead simple for v0: one service, one notify characteristic carrying a packed binary frame.

| | UUID (placeholder — generate your own) |
|---|---|
| Service | `4f7a0001-9b3e-4c2a-8d1f-0a1b2c3d4e5f` |
| Orientation characteristic (Notify) | `4f7a0002-9b3e-4c2a-8d1f-0a1b2c3d4e5f` |

**Packet format (little-endian, 32 bytes):**

| Offset | Type | Field |
|--------|------|-------|
| 0  | float32 | quaternion w |
| 4  | float32 | quaternion x |
| 8  | float32 | quaternion y |
| 12 | float32 | quaternion z |
| 16 | float32 | raw accel x (m/s²) — optional, for the text display |
| 20 | float32 | raw accel y |
| 24 | float32 | raw accel z |
| 28 | uint8   | calibration status (0–3) |
| 29 | uint8[3]| padding |

Notify at ~50 Hz. (float32 is wasteful but trivial to parse; optimize later if needed.)

---

## Firmware spec (ESP32-C3, Arduino)

**Libraries:** `Adafruit_BNO08x` (or `Adafruit_BNO055`), ESP32 BLE (`NimBLE-Arduino` preferred —
lighter than the stock BLE stack).

**Responsibilities:**
1. Init I2C + IMU. Enable the **Rotation Vector** report (BNO08x) / read quaternion (BNO055).
2. Start a BLE server, advertise the service above.
3. On each IMU sample: pack the 32-byte frame, `notify()` the orientation characteristic.
4. Print quaternion to Serial for bring-up debugging.

**Build order:** get quaternion-over-Serial working first, *then* wrap it in BLE.

---

## visionOS app spec

### Project setup
- New visionOS app, SwiftUI lifecycle.
- `Info.plist`: add **`NSBluetoothAlwaysUsageDescription`** ("Connect to the orientation sensor").
- Min target: visionOS 1.0+.

### File layout
```
ESP32Tracker/
├── ESP32TrackerApp.swift          // @main, WindowGroup
├── ContentView.swift              // layout: RealityView + two data panels + Scan/Re-center
├── BLEManager.swift               // CoreBluetooth, ObservableObject; connects to BOTH boards,
│                                  //   routes each peripheral to its hand by advertised name
├── TrackerState.swift             // per-hand model: Hand/ConnectionState enums, quaternion, accel,
│                                  //   calib, packet parsing, and the orientation-shaping math
└── OrientationScene.swift         // RealityView setup + per-frame rotation of the two entities
```

> Note: the orientation-shaping logic (`basis`, `mountOffset`, `reference`, `displayOrientation`,
> `recenter`) lives in **TrackerState.swift** — one instance per hand — not in BLEManager.

### `BLEManager` (CoreBluetooth)
- `NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate`.
- `@Published var state: SensorState` (orientation quaternion, accel, calibration, connection).
- Flow: `scanForPeripherals(withServices: [serviceUUID])` → connect → discover service →
  discover orientation characteristic → `setNotifyValue(true)`.
- In `didUpdateValueFor`: parse the 32-byte `Data` into floats, publish to `state`.
  ```swift
  let q = simd_quatf(ix: x, iy: y, iz: z, r: w)   // note: (x,y,z,w) order
  ```

### Coordinate-frame mapping (the one tricky bit)
The IMU's axes (X forward / Y left / Z up, right-handed) ≠ RealityKit's frame (Y up, Z toward
viewer, right-handed). The raw quaternion will look "rotated wrong." Fix with a constant
remap quaternion applied to the sensor quaternion:

```swift
entity.orientation = frameRemap * sensorQuaternion
```

Start with `frameRemap = identity`, observe how a real rotation maps on-screen, then derive the
±90° axis swaps empirically. **Budget time for this — it's where prototypes always stall.**

### SwiftUI views
- **`ContentView`**: VStack/HStack with
  - the `RealityView` (the rotating object),
  - a data panel: connection state, quaternion (w,x,y,z to 3 dp), accel, calibration status,
  - a "Scan / Connect" button and a "Re-center" button (snaps current orientation to identity).
- **`OrientationScene`**: `RealityView { content in ... }` adds a visible entity (start with a
  `ModelEntity(mesh: .generateBox(...))` with distinct face colors so rotation is readable; swap
  for a probe model later). `update:` closure reads `state.orientation` and sets
  `entity.orientation` each frame.

### Re-center feature
Store `referenceQuaternion = currentSensorQuaternion.inverse` on button tap; display
`reference * sensor` so "current pose" becomes the new zero. Makes the demo legible without
physically aligning the sensor to the room.

---

## Build milestones

1. **Firmware: quaternion over Serial.** IMU readings print to the serial monitor.
2. **Firmware: BLE notify.** Confirm the packet with a generic BLE scanner app (e.g. nRF Connect).
3. **App: connect + raw text.** Vision Pro connects, displays live quaternion/accel numbers.
4. **App: rotating box.** Box rotates with the sensor (frame mapping may be wrong — fine).
5. **App: fix frame mapping + re-center.** Box matches real-world orientation intuitively.
6. *(later)* Swap box for a probe model; revisit position tracking as a separate effort.

---

## Open questions / decisions to confirm

- **Which IMU did you buy** — BNO085, or the SEN0253 (BNO055)? Picks the firmware library.
- **Box or probe model for v0?** Recommend a colored box first (rotation is easier to read).
- **Re-center button in v0, or skip for first light?** Recommend include — cheap and makes the demo work.

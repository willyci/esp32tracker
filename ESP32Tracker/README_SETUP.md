# visionOS app — setup

These are the source files for the prototype. No `.xcodeproj` is committed (they're messy to
hand-author and noisy in git). Generate one with **XcodeGen** — config is in `../project.yml`.

## 1. Generate the Xcode project (recommended)
```
brew install xcodegen      # once
cd <repo root>             # where project.yml lives
xcodegen generate
open ESP32Tracker.xcodeproj
```
This wires up all five Swift files, the visionOS target, and the Bluetooth permission string
automatically — skip straight to **Run** below. To build on a real device, set your
`DEVELOPMENT_TEAM` in `project.yml` first (or pick a team in Xcode's Signing tab).

## 1b. Or create the project by hand
1. Xcode → **File ▸ New ▸ Project… ▸ visionOS ▸ App**. Name **ESP32Tracker**, SwiftUI, Swift.
   Immersive Space: **None** (we use a windowed RealityView for v0).
2. Delete the auto-generated `ESP32TrackerApp.swift` / `ContentView.swift`; drag in all five
   `.swift` files from this folder — `ESP32TrackerApp`, `ContentView`, `BLEManager`, `TrackerState`,
   `OrientationScene` ("Copy items if needed" ✓, add to the app target).
3. Add the Bluetooth permission: Target ▸ **Info** ▸ **Privacy - Bluetooth Always Usage
   Description** (`NSBluetoothAlwaysUsageDescription`) → e.g. *"Connect to the orientation sensor."*
   (The XcodeGen path sets this for you.)

## 2. Run
- Bluetooth/peripheral hardware doesn't work in the **Simulator** — run on a **real Vision Pro**
  (or use BLE peripherals the device can see). The UI/3D objects render in the Simulator, but the
  panels sit at "Scanning…" forever there.
- Tap **Scan** → it discovers **both** boards by their advertised name and connects to each:
  `Left Hand Tracker` → left object, `Right Hand Tracker` → right object. (Flash both boards first;
  with only one powered, the other panel stays "Scanning…".)

## What you should see
- The 3D viewport: **two** objects side by side — a **teal** box (left hand) and a **purple** box
  (right hand), each with an orange nose, rotating as you move its sensor.
- The data panels: one per hand, each with live quaternion, accel, calibration, and connection state.
- The objects will likely rotate on the "wrong" axes at first — that's the coordinate-frame mapping.
  Adjust `basis` / `mountOffset` in `TrackerState.swift` (start from identity) per `../FRAME_MAPPING.md`,
  and use the **Re-center** buttons (per hand, or "Re-center both") to zero out the current pose.

## Matching the firmware
`BLEManager.serviceUUID` / `orientationCharUUID` and the 32-byte packet layout (parsed in
`TrackerState.ingest`) must match the ESP32 sketch exactly. The hand names in the `Hand` enum
(`TrackerState.swift`) must match each board's `DEVICE_NAME`. See `../SPEC.md` for the canonical
definitions.

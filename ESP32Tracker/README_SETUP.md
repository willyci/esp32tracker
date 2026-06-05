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
This wires up all four Swift files, the visionOS target, and the Bluetooth permission string
automatically — skip straight to **Run** below. To build on a real device, set your
`DEVELOPMENT_TEAM` in `project.yml` first (or pick a team in Xcode's Signing tab).

## 1b. Or create the project by hand
1. Xcode → **File ▸ New ▸ Project… ▸ visionOS ▸ App**. Name **ESP32Tracker**, SwiftUI, Swift.
   Immersive Space: **None** (we use a windowed RealityView for v0).
2. Delete the auto-generated `ESP32TrackerApp.swift` / `ContentView.swift`; drag in all four
   `.swift` files from this folder ("Copy items if needed" ✓, add to the app target).
3. Add the Bluetooth permission: Target ▸ **Info** ▸ **Privacy - Bluetooth Always Usage
   Description** (`NSBluetoothAlwaysUsageDescription`) → e.g. *"Connect to the orientation sensor."*
   (The XcodeGen path sets this for you.)

## 2. Run
- Bluetooth/peripheral hardware doesn't work in the **Simulator** — run on a **real Vision Pro**
  (or use a BLE peripheral the device can see). The UI/3D box will render in the Simulator, but
  it will sit at "Scanning…" forever there.
- Tap **Scan** → it auto-connects to the first board advertising the service UUID.

## What you should see
- Left: a teal box with an orange nose, rotating as you move the sensor.
- Right: live quaternion, accel, calibration, and connection state.
- The box will likely rotate on the "wrong" axes at first — that's the coordinate-frame mapping.
  Adjust `frameRemap` in `BLEManager.swift` (start from identity) until it feels right, and use
  **Re-center** to zero out the current pose.

## Matching the firmware
`BLEManager.serviceUUID` / `orientationCharUUID` and the 32-byte packet layout must match the
ESP32 sketch exactly. See `../SPEC.md` for the canonical definitions.

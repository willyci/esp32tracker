# esp32tracker

A prototype that streams hardware IMU orientation to an **Apple Vision Pro** app and rotates 3D
objects in real time. Two trackers — **left** and **right hand** — each drive their own object.

```
┌──────────────────────┐    BLE notify ~50 Hz    ┌───────────────────────────┐
│ ESP32-C3 + BNO085 ×2 │ ──────────────────────► │   visionOS app            │
│ Left + Right hand    │   32-byte packet each   │   SwiftUI + RealityKit    │
│ (fused quaternion)   │                         │   two objects             │
└──────────────────────┘                         └───────────────────────────┘
```

Each IMU does the sensor fusion onboard and sends a drift-corrected quaternion; the app stays simple
and just renders it. The two boards share the same BLE service/characteristic UUIDs and are told apart
by their advertised name (`Left Hand Tracker` / `Right Hand Tracker`).

> **Orientation only** — position tracking is out of scope. An IMU alone cannot recover position
> (double-integrating acceleration drifts to meters within seconds), so these trackers report how each
> hand is *rotated*, not *where it is*. For hand position on Vision Pro, use ARKit hand tracking. See
> [SPEC.md](SPEC.md).

## Repository layout

| Path | What it is |
|------|-----------|
| [`SPEC.md`](SPEC.md) | Architecture, BLE protocol, packet format, build milestones |
| [`FRAME_MAPPING.md`](FRAME_MAPPING.md) | How sensor axes map to RealityKit, and how to calibrate it |
| [`project.yml`](project.yml) | XcodeGen config — generates the visionOS Xcode project |
| [`ESP32Tracker/`](ESP32Tracker/) | The visionOS app (SwiftUI + RealityKit + Core Bluetooth) |
| [`firmware/`](firmware/) | The ESP32-C3 Arduino sketch + PlatformIO config |

## Hardware

Two identical trackers (one per hand), each:

- **ESP32-C3 SuperMini** — chosen because it has **BLE** (the WiFi-only D1 Mini can't talk to Vision Pro).
- **BNO085** IMU over I2C (SDA = GPIO0, SCL = GPIO1) — fused, drift-corrected orientation. (A
  BNO055 / DFRobot SEN0253 also works with a library swap.)

The same sketch flashes both; one `#define IS_LEFT_HAND` line picks the hand (and the BLE name). Wiring
and the two-board flashing steps are in [`firmware/README.md`](firmware/README.md).

## Quick start

### 1. Firmware
```
cd firmware
pio run -t upload      # then: pio device monitor
```
Confirm the serial log shows `BNO08x ready` and `BLE advertising as Left Hand Tracker` (or
`Right Hand Tracker`). Flash both boards — set `IS_LEFT_HAND` per board. Verify the BLE data with a
phone scanner (nRF Connect) **before** touching the app — see [`firmware/README.md`](firmware/README.md).

### 2. visionOS app
```
brew install xcodegen   # once
xcodegen generate
open ESP32Tracker.xcodeproj
```
Set your signing team, build to a **real Vision Pro** (Bluetooth doesn't work in the Simulator), tap
**Scan**. Details and the manual-Xcode alternative: [`ESP32Tracker/README_SETUP.md`](ESP32Tracker/README_SETUP.md).

### 3. Make it line up
The objects will rotate on the "wrong" axes until you calibrate the coordinate-frame mapping. Follow
the 10-minute procedure in [`FRAME_MAPPING.md`](FRAME_MAPPING.md), then use the app's **Re-center**
buttons (per hand, or "Re-center both").

## Status

Prototype / v0. Working toward the milestones in [`SPEC.md`](SPEC.md):
connect → display raw data → rotate an object. Position tracking and a real probe model are future work.

## Notes

- `project.yml` is the source of truth for the Xcode project; the generated `.xcodeproj` is a build
  artifact and is git-ignored. Edit the yml and re-run `xcodegen generate` rather than changing
  project settings in Xcode's UI.
- BLE UUIDs and the 32-byte packet layout are defined once in [`SPEC.md`](SPEC.md) and must match in
  both `firmware/esp32_tracker/esp32_tracker.ino` and `ESP32Tracker/TrackerState.swift` (packet
  parsing) / `ESP32Tracker/BLEManager.swift` (UUIDs).

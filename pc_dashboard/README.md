# PC dashboard — live view of both hand trackers

A small Windows/Mac/Linux tool to verify the BLE pipeline without the Vision Pro: it connects
to **both hand trackers and both foot pedals** over Bluetooth LE and shows two live 3D dice
cubes (teal = left, purple = right), the numeric data, pedal status, and a
**catheter/guidewire simulation panel** (same manipulation model as the visionOS app), in
your browser. Speaks the exact same GATT protocol as the visionOS app (same UUIDs, same
32-byte packet, same name-based device matching).

## Run

```
pip install bleak aiohttp        # once
python tracker_dashboard.py
```

The browser opens `http://localhost:8765` automatically. Power on one or both trackers —
each card flips from *Scanning…* to *Connected* as its board is found.

### macOS notes

Works unchanged (`bleak` uses CoreBluetooth there; use `pip3` / `python3`). Two gotchas:

- **Bluetooth permission:** the first scan pops a macOS privacy prompt for your terminal app.
  If denied, scans silently find nothing — re-enable under **System Settings → Privacy &
  Security → Bluetooth**.
- **Quit the dashboard before testing the Vision Pro app.** A board accepts only one central;
  while the dashboard is connected, the board stops advertising and the headset can't find it.
  (Conversely, the dashboard is a good reference: if it sees both boards and the visionOS app
  doesn't, the bug is in the Swift side.)

## How to test

Do these in order — each step verifies one layer of the pipeline.

### 1. Connection (both boards found)
Power both boards (USB or battery). Within ~10 s both cards should show a green
**Connected** badge. The console also logs `[left] connected` / `[right] connected`.

- Only one connects? The other board isn't advertising — check its serial monitor for
  `BLE advertising as …`, and make sure nothing else (nRF Connect, a phone) is already
  connected to it (a board accepts only ONE central at a time).

### 2. Data flow (numbers are live)
Pick up either sensor and move it. Its quaternion values (w/x/y/z) and accel values should
change continuously. A still sensor shows a steady quaternion and accel ≈ gravity (one axis
near ±9.8 m/s²).

### 3. Left/right routing (the key two-device test)
Rotate ONLY the left sensor → only the **teal** cube moves. Rotate ONLY the right sensor →
only the **purple** cube moves. This proves the name-based matching that the visionOS app
will rely on.

### 4. Calibration
Each card shows **Calib n/3**, color-coded red (0) → green (3). If a sensor reads 0–1, wave
it in a few slow **figure-8** motions until it climbs to 2–3. Heading (yaw) accuracy is poor
until calibrated; near large metal objects it may refuse to reach 3 — move away and retry.

### 5. Re-center
Hold a sensor in any pose and click its **Re-center** button — the cube should snap to the
neutral (face 1 toward you) pose, and rotations are now relative to that. **Re-center both**
does the same for the two at once.

### 6. Rotation axes (frame mapping)
With the sensor flat and re-centered, test one axis at a time:

| You do | Cube should |
|---|---|
| Tilt the front edge up (pitch) | tip its top face toward you |
| Tilt sideways (roll) | roll the same direction |
| Turn clockwise on the table (yaw) | turn the same direction |

If a motion appears on the wrong axis or mirrored, that's the coordinate-frame mapping —
not a hardware fault. The mapping constants (`BASIS`, `MOUNT_OFFSET` in `index.html`) are
ports of `ESP32Tracker/TrackerState.swift`; tune per `../FRAME_MAPPING.md` and keep the
two files in sync. This lets you nail the mapping on the PC before ever building the
visionOS app.

### 7. Drop / reconnect
Unplug one board → its badge returns to *Scanning…* within a few seconds, the other keeps
streaming. Plug it back in → it reconnects automatically (~5–10 s). Ctrl+C the server and
restart it → the page reconnects on its own.

### 8. Catheter/guidewire simulation (glove interaction, no headset)
The bottom panel is a 1:1 port of the visionOS app's `SimulationModel.swift` — use it to
tune and verify the glove interaction before a headset session:

- **Grab**: touch a SoftPot → that tool's badge flips to **GRABBED** and its rod glows.
  Left tracker drives the teal **catheter**, right drives the purple **guidewire**.
- **Twist**: while grabbed, slide along the strip (full strip = 1 turn) or physically
  roll the tracker — both accumulate, shown as stripe movement on the rod, the roll dial,
  and a turns counter. Releasing freezes the twist; re-grabbing never jumps.
- **Insert**: the headset uses ARKit hand tracking for this, which a PC doesn't have, so
  ←/→ arrow keys or the mouse wheel over the scene stand in — they advance/retract any
  grabbed tool with the same VascCath scale factors and depth limits (58 cm / 61 cm).
- **X-ray**: either board's button renders the catheter translucent so the wire inside shows.

If twist runs the wrong way or is too sensitive, tune `SIM.twistAxis` / `SIM.stripFullTurns`
in `index.html` — then apply the same values to `SimulationModel.swift` (they must match).

### 9. Foot pedals
Power on the ESP32-S3 SuperMini pedals (`../firmware/left-foot/`, `../firmware/right-foot/`) —
their badges under the X-ray banner flip to green **Connected** just like the hand cards.

- **Left pedal** (X-ray): each stomp toggles the shared X-RAY banner — same effect as the
  hand trackers' GPIO6/7 button, and it also drives the catheter transparency in the
  simulation panel.
- **Right pedal** (capture): each stomp increments the **captures** counter and fires a
  brief white full-screen flash (fluoro-shot style). The dashboard only counts and flashes —
  what a "capture" saves is up to each consumer (the visionOS app will define its own action).

All devices are independent BLE connections; any subset may be on at a time.

### 10. Mini trackers
The ESP32-C3 0.42"-OLED glove units (`../firmware/left-mini/`, `../firmware/right-mini/` —
SoftPot + X-ray button + capture button, no IMU) are **drop-in alternatives for the hand
slots**: "Left Mini Tracker" fills the same card as "Left Hand Tracker" (first one found
wins the slot). The cube stays frozen (identity quaternion — orientation comes from the
headset), but the SoftPot track, the simulation panel's grab/twist, the shared X-ray
toggle, and the capture counter/flash all work exactly as with the big trackers.

### 8. Latency feel
Twist a sensor sharply — the cube should respond with no perceptible lag (the link runs at
50 Hz; the page renders at 30 Hz). Stutter or sluggishness usually means weak signal
(distance/walls) or another app hammering the Bluetooth adapter.

## Troubleshooting

- **Stuck at "Scanning…"** — check the boards are powered and advertising (serial monitor
  shows `BLE advertising as Left/Right Hand Tracker`), and that the PC's Bluetooth is on.
  Note a board can hold only ONE central: if nRF Connect (or a previous dashboard run) is
  still connected to it, it stops advertising — disconnect there first.
- **`Scan failed`** in the console — Bluetooth adapter off/missing, or another app holds it.
- **Cube rotates on the "wrong" axes** — see step 6 above; it's the frame mapping, not the
  hardware.
- **Port 8765 in use** — change `PORT` in `tracker_dashboard.py`.

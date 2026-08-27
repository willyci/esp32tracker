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

### 9. Foot pedals (broadcast, never connected)
Power on the ESP32-S3 SuperMini pedals (`../firmware/left-foot/`, `../firmware/right-foot/`) —
their badges under the X-ray banner turn green **Detected** within a second or two.

Pedals are **connectionless**: they advertise `[0xFF,0xFF, pressCount, held]` in manufacturer
data and the dashboard reads it straight from a continuous scan. (Vision Pro runs out of BLE
connection slots with two hands already connected — `CBError 11` — so pedals must never take
one. The PC dashboard follows the same design so both consumers behave identically.)
"Detected" therefore means *we can hear it advertising*, not that a link is open.

- **Left pedal** (X-ray): **hold-to-activate**, like a real fluoro pedal — X-RAY is on while
  your foot is down and off the instant you lift it (badge reads *HELD — X-RAY ON*). It is
  NOT a toggle. It also drives the catheter transparency in the simulation panel. The hand
  buttons still *latch* X-ray independently; effective state is `latched OR pedal-held`, so
  releasing the pedal returns to whatever the buttons had set.
- **Right pedal** (capture): each stomp increments the **captures** counter and fires a
  brief white full-screen flash (fluoro-shot style). The dashboard only counts and flashes —
  what a "capture" saves is up to each consumer (the visionOS app defines its own action).

**Fail-safe:** because the left pedal is a *level*, silence must not leave X-ray stuck on. Two
windows handle it (`LEVEL_RELEASE_AFTER` / `PEDAL_OFFLINE_AFTER` in `tracker_dashboard.py`):
after **4 s** of silence a held level is released (battery died, out of range, board crashed),
and after **5 s** the pedal is marked offline in the UI.

Those look generous next to a pedal that advertises every ~100–150 ms, and they are — on
purpose. **Sending fast is not the same as being heard fast:** measured on Windows, a pedal
emitting ~7–10 ads/s was delivered to the dashboard at only **~1–3/s, with normal gaps up to
~3 s**, because the host OS aggregates repeat advertisements from the same device. (WinRT's
`SignalStrengthFilter` sampling interval was tried and measured to make delivery *worse*.) A
tighter window just produces false "lost pedal" events and, worse, releases the pedal
mid-press. Size these from measurement, not from the firmware interval — and note the
consequence: a dead pedal takes a few seconds to clear. That is the right trade for a
training simulator, where the alternative is X-ray flickering off during fluoro.

Any subset of devices may be on at a time; the two hands use connections, the pedals don't.

### 9b. DSA pedal (pedal 3 — contrast run)
`../firmware/dsa-foot/` — the DSA (digital subtraction angiography) run: the images taken once
the catheter is in position and X-ray dye is being injected, so the vessels fill.

- **Hold = run.** While the pedal is down the **DSA** badge reads *RUN — CONTRAST* in amber,
  the **runs** counter increments once per run, and the simulation panel's vessel **fills
  dark** — injected iodine reads dark on a subtracted angiogram, so it fills in rather than
  lighting up. Release ends the run and the vessel clears.
- **A DSA run implies X-ray.** A run *is* an X-ray acquisition, so the X-RAY banner comes on
  for its duration (effective state is `latched OR fluoro-pedal-held OR dsa-active`). It does
  not disturb the hand buttons' latch — ending a run returns to whatever they had set.
- Works with a momentary pedal or a maintained (latching) toggle: the broadcast is a level
  that mirrors the switch position either way.

**3-wire wiring (COM/NO/NC) — and why it's better than the other two pedals:**

| Switch terminal | Pin | Reads |
|---|---|---|
| COM | GPIO12 | driven LOW — the switch's common ground |
| NO  | GPIO10 | LOW when **pressed** |
| NC  | GPIO8  | LOW when **at rest** |

Because both contacts are read and they are complementary, the firmware can tell pressed from
released *from a broken connection*: both open = unplugged / broken wire / switch failed open;
both closed = miswired or NO-NC short. Either fault reports "not running" (never start a run on
a suspect switch) and shows **DSA: SWITCH FAULT** in red on the dashboard and in the headset.
A plain 2-wire button cannot distinguish "not pressed" from "not connected" at all.

Same fail-safe as the fluoro pedal (4 s of silence): if a pedal dies mid-run, the run ends and
X-ray goes off rather than latching on forever.

### 9c. Touchscreen pedal panel
`../firmware/pedal-panel/` — a Waveshare ESP32-S3-Touch-AMOLED-2.06 that replaces all three
foot pedals with one screen: three full-width buttons (X-RAY / DSA / CAPTURE) with LRA haptics
and a speaker click.

It broadcasts under the name **`Pedal Panel`**, using the same 5-byte manufacturer data as the
pedals but with the level byte as a **bitfield**: `bit0` = X-ray held, `bit1` = DSA run, and the
count byte = captures. X-RAY and DSA are **hold** controls (finger down = active), matching the
real pedals' dead-man behaviour; CAPTURE is a tap. The buttons show accumulated fluoro and run
time (`X-RAY 12s`, `DSA 3s`) and the capture count.

**The panel and the foot pedals are independent sources of the same two signals**, so the
dashboard ORs them (`xray_held()` / `dsa_running()`). You can run both at once and neither
cancels the other — an earlier version shared one variable, and an idle panel's advertisement
would silently release a genuinely held foot pedal.

Because it is broadcast-only it never takes an AVP connection slot, but it also **cannot know
whether anyone is listening**: its status screen reports *advertising*, not *connected*, and it
switches to the buttons on a 15 s timer.

### 9d. Panel modes (only one half runs at a time)
The panel does two jobs that are never needed together, and each is expensive, so it runs
one at a time:

- **PEDAL mode** (default) — the screen and its touch buttons drive X-ray / DSA / capture.
  IMU integration and the SoftPot are skipped, so the cube **holds its last orientation**
  rather than snapping to identity. Tap **IMU ▶** (top-right of the button screen) to switch.
- **TRACKER mode** — orientation and SoftPot at full rate, LVGL not running and no frames
  pushed, which reclaims ~10 ms per frame of QSPI. The buttons are unavailable, so
  X-ray/DSA/capture hold their last values. **Tap anywhere to wake** — the touch controller
  is still polled at 10 Hz for exactly that.

  What happens to the glass is a compile-time choice (`TRACKER_SCREEN_OFF` in the sketch),
  and the CPU/QSPI saving is identical either way:

  | | Screen | Power | Why |
  |---|---|---|---|
  | `0` **freeze** (default) | last frame stays visible | panel current only | the CO5300 has its own GRAM and self-refreshes from it, so an image persists with zero host activity — you can still see which mode the board is in |
  | `1` power down | blank | lowest | `displayOff()`; darkest, but gives no clue why |

  When freezing, `TRACKER_DIM_BRIGHTNESS` can dim the panel to trade a little readability
  for current. It's a small win: pixel current dominates on an AMOLED and the frozen notice
  is mostly black already.

Waking always lands on the **status** screen, never straight on the pedals: the tap that
woke it is probably still under your finger, and landing on the button screen would fire
X-ray the moment LVGL resumed.

Either way the same 32-byte packet still goes out at 50 Hz — only which fields move changes.

### 10. Mini trackers
The ESP32-C3 0.42"-OLED glove units (`../firmware/left-mini/`, `../firmware/right-mini/` —
MPU-6050 + SoftPot + X-ray button + capture button) are **drop-in alternatives for the hand
slots**: "Left Mini Tracker" fills the same card as "Left Hand Tracker" (first one found
wins the slot). The cube rotates from the MPU-6050's gyro — expect slow drift, since a
6-DOF IMU has no absolute reference; re-center as needed. The SoftPot track, the
simulation panel's grab/twist, the shared X-ray
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

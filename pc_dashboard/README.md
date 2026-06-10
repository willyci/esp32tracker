# PC dashboard — live view of both hand trackers

A small Windows/Mac/Linux tool to verify the BLE pipeline without the Vision Pro: it connects
to **both** boards over Bluetooth LE and shows two live 3D dice cubes (teal = left, purple =
right) plus the numeric data, in your browser. Speaks the exact same GATT protocol as the
visionOS app (same UUIDs, same 32-byte packet, same name-based left/right matching).

## Run

```
pip install bleak aiohttp
python tracker_dashboard.py
```

The browser opens `http://localhost:8765` automatically. Power on one or both trackers —
each card flips from *Scanning…* to *Connected* as its board is found. Rotate a sensor and
its cube follows. **Re-center** zeroes the current pose (same math as the app).

## Troubleshooting

- **Stuck at "Scanning…"** — check the boards are powered and advertising (serial monitor
  shows `BLE advertising as Left/Right Hand Tracker`), and that the PC's Bluetooth is on.
  Note a board can hold only ONE central: if nRF Connect (or a previous dashboard run) is
  still connected to it, it stops advertising — disconnect there first.
- **`Scan failed`** in the console — Bluetooth adapter off/missing, or another app holds it.
- **Cube rotates on the "wrong" axes** — expected until the frame mapping is calibrated;
  the mapping constants (`BASIS`, `MOUNT_OFFSET` in `index.html`) are ports of
  `ESP32Tracker/TrackerState.swift` and should be kept in sync with it. See
  `../FRAME_MAPPING.md`.
- **Port 8765 in use** — change `PORT` in `tracker_dashboard.py`.

# PC BLE dashboard for the two hand trackers + two foot pedals.
#
# CONNECTS to the ESP32-C3 hand trackers ("Left/Right Hand Tracker" or the OLED "Mini"
# variants) and parses the same 32-byte packets the visionOS app consumes. The two foot
# pedals are NOT connected — they are CONNECTIONLESS BROADCASTERS (Vision Pro runs out of
# BLE connection slots, see firmware/left-foot), so their state is read straight out of
# advertising manufacturer data during a continuous scan. Everything is pushed over a
# WebSocket to index.html (two live 3D cubes, simulation panel, pedal status).
#
#   LEFT pedal  = hold-to-activate X-ray (dead-man switch: on only while held)
#   RIGHT pedal = one X-ray screen capture per press
#
#   pip install bleak aiohttp
#   python tracker_dashboard.py        → opens http://localhost:8765
#
# Protocol (must match ../firmware/esp32_tracker/esp32_tracker.ino — see ../SPEC.md):
#   service 4F7A0001-..., notify char 4F7A0002-...
#   packet: float32 w,x,y,z (quaternion), float32 ax,ay,az (m/s^2), uint8 calib, 3 pad

import asyncio
import json
import socket
import struct
import time
import webbrowser
from pathlib import Path

from aiohttp import web, WSMsgType
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "4f7a0001-9b3e-4c2a-8d1f-0a1b2c3d4e5f"
CHAR_UUID    = "4f7a0002-9b3e-4c2a-8d1f-0a1b2c3d4e5f"

# Advertised names → dashboard slot. Names are set by IS_LEFT_HAND / IS_LEFT_FOOT in the
# firmware and arrive in the BLE scan response (they don't fit the main advertising packet
# next to the 128-bit UUID). All boards speak the same 32-byte packet.
#
# "Mini Tracker" = the ESP32-C3 0.42"-OLED glove unit (MPU-6050 + SoftPot + 2 buttons). It is
# a drop-in alternative for a hand slot, and its second button reports X-ray captures by
# flipping byte 28 (the calib slot — a Mini has no BNO-style calibration status to put there).
DEVICE_ALIASES = {   # CONNECTED devices (hand slots) — these run a GATT server
    "left":       ("Left Hand Tracker",  "Left Mini Tracker",  "Left Panel Tracker"),
    "right":      ("Right Hand Tracker", "Right Mini Tracker", "Right Panel Tracker"),
}
# BROADCAST-ONLY devices (no connection, no GATT): state comes from manufacturer data
# [0xFF, 0xFF, count, level(, flags)] in their advertisements. See firmware/left-foot.
PEDAL_ALIASES = {
    "left-foot":  ("Left Foot Pedal",),    # level = X-ray on while the foot is down
    "right-foot": ("Right Foot Pedal",),   # count change = one X-ray screen capture
    "dsa-foot":   ("DSA Foot Pedal",),     # level = DSA contrast run in progress
    # The touchscreen panel replaces all three pedals with one device, so its `level` byte
    # is a BITFIELD rather than a single flag. Same 5-byte manufacturer-data shape.
}
# The touchscreen panel is a CONNECTED hand-slot tracker (it replaces a hand rather than
# adding a device, so the connection budget is unchanged and it can stream at 50 Hz — a
# broadcaster is only delivered at ~1-3 ads/s, which would step its cube along). It fills a
# hand slot AND carries the three pedal controls inside the same 32-byte packet:
#   byte 28 (calib)  = capture COUNT
#   byte 31 (xrayOn) = LEVEL BITFIELD, bit0 X-ray held, bit1 DSA run (not a flip)
PANEL_LEVEL_XRAY = 0x01
PANEL_LEVEL_DSA  = 0x02
ALL_ALIASES = {**DEVICE_ALIASES, **PEDAL_ALIASES}
DEVICE_NAMES = {dev: " / ".join(names) for dev, names in ALL_ALIASES.items()}  # for logs

# How long silence has to last before we act on it. These are set from MEASUREMENT, not from
# the firmware's advertising interval: a pedal advertising every 100-150 ms (~7-10/s) is only
# delivered to this PC at ~1-3 ads/s, with normal gaps up to ~3 s. Windows/WinRT aggregates
# repeat advertisements from the same device and there is no knob that fixes it (WinRT's
# SignalStrengthFilter sampling interval was measured to make delivery WORSE, not better).
# So anything under ~3 s produces false "lost pedal" events.
PEDAL_OFFLINE_AFTER = 5.0   # mark the pedal offline in the UI (cosmetic — be generous)
# Clearing a HELD level is the fail-safe, so it wants to be short; but too short and a real
# RF gap releases the pedal mid-press, which is worse (X-ray flickering off during fluoro)
# than a dead pedal taking a few seconds to clear. This is a training sim, not a live tube.
LEVEL_RELEASE_AFTER = 4.0

PORT = 8765
PACKET = struct.Struct("<7f4B")           # w x y z ax ay az, calib, touchStart, touchCurrent, xrayOn = 32 bytes
BROADCAST_INTERVAL = 1 / 30               # UI doesn't need the firmware's full 50 Hz

# Latest known state per device, broadcast as-is to the browser. (Pedals reuse the same
# shape — their quat/accel/touch fields simply never change.)
state = {
    dev: {"connected": False, "w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0,
          "ax": 0.0, "ay": 0.0, "az": 0.0, "calib": 0,
          "touchStart": 0, "touchCurrent": 0, "touchActive": 0}
    for dev in DEVICE_NAMES
}
# X-ray comes from three independent sources, OR'd together:
#   latched_xray — hand-tracker / mini BUTTONS toggle it (click on, click off)
#   foot_xray / panel_xray — X-ray held, from the FOOT PEDAL and the TOUCH PANEL respectively
#   foot_dsa  / panel_dsa  — DSA run in progress, from those same two sources
# The pedal and the panel are INDEPENDENT sources of the same two signals, so each keeps its
# own level and the effective signal is the OR (see xray_held()/dsa_running()). Sharing one
# variable meant an idle panel's advertisement cancelled a genuinely held foot pedal.
# A DSA run IS an X-ray acquisition, so a run implies imaging and also fills the vessel.
# Effective state = OR of the three, so a pedal always wins while it's down and releasing
# it returns to whatever the buttons had latched.
latched_xray = False
foot_xray = False
panel_xray = False
foot_dsa = False
panel_dsa = False
dsa_runs = 0                              # DSA contrast runs performed
dsa_fault = False                         # DSA switch wiring fault (COM/NO/NC self-check)
capture_count = 0                         # total X-ray captures (right pedal + mini capture buttons)
last_xray_bit = {dev: 0 for dev in DEVICE_NAMES}      # per-device byte-31 bit, for edge detection
last_capture_bit = {dev: 0 for dev in DEVICE_NAMES}   # per-device byte-28 bit (Mini trackers only)
last_pedal_count: dict[str, int] = {}     # pedal → last broadcast pressCount (edge detection)
last_pedal_seen: dict[str, float] = {}    # pedal → loop time of its last advertisement


def xray_on() -> bool:
    """The single shared X-ray state the UI and simulation consume."""
    return latched_xray or xray_held() or dsa_running()


def xray_held() -> bool:
    """X-ray held down on ANY source (foot pedal or touch panel)."""
    return foot_xray or panel_xray


def dsa_running() -> bool:
    """A contrast run in progress on ANY source."""
    return foot_dsa or panel_dsa


busy: set[str] = set()                    # devices currently connecting/connected
websockets: set[web.WebSocketResponse] = set()


def on_packet(dev: str, data: bytearray, mini: bool = False, panel: bool = False) -> None:
    """One 32-byte notification from a CONNECTED hand tracker (pedals never get here)."""
    # panel_xray/panel_dsa MUST be declared here: the panel path below assigns them, and
    # without this they would silently become locals and never reach xray_on().
    global latched_xray, capture_count, panel_xray, panel_dsa
    if len(data) < PACKET.size:
        return
    w, x, y, z, ax, ay, az, calib, tStart, tCur, xrayBit = PACKET.unpack(bytes(data[:PACKET.size]))
    # Mini trackers repurpose the calib byte as a capture-toggle bit (their MPU-6050 has no
    # BNO-style calibration status). A BNO085 tracker's calib legitimately changes 0-3, so
    # this ONLY applies to minis.
    if mini:
        if calib != last_capture_bit[dev]:
            last_capture_bit[dev] = calib
            capture_count += 1
            print(f"[capture] {dev} mini button -> X-RAY CAPTURE #{capture_count}")
        calib = 0   # don't show the flip bit as a calibration value
    # The PANEL is different from every other connected board: its byte 31 is a live LEVEL
    # bitfield from two hold-buttons, not a flip, and byte 28 is a capture counter.
    if panel:
        held = bool(xrayBit & PANEL_LEVEL_XRAY)
        run = bool(xrayBit & PANEL_LEVEL_DSA)
        if held != panel_xray:
            panel_xray = held
            print(f"[xray] panel X-ray {'DOWN' if held else 'UP'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
        if run != panel_dsa:
            panel_dsa = run
            print(f"[dsa] panel contrast run {'START' if run else 'END'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
        if calib != last_capture_bit[dev]:
            last_capture_bit[dev] = calib
            capture_count += 1
            print(f"[capture] panel -> X-RAY CAPTURE #{capture_count}")
        calib = 0                      # not a calibration value; don't show it as one
        state[dev].update(w=w, x=x, y=y, z=z, ax=ax, ay=ay, az=az, calib=calib,
                          touchStart=tStart, touchCurrent=tCur,
                          touchActive=1 if tCur > 0 else 0)
        return

    # A hand board flips byte 31 on each X-ray button click → TOGGLE the latched state.
    if xrayBit != last_xray_bit[dev]:
        last_xray_bit[dev] = xrayBit
        latched_xray = not latched_xray
        print(f"[xray] {dev} button -> X-RAY {'ON' if xray_on() else 'OFF'}")
    state[dev].update(w=w, x=x, y=y, z=z, ax=ax, ay=ay, az=az, calib=calib,
                      touchStart=tStart, touchCurrent=tCur, touchActive=1 if tCur > 0 else 0)


def on_pedal_ad(pedal: str, mfg_data: dict[int, bytes], now: float) -> None:
    """One advertisement from a broadcast-only pedal.

    Manufacturer data is [count, level(, flags)] under company ID 0xFFFF (the two 0xFF
    bytes the firmware writes first ARE that ID — bleak strips them and keys the dict by
    it). All three pedals share the first two payload bytes:
      LEFT  pedal: `level` is a LIVE LEVEL → X-ray on while the foot is down.
      RIGHT pedal: a changed `count` → fire exactly one screen capture.
      DSA   pedal: `level` is a LIVE LEVEL → contrast run in progress (implies X-ray);
                   a changed `count` numbers the runs; `flags` bit0 = switch wiring fault.
    """
    global foot_xray, panel_xray, foot_dsa, panel_dsa
    global capture_count, dsa_runs, dsa_fault, latched_xray
    payload = mfg_data.get(0xFFFF)
    if not payload:
        return

    first_sighting = pedal not in last_pedal_seen
    last_pedal_seen[pedal] = now
    state[pedal]["connected"] = True
    if first_sighting:
        print(f"[{pedal}] detected (broadcast) — mfg payload {len(payload)}B: {payload.hex(' ')}")
        # A LEVEL pedal running pre-hold-to-activate firmware sends no level byte, so it
        # would sit there looking "Detected" while never activating. Say so, don't go quiet.
        if pedal in ("left-foot", "dsa-foot") and len(payload) < 2:
            print(f"[{pedal}] !! OLD FIRMWARE: broadcast has no level byte, so hold-to-"
                  f"activate CANNOT work. Reflash firmware/{pedal}/{pedal}.ino")

    count = payload[0]
    level_raw = payload[1] if len(payload) > 1 else 0
    level = bool(level_raw)          # single-flag pedals; the panel reads level_raw as bits
    flags = payload[2] if len(payload) > 2 else 0

    if pedal == "left-foot":
        if level != foot_xray:
            foot_xray = level
            print(f"[xray] left pedal {'DOWN' if level else 'UP'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
    elif pedal == "right-foot":
        # Baseline on first sighting, so a restart of this script doesn't fire a phantom
        # capture from whatever count the pedal happens to be broadcasting.
        if not first_sighting and last_pedal_count.get(pedal) != count:
            capture_count += 1
            print(f"[capture] right pedal -> X-RAY CAPTURE #{capture_count}")
    elif pedal == "panel":
        # One device carrying all three controls: two live levels plus the capture counter.
        held = bool(level_raw & PANEL_LEVEL_XRAY)
        run  = bool(level_raw & PANEL_LEVEL_DSA)
        if held != panel_xray:
            panel_xray = held
            print(f"[xray] panel X-ray {'DOWN' if held else 'UP'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
        if run != panel_dsa:
            panel_dsa = run
            print(f"[dsa] panel contrast run {'START' if run else 'END'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
        if not first_sighting and last_pedal_count.get(pedal) != count:
            capture_count += 1
            print(f"[capture] panel -> X-RAY CAPTURE #{capture_count}")
    elif pedal == "dsa-foot":
        fault = bool(flags & 0x01)
        if fault != dsa_fault:
            dsa_fault = fault
            print("[dsa] SWITCH FAULT — check COM/NO/NC wiring" if fault
                  else "[dsa] switch wiring OK again")
        if level != foot_dsa:
            foot_dsa = level
            print(f"[dsa] contrast run {'START' if level else 'END'} -> X-RAY "
                  f"{'ON' if xray_on() else 'OFF'}")
        if not first_sighting and last_pedal_count.get(pedal) != count:
            dsa_runs += 1
            print(f"[dsa] run #{dsa_runs}")
    last_pedal_count[pedal] = count


def expire_stale_pedals(now: float) -> None:
    """Act on pedal silence, in two stages (see the timeout constants).

    LEVEL_RELEASE_AFTER — clear a held level. This is the fail-safe: a pedal that dies
        mid-press must not leave X-ray or a contrast run latched on forever.
    PEDAL_OFFLINE_AFTER — mark the pedal offline in the UI. Deliberately longer, because
        normal advertisement gaps are seconds long and flapping the badge (and spamming
        the console) on every gap made real problems impossible to spot.
    """
    global foot_xray, panel_xray, foot_dsa, panel_dsa, dsa_fault
    for pedal in PEDAL_ALIASES:
        seen = last_pedal_seen.get(pedal)
        if seen is None:
            continue
        silent = now - seen

        # Stage 1: release any held level.
        if silent > LEVEL_RELEASE_AFTER:
            if pedal == "left-foot" and foot_xray:
                foot_xray = False
                print(f"[xray] left pedal silent {silent:.1f}s while held -> X-RAY OFF (fail-safe)")
            if pedal == "dsa-foot" and foot_dsa:
                foot_dsa = False
                print(f"[dsa] pedal silent {silent:.1f}s mid-run -> run ENDED, X-RAY OFF (fail-safe)")
            if pedal == "__unused_panel__":
                # The panel owns BOTH levels, so a lost panel must clear both.
                if panel_xray or panel_dsa:
                    panel_xray = panel_dsa = False
                    print(f"[xray] panel silent {silent:.1f}s -> X-RAY OFF, run ENDED (fail-safe)")

        # Stage 2: it's been quiet long enough to call it gone.
        if silent > PEDAL_OFFLINE_AFTER:
            del last_pedal_seen[pedal]
            last_pedal_count.pop(pedal, None)
            state[pedal]["connected"] = False
            if pedal == "dsa-foot":
                dsa_fault = False   # can't know the wiring of a pedal we can't hear
            print(f"[{pedal}] broadcast lost — will re-detect")


async def serve_device(dev: str, device, mini: bool = False, panel: bool = False) -> None:
    """Hold the connection to one board, route its notifications, retry on drop."""
    disconnected = asyncio.Event()
    try:
        async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
            await client.start_notify(CHAR_UUID, lambda _, data: on_packet(dev, data, mini, panel))
            state[dev]["connected"] = True
            print(f"[{dev}] connected: {device.name or device.address}")
            await disconnected.wait()
    except Exception as e:
        print(f"[{dev}] connection error: {e}")
    finally:
        state[dev]["connected"] = False
        busy.discard(dev)
        print(f"[{dev}] disconnected — will rescan")


seen_names: set[str] = set()   # debug aid: log each BLE name once, as it first appears


def on_advertisement(device, adv) -> None:
    """Every advertisement the continuous scan hears.

    Pedals are handled entirely here (they never connect). A hand tracker sighting spawns
    a connection task the first time we see it while its slot is free.
    """
    name = adv.local_name or device.name or ""
    if not name:
        return
    if name not in seen_names:
        seen_names.add(name)
        print(f"  BLE name in range: {name}")

    for pedal, aliases in PEDAL_ALIASES.items():
        if any(a in name for a in aliases):
            on_pedal_ad(pedal, adv.manufacturer_data, time.monotonic())
            return
    for dev, aliases in DEVICE_ALIASES.items():
        if dev not in busy and any(a in name for a in aliases):
            busy.add(dev)
            asyncio.create_task(serve_device(dev, device, mini="Mini" in name,
                                                         panel="Panel" in name))
            return


async def scan_loop() -> None:
    """Scan CONTINUOUSLY — never stop.

    The pedals only ever exist as advertisements, so a steady scan is the only way to
    see a press; the same stream also re-discovers a hand tracker that drops. Duplicate
    advertisements must be allowed through, otherwise repeated pedal ads (which carry the
    live held level) would be filtered out.

    Windows/WinRT note: connect with the discovered BLEDevice object (not a bare address
    string) — it's the reliable path with bleak's WinRT backend.
    """
    while True:
        try:
            scanner = BleakScanner(detection_callback=on_advertisement)
            await scanner.start()
            print("Scanning continuously — hands connect, pedals are read from broadcasts")
            try:
                while True:
                    await asyncio.sleep(0.25)
                    expire_stale_pedals(time.monotonic())
            finally:
                await scanner.stop()
        except Exception as e:
            print(f"Scan failed ({e}) — is Bluetooth on? Retrying…")
            await asyncio.sleep(3)


async def broadcast_loop() -> None:
    while True:
        if websockets:
            payload = json.dumps({**state, "xray": xray_on(), "pedalHeld": xray_held(),
                                  "captureCount": capture_count, "dsaActive": dsa_running(),
                                  "dsaRuns": dsa_runs, "dsaFault": dsa_fault})
            await asyncio.gather(
                *(ws.send_str(payload) for ws in list(websockets)),
                return_exceptions=True,   # a closing socket shouldn't kill the loop
            )
        await asyncio.sleep(BROADCAST_INTERVAL)


async def handle_index(request: web.Request) -> web.FileResponse:
    return web.FileResponse(Path(__file__).parent / "index.html")


async def handle_ws(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    websockets.add(ws)
    try:
        async for msg in ws:               # drain (client never sends; detect close)
            if msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                break
    finally:
        websockets.discard(ws)
    return ws


def lan_ip() -> str:
    """Best-effort LAN IP of this machine (for reaching the dashboard from a phone)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))   # no packets sent; just selects the outbound interface
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


async def main() -> None:
    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)

    runner = web.AppRunner(app)
    await runner.setup()
    # Bind to 0.0.0.0 so phones/tablets on the same Wi-Fi can reach it — not just this PC.
    site = web.TCPSite(runner, "0.0.0.0", PORT)
    await site.start()

    local_url = f"http://localhost:{PORT}"
    phone_url = f"http://{lan_ip()}:{PORT}"
    print("Dashboard running:")
    print(f"  this PC : {local_url}")
    print(f"  phone   : {phone_url}   (same Wi-Fi)")
    print("  (Ctrl+C to stop)")
    webbrowser.open(local_url)

    await asyncio.gather(scan_loop(), broadcast_loop())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")

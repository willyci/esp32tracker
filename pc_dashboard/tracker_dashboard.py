# PC BLE dashboard for the two hand trackers + two foot pedals.
#
# Connects to the ESP32-C3 hand trackers ("Left/Right Hand Tracker") and the ESP32-S3
# foot pedals ("Left/Right Foot Pedal") over Bluetooth LE, parses the same 32-byte
# packets the visionOS app consumes, and pushes them over a WebSocket to a local web
# page (index.html) that renders two live 3D cubes, the simulation panel, and pedal
# status. Left pedal toggles the shared X-ray; right pedal fires a screen capture.
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
# "Mini Tracker" = the ESP32-C3 0.42"-OLED glove unit (SoftPot + 2 buttons, NO IMU). It is
# a drop-in alternative for a hand slot: quaternion stays identity, and its second button
# reports X-ray captures by flipping byte 28 (the calib slot, meaningless without an IMU).
DEVICE_ALIASES = {
    "left":       ("Left Hand Tracker",  "Left Mini Tracker"),
    "right":      ("Right Hand Tracker", "Right Mini Tracker"),
    "left-foot":  ("Left Foot Pedal",),   # byte-31 flip = toggle the shared X-ray
    "right-foot": ("Right Foot Pedal",),  # byte-31 flip = one X-ray screen capture
}
DEVICE_NAMES = {dev: " / ".join(names) for dev, names in DEVICE_ALIASES.items()}  # for logs

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
master_xray = False                       # single shared X-ray state (hands + left pedal toggle it)
capture_count = 0                         # total X-ray captures (right pedal + mini capture buttons)
last_xray_bit = {dev: 0 for dev in DEVICE_NAMES}      # per-device byte-31 bit, for edge detection
last_capture_bit = {dev: 0 for dev in DEVICE_NAMES}   # per-device byte-28 bit (Mini trackers only)
busy: set[str] = set()                    # devices currently connecting/connected
websockets: set[web.WebSocketResponse] = set()


def on_packet(dev: str, data: bytearray, mini: bool = False) -> None:
    global master_xray, capture_count
    if len(data) < PACKET.size:
        return
    w, x, y, z, ax, ay, az, calib, tStart, tCur, xrayBit = PACKET.unpack(bytes(data[:PACKET.size]))
    # Mini trackers have no IMU and repurpose the calib byte as a capture-toggle bit.
    # (Real trackers' calib legitimately changes 0-3, so this ONLY applies to minis.)
    if mini:
        if calib != last_capture_bit[dev]:
            last_capture_bit[dev] = calib
            capture_count += 1
            print(f"[capture] {dev} mini button -> X-RAY CAPTURE #{capture_count}")
        calib = 0   # don't show the flip bit as a calibration value
    # Every board flips xrayBit on each button/pedal press; the MEANING depends on who sent it:
    # the right foot pedal fires one screen capture, everything else toggles the shared X-ray.
    if xrayBit != last_xray_bit[dev]:
        last_xray_bit[dev] = xrayBit
        if dev == "right-foot":
            capture_count += 1
            print(f"[capture] right pedal -> X-RAY CAPTURE #{capture_count}")
        else:
            master_xray = not master_xray
            print(f"[xray] {dev} button -> X-RAY {'ON' if master_xray else 'OFF'}")
    state[dev].update(w=w, x=x, y=y, z=z, ax=ax, ay=ay, az=az, calib=calib,
                      touchStart=tStart, touchCurrent=tCur, touchActive=1 if tCur > 0 else 0)


async def serve_device(dev: str, device, mini: bool = False) -> None:
    """Hold the connection to one board, route its notifications, retry on drop."""
    disconnected = asyncio.Event()
    try:
        async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
            await client.start_notify(CHAR_UUID, lambda _, data: on_packet(dev, data, mini))
            state[dev]["connected"] = True
            print(f"[{dev}] connected: {device.name or device.address}")
            await disconnected.wait()
    except Exception as e:
        print(f"[{dev}] connection error: {e}")
    finally:
        state[dev]["connected"] = False
        busy.discard(dev)
        print(f"[{dev}] disconnected — will rescan")


async def scan_loop() -> None:
    """Keep scanning until both hands are connected; resume when either drops.

    Windows/WinRT note: connect with the discovered BLEDevice object (not a bare
    address string) — it's the reliable path with bleak's WinRT backend.
    """
    while True:
        missing = [d for d in DEVICE_NAMES if d not in busy]
        if missing:
            print(f"Scanning for: {', '.join(DEVICE_NAMES[d] for d in missing)}")
            try:
                found = await BleakScanner.discover(timeout=4.0, return_adv=True)
            except Exception as e:
                print(f"Scan failed ({e}) — is Bluetooth on? Retrying…")
                await asyncio.sleep(3)
                continue
            # Debug aid: list every named device in range, so "is the board advertising
            # at all?" is answerable straight from this console.
            named = sorted({adv.local_name or device.name
                            for device, adv in found.values()
                            if adv.local_name or device.name})
            if named:
                print(f"  BLE names in range: {', '.join(named)}")
            for device, adv in found.values():
                name = adv.local_name or device.name or ""
                for dev, aliases in DEVICE_ALIASES.items():
                    if dev not in busy and any(a in name for a in aliases):
                        busy.add(dev)
                        asyncio.create_task(serve_device(dev, device, mini="Mini" in name))
        await asyncio.sleep(2)


async def broadcast_loop() -> None:
    while True:
        if websockets:
            payload = json.dumps({**state, "xray": master_xray, "captureCount": capture_count})
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

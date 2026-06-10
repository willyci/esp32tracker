# PC BLE dashboard for the two hand trackers.
#
# Connects to BOTH ESP32-C3 boards ("Left Hand Tracker" / "Right Hand Tracker") over
# Bluetooth LE, parses the same 32-byte packets the visionOS app consumes, and pushes
# them over a WebSocket to a local web page (index.html) that renders two live 3D cubes.
#
#   pip install bleak aiohttp
#   python tracker_dashboard.py        → opens http://localhost:8765
#
# Protocol (must match ../firmware/esp32_tracker/esp32_tracker.ino — see ../SPEC.md):
#   service 4F7A0001-..., notify char 4F7A0002-...
#   packet: float32 w,x,y,z (quaternion), float32 ax,ay,az (m/s^2), uint8 calib, 3 pad

import asyncio
import json
import struct
import webbrowser
from pathlib import Path

from aiohttp import web, WSMsgType
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "4f7a0001-9b3e-4c2a-8d1f-0a1b2c3d4e5f"
CHAR_UUID    = "4f7a0002-9b3e-4c2a-8d1f-0a1b2c3d4e5f"

# Advertised names set by IS_LEFT_HAND in the firmware. The name arrives in the BLE
# scan response (it doesn't fit the main advertising packet next to the 128-bit UUID).
HAND_NAMES = {
    "left":  "Left Hand Tracker",
    "right": "Right Hand Tracker",
}

PORT = 8765
PACKET = struct.Struct("<7fB3x")          # w x y z ax ay az calib (+3 pad) = 32 bytes
BROADCAST_INTERVAL = 1 / 30               # UI doesn't need the firmware's full 50 Hz

# Latest known state per hand, broadcast as-is to the browser.
state = {
    hand: {"connected": False, "w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0,
           "ax": 0.0, "ay": 0.0, "az": 0.0, "calib": 0}
    for hand in HAND_NAMES
}
busy: set[str] = set()                    # hands currently connecting/connected
websockets: set[web.WebSocketResponse] = set()


def on_packet(hand: str, data: bytearray) -> None:
    if len(data) < PACKET.size:
        return
    w, x, y, z, ax, ay, az, calib = PACKET.unpack(bytes(data[:PACKET.size]))
    state[hand].update(w=w, x=x, y=y, z=z, ax=ax, ay=ay, az=az, calib=calib)


async def serve_device(hand: str, device) -> None:
    """Hold the connection to one board, route its notifications, retry on drop."""
    disconnected = asyncio.Event()
    try:
        async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
            await client.start_notify(CHAR_UUID, lambda _, data: on_packet(hand, data))
            state[hand]["connected"] = True
            print(f"[{hand}] connected: {device.name or device.address}")
            await disconnected.wait()
    except Exception as e:
        print(f"[{hand}] connection error: {e}")
    finally:
        state[hand]["connected"] = False
        busy.discard(hand)
        print(f"[{hand}] disconnected — will rescan")


async def scan_loop() -> None:
    """Keep scanning until both hands are connected; resume when either drops.

    Windows/WinRT note: connect with the discovered BLEDevice object (not a bare
    address string) — it's the reliable path with bleak's WinRT backend.
    """
    while True:
        missing = [h for h in HAND_NAMES if h not in busy]
        if missing:
            print(f"Scanning for: {', '.join(HAND_NAMES[h] for h in missing)}")
            try:
                found = await BleakScanner.discover(timeout=4.0, return_adv=True)
            except Exception as e:
                print(f"Scan failed ({e}) — is Bluetooth on? Retrying…")
                await asyncio.sleep(3)
                continue
            for device, adv in found.values():
                name = adv.local_name or device.name or ""
                for hand, expected in HAND_NAMES.items():
                    if expected in name and hand not in busy:
                        busy.add(hand)
                        asyncio.create_task(serve_device(hand, device))
        await asyncio.sleep(2)


async def broadcast_loop() -> None:
    while True:
        if websockets:
            payload = json.dumps(state)
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


async def main() -> None:
    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "localhost", PORT)
    await site.start()

    url = f"http://localhost:{PORT}"
    print(f"Dashboard running at {url}  (Ctrl+C to stop)")
    webbrowser.open(url)

    await asyncio.gather(scan_loop(), broadcast_loop())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")

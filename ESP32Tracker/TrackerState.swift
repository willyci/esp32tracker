import Foundation
import CoreBluetooth
import simd

/// Which physical tracker a peripheral is. The raw value MUST match the firmware's
/// `DEVICE_NAME` ("Left Hand Tracker" / "Right Hand Tracker"), set by the
/// `IS_LEFT_HAND` switch in esp32_tracker.ino.
enum Hand: String, CaseIterable, Identifiable {
    case left  = "Left Hand Tracker"
    case right = "Right Hand Tracker"

    var id: String { rawValue }
    var label: String { self == .left ? "Left" : "Right" }

    /// Map a BLE advertised name to a hand (nil if it isn't one of our trackers).
    static func from(advertisedName name: String?) -> Hand? {
        guard let name else { return nil }
        return Hand.allCases.first { name.contains($0.rawValue) }
    }
}

/// High-level connection status for a single tracker, surfaced to the UI.
enum ConnectionState: String {
    case bluetoothOff = "Bluetooth off"
    case scanning     = "Scanning…"
    case connecting   = "Connecting…"
    case connected    = "Connected"
    case disconnected = "Disconnected"
}

/// Per-hand sensor state plus the orientation-shaping math. One instance per tracker;
/// `BLEManager` owns two of them (left + right) and routes incoming packets here.
///
/// Packet layout (32 bytes, little-endian — see SPEC.md):
///   [0..16)  quaternion w,x,y,z (float32 each)
///   [16..28) accel x,y,z (float32 each, m/s²)
///   [28]     calibration status 0–3 (uint8)
final class TrackerState: ObservableObject, Identifiable {
    let hand: Hand
    var id: Hand { hand }

    @Published private(set) var connection: ConnectionState = .disconnected
    @Published private(set) var quaternion = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) // raw, sensor frame
    @Published private(set) var accel = SIMD3<Float>(0, 0, 0)
    @Published private(set) var calibration: UInt8 = 0

    // MARK: Orientation shaping (identical math to the original single-device app)

    /// Inverse of the sensor pose captured on "Re-center"; makes that pose the new zero.
    private var reference = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Change of basis: BNO085 frame (Z-up) → RealityKit (Y-up). See FRAME_MAPPING.md.
    private let basis = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))
                      * simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(0, 0, 1))

    /// Physical mounting offset (how the sensor is glued to the hand). Tune per FRAME_MAPPING.md.
    private let mountOffset = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))

    /// What the 3D entity for this hand should use.
    var displayOrientation: simd_quatf {
        basis * (reference * quaternion) * basis.inverse * mountOffset
    }

    init(hand: Hand) { self.hand = hand }

    /// Called by BLEManager as the connection state changes.
    func setConnection(_ newState: ConnectionState) { connection = newState }

    /// Capture this hand's current pose as its new zero orientation.
    func recenter() { reference = quaternion.inverse }

    /// Parse one 32-byte notification and publish the new values.
    func ingest(_ data: Data) {
        guard data.count >= 29 else { return }
        let w = data.readFloat(at: 0)
        let x = data.readFloat(at: 4)
        let y = data.readFloat(at: 8)
        let z = data.readFloat(at: 12)
        // Negate x and z: the BNO085 frame is mirrored vs RealityKit on roll+yaw
        // (flipping those two — not pitch — is a valid rotation). See BLEManager history.
        quaternion  = simd_quatf(ix: -x, iy: y, iz: -z, r: w)   // (x, y, z, w) arg order
        accel       = SIMD3<Float>(data.readFloat(at: 16), data.readFloat(at: 20), data.readFloat(at: 24))
        calibration = data[data.startIndex + 28]
    }
}

// MARK: - Little-endian reads (ESP32 and Apple silicon are both little-endian → no swap)
private extension Data {
    func readFloat(at offset: Int) -> Float {
        withUnsafeBytes { $0.loadUnaligned(fromByteOffset: offset, as: Float32.self) }
    }
}

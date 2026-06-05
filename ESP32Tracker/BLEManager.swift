import Foundation
import CoreBluetooth
import simd

/// High-level connection status, surfaced to the UI.
enum ConnectionState: String {
    case bluetoothOff = "Bluetooth off"
    case scanning     = "Scanning…"
    case connecting   = "Connecting…"
    case connected    = "Connected"
    case disconnected = "Disconnected"
}

/// Owns the Core Bluetooth session and exposes the latest IMU sample to SwiftUI.
///
/// The ESP32-C3 advertises one service with one notify characteristic carrying a
/// packed 32-byte little-endian frame (see SPEC.md):
///   [0..16)  quaternion w,x,y,z (float32 each)
///   [16..28) accel x,y,z (float32 each, m/s²)
///   [28]     calibration status 0–3 (uint8)
final class BLEManager: NSObject, ObservableObject {

    // MARK: Identifiers — must match the firmware exactly.
    static let serviceUUID        = CBUUID(string: "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F")
    static let orientationCharUUID = CBUUID(string: "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F")

    // MARK: Published state (CBCentralManager uses the main queue, so these update on main).
    @Published private(set) var connection: ConnectionState = .disconnected
    @Published private(set) var quaternion = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) // raw, sensor frame
    @Published private(set) var accel = SIMD3<Float>(0, 0, 0)
    @Published private(set) var calibration: UInt8 = 0

    // MARK: Orientation shaping
    /// Inverse of the sensor pose captured on "Re-center"; makes that pose the new zero.
    private var reference = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Change of basis: BNO085 frame (Z-up, right-handed) → RealityKit (Y-up, right-handed).
    /// +90° about Z made pitch correct but swapped roll↔yaw; composing a further +90° about X
    /// (the now-correct pitch axis) swaps roll/yaw back without disturbing pitch, so all three
    /// motions map correctly. Applied as a conjugation below — see FRAME_MAPPING.md.
    private let basis = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))
                      * simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(0, 0, 1))

    /// Corrects for how the sensor is physically glued onto the object (mounting offset).
    /// Leave as identity until the box rotates on the right axes, then tune — see the guide.
    private let mountOffset = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))

    /// What the 3D entity should actually use.
    /// `basis * X * basis.inverse` re-expresses the sensor rotation in RealityKit's frame.
    var displayOrientation: simd_quatf {
        basis * (reference * quaternion) * basis.inverse * mountOffset
    }

    // MARK: Core Bluetooth plumbing
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    func startScan() {
        guard central.state == .poweredOn else { return }
        connection = .scanning
        central.scanForPeripherals(withServices: [Self.serviceUUID])
    }

    func disconnect() {
        if let peripheral { central.cancelPeripheralConnection(peripheral) }
    }

    /// Capture the current pose as the new zero orientation.
    func recenter() {
        reference = quaternion.inverse
    }
}

// MARK: - CBCentralManagerDelegate
extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:  startScan()
        case .poweredOff: connection = .bluetoothOff
        default:          connection = .disconnected
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        // First match wins for v0. Add an RSSI/name filter later if multiple boards appear.
        self.peripheral = peripheral
        peripheral.delegate = self
        connection = .connecting
        central.stopScan()
        central.connect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        connection = .disconnected
        self.peripheral = nil
    }
}

// MARK: - CBPeripheralDelegate
extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else { return }
        peripheral.discoverCharacteristics([Self.orientationCharUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        guard let ch = service.characteristics?.first(where: { $0.uuid == Self.orientationCharUUID }) else { return }
        peripheral.setNotifyValue(true, for: ch)
        connection = .connected
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard let data = characteristic.value, data.count >= 29 else { return }
        let w = data.readFloat(at: 0)
        let x = data.readFloat(at: 4)
        let y = data.readFloat(at: 8)
        let z = data.readFloat(at: 12)
        // Negate x and z (reflect about the sensor's pitch axis): the BNO085's frame is
        // mirrored vs. RealityKit on roll+yaw. Flipping those two — not pitch — is a valid
        // rotation (reversing a single axis alone would be a mirror, not a rotation).
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

import Foundation
import CoreBluetooth
import simd

/// Owns the Core Bluetooth session and drives BOTH hand trackers.
///
/// Both ESP32 boards advertise the same service + characteristic UUID; they differ
/// only by advertised name ("Left Hand Tracker" / "Right Hand Tracker"). This manager
/// scans for the service, identifies each board by name, connects to both, and routes
/// each board's notifications to its own `TrackerState`.
final class BLEManager: NSObject, ObservableObject {

    // MARK: Identifiers — must match the firmware exactly.
    static let serviceUUID         = CBUUID(string: "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F")
    static let orientationCharUUID = CBUUID(string: "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F")

    // MARK: Per-hand state. Identity is stable, so SwiftUI views @ObservedObject these directly.
    let left  = TrackerState(hand: .left)
    let right = TrackerState(hand: .right)

    @Published private(set) var bluetoothReady = false

    // MARK: Core Bluetooth plumbing
    private var central: CBCentralManager!
    /// Strong refs to the peripherals we're connected/connecting to, keyed by hand.
    private var peripherals: [Hand: CBPeripheral] = [:]
    /// Reverse lookup so delegate callbacks (which only hand us a peripheral) know the hand.
    private var handFor: [UUID: Hand] = [:]

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    func state(for hand: Hand) -> TrackerState { hand == .left ? left : right }

    private var allConnected: Bool { peripherals.count == Hand.allCases.count }

    // MARK: Public controls
    func startScan() {
        guard central.state == .poweredOn else { return }
        for hand in Hand.allCases where peripherals[hand] == nil {
            state(for: hand).setConnection(.scanning)
        }
        central.scanForPeripherals(withServices: [Self.serviceUUID])
    }

    func recenter(_ hand: Hand) { state(for: hand).recenter() }
    func recenterAll() { Hand.allCases.forEach { state(for: $0).recenter() } }

    func disconnect(_ hand: Hand) {
        if let p = peripherals[hand] { central.cancelPeripheralConnection(p) }
    }
    func disconnectAll() { Hand.allCases.forEach(disconnect) }

    private func cleanup(_ peripheral: CBPeripheral, newState: ConnectionState) {
        guard let hand = handFor[peripheral.identifier] else { return }
        state(for: hand).setConnection(newState)
        peripherals[hand] = nil
        handFor[peripheral.identifier] = nil
    }
}

// MARK: - CBCentralManagerDelegate
extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            bluetoothReady = true
            startScan()
        case .poweredOff:
            bluetoothReady = false
            left.setConnection(.bluetoothOff)
            right.setConnection(.bluetoothOff)
        default:
            bluetoothReady = false
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        // Identify which hand this is by its advertised name (scan-response local name).
        let advName = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? peripheral.name
        guard let hand = Hand.from(advertisedName: advName) else { return }   // not one of ours
        guard peripherals[hand] == nil else { return }                        // already have this hand

        peripherals[hand] = peripheral
        handFor[peripheral.identifier] = hand
        peripheral.delegate = self
        state(for: hand).setConnection(.connecting)
        central.connect(peripheral)

        if allConnected { central.stopScan() }   // both boards found — stop scanning
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral, error: Error?) {
        cleanup(peripheral, newState: .disconnected)
        startScan()
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        cleanup(peripheral, newState: .disconnected)
        startScan()   // try to bring this hand back
    }
}

// MARK: - CBPeripheralDelegate
extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else { return }
        peripheral.discoverCharacteristics([Self.orientationCharUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let ch = service.characteristics?.first(where: { $0.uuid == Self.orientationCharUUID }) else { return }
        peripheral.setNotifyValue(true, for: ch)
        if let hand = handFor[peripheral.identifier] {
            state(for: hand).setConnection(.connected)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value,
              let hand = handFor[peripheral.identifier] else { return }
        state(for: hand).ingest(data)
    }
}

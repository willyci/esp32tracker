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
    /// Writable: we push the shared X-ray state (1 byte) to every board so their
    /// displays/LEDs stay in sync — the boards only send flip events, we own the state.
    static let xrayStateCharUUID   = CBUUID(string: "4F7A0003-9B3E-4C2A-8D1F-0A1B2C3D4E5F")

    // MARK: Per-hand state. Identity is stable, so SwiftUI views @ObservedObject these directly.
    let left  = TrackerState(hand: .left)
    let right = TrackerState(hand: .right)

    @Published private(set) var bluetoothReady = false

    /// Single shared X-ray state — the button on EITHER hand toggles it (each board just
    /// flips its packet bit per click; we own the actual on/off, like the PC dashboard).
    @Published private(set) var xrayOn = false

    /// Optional third device: the foot pedal ("Foot Pedal"), which only toggles X-ray.
    @Published private(set) var footPedalConnected = false
    static let footPedalName = "Foot Pedal"
    private var footPeripheral: CBPeripheral?
    /// Last pedal flip-bit seen; nil until the first packet so connecting never toggles.
    private var lastFootXrayBit: UInt8?

    // MARK: Core Bluetooth plumbing
    private var central: CBCentralManager!
    /// Strong refs to the peripherals we're connected/connecting to, keyed by hand.
    private var peripherals: [Hand: CBPeripheral] = [:]
    /// Reverse lookup so delegate callbacks (which only hand us a peripheral) know the hand.
    private var handFor: [UUID: Hand] = [:]
    /// Each connected board's writable X-ray-state characteristic, for state push-back.
    private var xrayStateChars: [UUID: CBCharacteristic] = [:]

    override init() {
        super.init()
        // Either hand's hardware button toggles the one shared X-ray state.
        left.onXrayToggle  = { [weak self] in self?.toggleXray() }
        right.onXrayToggle = { [weak self] in self?.toggleXray() }
        central = CBCentralManager(delegate: self, queue: nil)
    }

    /// Flip the shared X-ray state (hardware buttons, pedal, and the UI button all land
    /// here), then push the new truth to every connected board.
    func toggleXray() {
        xrayOn.toggle()
        pushXrayState()
    }

    /// Write the shared state to all boards (or one) so displays/LEDs stay in sync.
    private func pushXrayState(to onlyPeripheral: CBPeripheral? = nil) {
        let payload = Data([xrayOn ? 1 : 0])
        let targets = onlyPeripheral.map { [$0] }
            ?? Array(peripherals.values) + (footPeripheral.map { [$0] } ?? [])
        for p in targets {
            if let ch = xrayStateChars[p.identifier] {
                p.writeValue(payload, for: ch, type: .withResponse)
            }
        }
    }

    func state(for hand: Hand) -> TrackerState { hand == .left ? left : right }

    private var allConnected: Bool {
        peripherals.count == Hand.allCases.count && footPeripheral != nil
    }

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
    func disconnectAll() {
        Hand.allCases.forEach(disconnect)
        if let p = footPeripheral { central.cancelPeripheralConnection(p) }
    }

    private func cleanup(_ peripheral: CBPeripheral, newState: ConnectionState) {
        xrayStateChars[peripheral.identifier] = nil
        if peripheral.identifier == footPeripheral?.identifier {
            footPeripheral = nil
            footPedalConnected = false
            lastFootXrayBit = nil
            return
        }
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
        // Identify the device by its advertised name (scan-response local name).
        let advName = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? peripheral.name
        if let hand = Hand.from(advertisedName: advName) {
            guard peripherals[hand] == nil else { return }    // already have this hand
            peripherals[hand] = peripheral
            handFor[peripheral.identifier] = hand
            peripheral.delegate = self
            state(for: hand).setConnection(.connecting)
            central.connect(peripheral)
        } else if advName?.contains(Self.footPedalName) == true, footPeripheral == nil {
            footPeripheral = peripheral
            peripheral.delegate = self
            central.connect(peripheral)
        } else {
            return   // not one of ours
        }

        if allConnected { central.stopScan() }   // all boards found — stop scanning
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
        peripheral.discoverCharacteristics([Self.orientationCharUUID, Self.xrayStateCharUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let ch = service.characteristics?.first(where: { $0.uuid == Self.orientationCharUUID }) else { return }
        peripheral.setNotifyValue(true, for: ch)
        if peripheral.identifier == footPeripheral?.identifier {
            footPedalConnected = true
        } else if let hand = handFor[peripheral.identifier] {
            state(for: hand).setConnection(.connected)
        }

        // Sync the freshly connected board to the current shared X-ray state right away.
        // (Old firmware without this characteristic just never gets writes — harmless.)
        if let xr = service.characteristics?.first(where: { $0.uuid == Self.xrayStateCharUUID }) {
            xrayStateChars[peripheral.identifier] = xr
            pushXrayState(to: peripheral)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }

        // Foot pedal: only the flip-bit at byte 31 is live — any change toggles X-ray.
        if peripheral.identifier == footPeripheral?.identifier {
            guard data.count >= 32 else { return }
            let bit = data[data.startIndex + 31]
            if let last = lastFootXrayBit, bit != last { toggleXray() }
            lastFootXrayBit = bit
            return
        }

        guard let hand = handFor[peripheral.identifier] else { return }
        state(for: hand).ingest(data)
    }
}

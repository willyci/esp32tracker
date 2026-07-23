import Foundation
import CoreBluetooth
import simd

/// Which foot pedal a peripheral is. Pedals are CONNECTIONLESS — they broadcast a press
/// counter in their advertising manufacturer data and we read it from the scan (never
/// connect). Meaning is keyed by the advertised name (set by IS_LEFT_FOOT in firmware).
enum Pedal: String, CaseIterable {
    case leftFoot  = "Left Foot Pedal"    // press = toggle the shared X-ray
    case rightFoot = "Right Foot Pedal"   // press = fire one X-ray capture

    var label: String { self == .leftFoot ? "L foot (X-ray)" : "R foot (capture)" }

    static func from(advertisedName name: String?) -> Pedal? {
        guard let name else { return nil }
        return Pedal.allCases.first { name.contains($0.rawValue) }
    }
}

/// Owns the Core Bluetooth session. Drives the two hand trackers over CONNECTIONS and the
/// two foot pedals over ADVERTISEMENTS (broadcast).
///
/// Why the split: Vision Pro's BLE connection budget is small — with both hand trackers
/// connected, the system refuses more connections (CBError 11). A pedal only needs to say
/// "I was pressed," so instead of holding a scarce connection each pedal broadcasts a
/// press counter in its manufacturer data; we read it straight from the scan. Only the two
/// hands consume connection slots, so we stay well under the limit and the pedals can't
/// hit connection-failure modes at all.
final class BLEManager: NSObject, ObservableObject {

    // MARK: Identifiers — must match the firmware exactly.
    static let serviceUUID         = CBUUID(string: "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F")
    static let orientationCharUUID = CBUUID(string: "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F")

    // MARK: Per-hand state. Identity is stable, so SwiftUI views @ObservedObject these directly.
    let left  = TrackerState(hand: .left)
    let right = TrackerState(hand: .right)

    @Published private(set) var bluetoothReady = false

    /// Single shared X-ray state — either hand's button or the LEFT foot pedal toggles it.
    @Published private(set) var xrayOn = false

    /// Total X-ray captures fired by the RIGHT foot pedal (or the UI).
    @Published private(set) var captureCount = 0
    /// Fired after each capture — the playground records a snapshot of the sim state here.
    var onXrayCapture: (() -> Void)?

    /// Per-pedal "seen recently in the scan" status for the UI.
    @Published private(set) var pedalConnected: [Pedal: Bool] = [:]

    // MARK: In-app event log (the app runs untethered — this replaces the Xcode console).
    struct LogEntry: Identifiable {
        let id = UUID()
        let time = Date()
        let message: String
    }
    @Published private(set) var logEntries: [LogEntry] = []

    func log(_ message: String) {
        print("[BLE] \(message)")
        logEntries.append(LogEntry(message: message))
        if logEntries.count > 200 { logEntries.removeFirst(logEntries.count - 200) }
    }

    private func roleName(_ p: CBPeripheral) -> String {
        if let hand = handFor[p.identifier] { return hand.rawValue }
        return p.name ?? "unknown device"
    }

    // MARK: Core Bluetooth plumbing
    private var central: CBCentralManager!
    /// Hand-tracker connections (pedals are never connected).
    private var peripherals: [Hand: CBPeripheral] = [:]
    private var handFor: [UUID: Hand] = [:]

    /// Hand connections are established ONE AT A TIME (single in-flight) so we never pile
    /// up pending connects and exhaust the pool.
    private var connectQueue: [CBPeripheral] = []
    private var inFlight: CBPeripheral?
    private var pendingSince: [UUID: Date] = [:]
    private let connectTimeout: TimeInterval = 8

    /// Pedal broadcast tracking: last press counter seen, and when last seen.
    private var lastPedalCount: [Pedal: UInt8] = [:]
    private var lastPedalSeen: [Pedal: Date] = [:]
    private let pedalStaleAfter: TimeInterval = 5

    private var watchdog: Timer?

    override init() {
        super.init()
        left.onXrayToggle  = { [weak self] in self?.toggleXray() }
        right.onXrayToggle = { [weak self] in self?.toggleXray() }
        central = CBCentralManager(delegate: self, queue: nil)
        watchdog = Timer.scheduledTimer(withTimeInterval: 3, repeats: true) { [weak self] _ in
            self?.maintenanceTick()
        }
    }

    deinit { watchdog?.invalidate() }

    // MARK: Controls
    func toggleXray() {
        xrayOn.toggle()
        log("X-ray \(xrayOn ? "ON" : "OFF")")
    }

    func captureXray() {
        captureCount += 1
        log("X-ray capture #\(captureCount)")
        onXrayCapture?()
    }

    func state(for hand: Hand) -> TrackerState { hand == .left ? left : right }
    func isConnected(_ pedal: Pedal) -> Bool { pedalConnected[pedal] ?? false }

    func recenter(_ hand: Hand) { state(for: hand).recenter() }
    func recenterAll() { Hand.allCases.forEach { state(for: $0).recenter() } }

    func disconnect(_ hand: Hand) {
        if let p = peripherals[hand] { central.cancelPeripheralConnection(p) }
    }
    func disconnectAll() { Hand.allCases.forEach(disconnect) }

    /// Scanning runs CONTINUOUSLY (never stopped): we need a steady stream of pedal
    /// advertisements to catch presses, and it also re-discovers a hand that drops.
    /// Duplicates are allowed so repeated pedal ads (and post-scan-response names) arrive.
    func startScan() {
        guard central.state == .poweredOn else { return }
        for hand in Hand.allCases where peripherals[hand] == nil {
            state(for: hand).setConnection(.scanning)
        }
        central.scanForPeripherals(withServices: [Self.serviceUUID],
                                   options: [CBCentralManagerScanOptionAllowDuplicatesKey: true])
    }

    // MARK: Hand connection queue
    private func isReady(_ id: UUID) -> Bool {
        if let hand = handFor[id] { return state(for: hand).connection == .connected }
        return false
    }

    private func peripheral(for id: UUID) -> CBPeripheral? {
        peripherals.values.first { $0.identifier == id }
    }

    private func enqueueConnect(_ p: CBPeripheral) {
        if !connectQueue.contains(where: { $0.identifier == p.identifier }) {
            connectQueue.append(p)
        }
        pumpConnectQueue()
    }

    private func pumpConnectQueue() {
        guard inFlight == nil, bluetoothReady else { return }
        while let next = connectQueue.first {
            connectQueue.removeFirst()
            if isReady(next.identifier) { continue }
            inFlight = next
            pendingSince[next.identifier] = Date()
            log("\(roleName(next)) — connecting…")
            central.connect(next)
            return
        }
    }

    private func forget(_ peripheral: CBPeripheral) {
        let id = peripheral.identifier
        if let hand = handFor[id] {
            peripherals[hand] = nil; handFor[id] = nil
            state(for: hand).setConnection(.scanning)
        }
        pendingSince[id] = nil
        connectQueue.removeAll { $0.identifier == id }
        if inFlight?.identifier == id { inFlight = nil }
    }

    private func cleanup(_ peripheral: CBPeripheral, newState: ConnectionState) {
        pendingSince[peripheral.identifier] = nil
        connectQueue.removeAll { $0.identifier == peripheral.identifier }
        if inFlight?.identifier == peripheral.identifier { inFlight = nil }
        guard let hand = handFor[peripheral.identifier] else { return }
        state(for: hand).setConnection(newState)
        peripherals[hand] = nil
        handFor[peripheral.identifier] = nil
    }

    // MARK: Pedals (connectionless)
    /// Handle one pedal advertisement: mark it seen, and if its broadcast press counter
    /// changed since last time, fire the pedal's event. First sighting only baselines.
    private func handlePedalAd(_ pedal: Pedal, _ advertisementData: [String: Any]) {
        let firstSighting = (pedalConnected[pedal] != true)
        pedalConnected[pedal] = true
        lastPedalSeen[pedal] = Date()

        guard let mfg = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data,
              mfg.count >= 3 else { return }               // [0xFF,0xFF,count]
        let count = mfg[mfg.startIndex + 2]

        if firstSighting { log("\(pedal.rawValue) detected (broadcast)") }

        if let last = lastPedalCount[pedal], count != last {
            switch pedal {
            case .leftFoot:  toggleXray()
            case .rightFoot: captureXray()
            }
        }
        lastPedalCount[pedal] = count
    }

    // MARK: Watchdog + heartbeat
    private func maintenanceTick() {
        guard bluetoothReady else { return }
        let now = Date()

        // Kick any hand connection stuck past the timeout.
        let stuck = pendingSince
            .filter { now.timeIntervalSince($0.value) > connectTimeout && !isReady($0.key) }
            .keys
        for id in stuck {
            guard let p = peripheral(for: id) else { pendingSince[id] = nil; continue }
            log("\(roleName(p)) stuck connecting (>\(Int(connectTimeout))s) — retrying")
            central.cancelPeripheralConnection(p)
            forget(p)
        }
        if !stuck.isEmpty { pumpConnectQueue() }

        // Expire pedals we haven't heard broadcast recently.
        for pedal in Pedal.allCases {
            if pedalConnected[pedal] == true,
               let seen = lastPedalSeen[pedal], now.timeIntervalSince(seen) > pedalStaleAfter {
                pedalConnected[pedal] = false
                log("\(pedal.rawValue) not heard for \(Int(pedalStaleAfter))s — marking offline")
            }
        }

        // Report what's still missing.
        var missing: [String] = []
        for hand in Hand.allCases where !(peripherals[hand].map { isReady($0.identifier) } ?? false) {
            missing.append(hand.rawValue)
        }
        for pedal in Pedal.allCases where !(pedalConnected[pedal] ?? false) {
            missing.append(pedal.rawValue)
        }
        if missing.isEmpty {
            // everything present — stay quiet
        } else {
            log("waiting on: \(missing.joined(separator: ", "))")
        }
    }

    /// Human-readable CoreBluetooth failure reason for the log.
    private func reason(_ error: Error?) -> String {
        guard let error else { return "no error given" }
        let ns = error as NSError
        return "\(error.localizedDescription) [\(ns.domain) \(ns.code)]"
    }
}

// MARK: - CBCentralManagerDelegate
extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            bluetoothReady = true
            log("Bluetooth on — scanning (hands connect, pedals broadcast)")
            startScan()
        case .poweredOff:
            bluetoothReady = false
            log("Bluetooth is OFF")
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
        let advName = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? peripheral.name

        // Pedals: read from the advertisement, never connect.
        if let pedal = Pedal.from(advertisedName: advName) {
            handlePedalAd(pedal, advertisementData)
            return
        }

        // Hands: connect (queued, one at a time).
        guard let hand = Hand.from(advertisedName: advName) else { return }
        guard peripherals[hand] == nil else { return }
        peripherals[hand] = peripheral
        handFor[peripheral.identifier] = hand
        peripheral.delegate = self
        state(for: hand).setConnection(.connecting)
        log("found \(hand.rawValue) (\(RSSI) dBm) — queued")
        enqueueConnect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        log("\(roleName(peripheral)) link established — discovering services")
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral, error: Error?) {
        log("FAILED to connect: \(roleName(peripheral)) — \(reason(error))")
        cleanup(peripheral, newState: .disconnected)
        pumpConnectQueue()
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        log("disconnected: \(roleName(peripheral)) — \(reason(error)) — will reconnect")
        cleanup(peripheral, newState: .disconnected)
        pumpConnectQueue()
    }
}

// MARK: - CBPeripheralDelegate (hands only)
extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error {
            log("\(roleName(peripheral)) service discovery ERROR: \(error.localizedDescription)")
            return
        }
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
            log("\(roleName(peripheral)) has NO matching service — power-cycle the board")
            return
        }
        peripheral.discoverCharacteristics([Self.orientationCharUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error {
            log("\(roleName(peripheral)) characteristic discovery ERROR: \(error.localizedDescription)")
            return
        }
        guard let ch = service.characteristics?.first(where: { $0.uuid == Self.orientationCharUUID }) else {
            log("\(roleName(peripheral)) service has NO orientation characteristic — power-cycle the board")
            return
        }
        peripheral.setNotifyValue(true, for: ch)
        pendingSince[peripheral.identifier] = nil
        if inFlight?.identifier == peripheral.identifier { inFlight = nil }
        if let hand = handFor[peripheral.identifier] {
            state(for: hand).setConnection(.connected)
        }
        log("\(roleName(peripheral)) ready (receiving data)")
        pumpConnectQueue()   // start the other hand if it's queued
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value,
              let hand = handFor[peripheral.identifier] else { return }
        state(for: hand).ingest(data)
    }
}

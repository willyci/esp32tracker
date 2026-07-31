import SwiftUI

@main
struct ESP32TrackerApp: App {
    @StateObject private var ble: BLEManager
    @StateObject private var sim: SimulationModel
    @StateObject private var captures: CaptureStore

    init() {
        let ble = BLEManager()
        let sim = SimulationModel(ble: ble)
        let captures = CaptureStore()
        // Right foot pedal (or the UI's capture button) → freeze the sim state and shoot
        // BOTH photos: the X-ray monitor image and the 180° room view.
        ble.onXrayCapture = { [weak ble] in
            guard let ble else { return }
            sim.recordSnapshot(xrayOn: ble.xrayOn)
            captures.capture(sim.captureScene(index: ble.captureCount,
                                              xrayOn: ble.xrayOn,
                                              dsaActive: ble.dsaActive,
                                              dsaRuns: ble.dsaRuns),
                             log: { [weak ble] msg in ble?.log(msg) })
        }
        _ble = StateObject(wrappedValue: ble)
        _sim = StateObject(wrappedValue: sim)
        _captures = StateObject(wrappedValue: captures)
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(ble)
                .environmentObject(sim)
                .environmentObject(captures)
        }
        // A plain (flat) window is the initial scene. A volumetric WindowGroup as the
        // *only* scene crashes at launch ("no scenes ... match this role"), because the
        // system's launch scene request is for a standard window. ContentView is a flat
        // HStack anyway; the RealityView still renders the box in 3D inside it.
        .windowStyle(.automatic)
        .defaultSize(width: 1560, height: 900)

        // Catheter/wire simulation — immersive so ARKit hand tracking can run
        // (hand anchors are only delivered inside an immersive space).
        ImmersiveSpace(id: "simulation") {
            SimulationView(sim: sim, ble: ble)
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed)
    }
}

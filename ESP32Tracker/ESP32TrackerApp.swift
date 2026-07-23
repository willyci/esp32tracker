import SwiftUI

@main
struct ESP32TrackerApp: App {
    @StateObject private var ble: BLEManager
    @StateObject private var sim: SimulationModel

    init() {
        let ble = BLEManager()
        let sim = SimulationModel(ble: ble)
        // Right foot pedal (or the UI's capture button) → freeze the sim state.
        ble.onXrayCapture = { [weak ble] in
            guard let ble else { return }
            sim.recordSnapshot(xrayOn: ble.xrayOn)
        }
        _ble = StateObject(wrappedValue: ble)
        _sim = StateObject(wrappedValue: sim)
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(ble)
                .environmentObject(sim)
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

import SwiftUI

@main
struct ESP32TrackerApp: App {
    @StateObject private var ble = BLEManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(ble)
        }
        // A plain (flat) window is the initial scene. A volumetric WindowGroup as the
        // *only* scene crashes at launch ("no scenes ... match this role"), because the
        // system's launch scene request is for a standard window. ContentView is a flat
        // HStack anyway; the RealityView still renders the box in 3D inside it.
        .windowStyle(.automatic)
        .defaultSize(width: 900, height: 560)
    }
}

import SwiftUI

struct ContentView: View {
    @EnvironmentObject var ble: BLEManager

    var body: some View {
        HStack(alignment: .top, spacing: 28) {
            OrientationScene()
                .frame(minWidth: 360, minHeight: 360)
                .glassBackgroundEffect()

            dataPanel
                .frame(width: 300)
        }
        .padding(28)
    }

    private var dataPanel: some View {
        VStack(alignment: .leading, spacing: 14) {
            Text("ESP32 Tracker")
                .font(.title2.bold())

            Label(ble.connection.rawValue,
                  systemImage: ble.connection == .connected ? "checkmark.circle.fill" : "circle")
                .foregroundStyle(ble.connection == .connected ? .green : .secondary)

            Divider()

            Text("Quaternion").font(.headline)
            row("w", ble.quaternion.real)
            row("x", ble.quaternion.imag.x)
            row("y", ble.quaternion.imag.y)
            row("z", ble.quaternion.imag.z)

            Divider()

            Text("Accel (m/s²)").font(.headline)
            row("x", ble.accel.x)
            row("y", ble.accel.y)
            row("z", ble.accel.z)

            Divider()

            Text("Calibration: \(ble.calibration)/3")
                .foregroundStyle(.secondary)

            Divider()

            HStack(spacing: 12) {
                Button("Scan") { ble.startScan() }
                    .buttonStyle(.borderedProminent)
                Button("Re-center") { ble.recenter() }
            }
            Button("Disconnect", role: .destructive) { ble.disconnect() }

            Spacer()
        }
    }

    private func row(_ label: String, _ value: Float) -> some View {
        HStack {
            Text(label)
                .foregroundStyle(.secondary)
                .frame(width: 28, alignment: .leading)
            Text(String(format: "%+.3f", value))
                .monospacedDigit()
            Spacer()
        }
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environmentObject(BLEManager())
}

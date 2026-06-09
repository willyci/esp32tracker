import SwiftUI

struct ContentView: View {
    @EnvironmentObject var ble: BLEManager

    var body: some View {
        HStack(alignment: .top, spacing: 28) {
            OrientationScene(left: ble.left, right: ble.right)
                .frame(minWidth: 420, minHeight: 380)
                .glassBackgroundEffect()

            VStack(spacing: 18) {
                TrackerPanel(tracker: ble.left)
                TrackerPanel(tracker: ble.right)

                HStack(spacing: 12) {
                    Button("Scan") { ble.startScan() }
                        .buttonStyle(.borderedProminent)
                    Button("Re-center both") { ble.recenterAll() }
                }
                Button("Disconnect both", role: .destructive) { ble.disconnectAll() }

                Spacer()
            }
            .frame(width: 320)
        }
        .padding(28)
    }
}

/// One panel per hand: connection state, quaternion, accel, calibration, re-center.
struct TrackerPanel: View {
    @ObservedObject var tracker: TrackerState

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("\(tracker.hand.label) Hand").font(.title3.bold())
                Spacer()
                Label(tracker.connection.rawValue,
                      systemImage: tracker.connection == .connected ? "checkmark.circle.fill" : "circle")
                    .font(.caption)
                    .foregroundStyle(tracker.connection == .connected ? .green : .secondary)
            }

            Text("Quaternion").font(.subheadline.bold())
            row("w", tracker.quaternion.real)
            row("x", tracker.quaternion.imag.x)
            row("y", tracker.quaternion.imag.y)
            row("z", tracker.quaternion.imag.z)

            Text("Accel (m/s²)").font(.subheadline.bold())
            row("x", tracker.accel.x)
            row("y", tracker.accel.y)
            row("z", tracker.accel.z)

            HStack {
                Text("Calibration: \(tracker.calibration)/3")
                    .font(.caption).foregroundStyle(.secondary)
                Spacer()
                Button("Re-center") { tracker.recenter() }
                    .controlSize(.small)
            }
        }
        .padding(16)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private func row(_ label: String, _ value: Float) -> some View {
        HStack {
            Text(label)
                .foregroundStyle(.secondary)
                .frame(width: 22, alignment: .leading)
            Text(String(format: "%+.3f", value))
                .monospacedDigit()
                .font(.callout)
            Spacer()
        }
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environmentObject(BLEManager())
}

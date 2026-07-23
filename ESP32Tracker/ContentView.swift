import SwiftUI

struct ContentView: View {
    @EnvironmentObject var ble: BLEManager
    @EnvironmentObject var sim: SimulationModel
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @State private var simOpen = false
    @State private var flashOpacity: Double = 0   // white flash on each X-ray capture

    var body: some View {
        HStack(alignment: .top, spacing: 28) {
            // Left column: 3D viewport on top, live event log filling the space below.
            VStack(spacing: 18) {
                OrientationScene(left: ble.left, right: ble.right, xrayOn: ble.xrayOn)
                    .frame(minWidth: 760, minHeight: 380)   // wide enough for both cubes + spin room
                    .glassBackgroundEffect()

                LogPanel(ble: ble)
                    .frame(maxHeight: .infinity)
            }

            VStack(spacing: 18) {
                // Shared X-ray state — either hand's hardware button (or a tap here) flips it.
                Button {
                    ble.toggleXray()
                } label: {
                    Label(ble.xrayOn ? "X-RAY: ON" : "X-RAY: OFF",
                          systemImage: ble.xrayOn ? "eye" : "eye.slash")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(ble.xrayOn ? .green : .gray)

                // Capture (right foot pedal, or tap here): freezes the sim state + flashes.
                Button {
                    ble.captureXray()
                } label: {
                    Label("Capture X-ray  (\(ble.captureCount))", systemImage: "camera.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)

                // Foot pedals: left toggles X-ray, right captures.
                HStack(spacing: 14) {
                    ForEach(Pedal.allCases, id: \.self) { pedal in
                        HStack(spacing: 6) {
                            Circle()
                                .fill(ble.isConnected(pedal) ? .green : .gray)
                                .frame(width: 8, height: 8)
                            Text(pedal.label)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                    }
                    Spacer()
                    if let last = sim.snapshots.last {
                        Text("last: \(last.takenAt.formatted(date: .omitted, time: .standard)) · C \(String(format: "%.1f", last.catheter.insertion * 100))cm · W \(String(format: "%.1f", last.wire.insertion * 100))cm")
                            .font(.caption2)
                            .foregroundStyle(.secondary)
                            .lineLimit(1)
                    }
                }

                // Catheter/wire simulation: grab = SoftPot touch, twist = slide + roll,
                // insertion = hand movement (needs the immersive space for hand tracking).
                Button {
                    Task {
                        if simOpen {
                            await dismissImmersiveSpace()
                            simOpen = false
                        } else if await openImmersiveSpace(id: "simulation") == .opened {
                            simOpen = true
                        }
                    }
                } label: {
                    Label(simOpen ? "Exit Simulation" : "Enter Simulation",
                          systemImage: "hand.point.up.left.and.text")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)

                SimToolRow(name: "Catheter (L)", color: .teal, tool: sim.catheter)
                SimToolRow(name: "Wire (R)", color: .purple, tool: sim.wire)

                // Side by side so the whole column fits on screen without resizing.
                HStack(alignment: .top, spacing: 18) {
                    TrackerPanel(tracker: ble.left, color: .teal)      // matches the teal cube
                    TrackerPanel(tracker: ble.right, color: .purple)   // matches the purple cube
                }

                HStack(spacing: 12) {
                    Button("Scan") { ble.startScan() }
                        .buttonStyle(.borderedProminent)
                    Button("Re-center both") { ble.recenterAll() }
                }
                Button("Disconnect both", role: .destructive) { ble.disconnectAll() }

                Spacer()
            }
            .frame(width: 640)
        }
        .padding(28)
        .overlay {
            // Camera-flash acknowledging each capture (same idea as the PC dashboard).
            Color.white
                .opacity(flashOpacity)
                .allowsHitTesting(false)
        }
        .onChange(of: ble.captureCount) {
            flashOpacity = 0.8
            withAnimation(.easeOut(duration: 0.35)) { flashOpacity = 0 }
        }
        .task {
            // Open the catheter/wire simulation immediately on launch — no extra step.
            if !simOpen, await openImmersiveSpace(id: "simulation") == .opened {
                simOpen = true
            }
        }
    }
}

/// Live BLE event log — the app usually runs untethered, so this replaces the
/// Xcode console: discovery, connections, disconnections, X-ray and captures.
struct LogPanel: View {
    @ObservedObject var ble: BLEManager

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Log").font(.subheadline.bold())
            ScrollViewReader { proxy in
                ScrollView {
                    VStack(alignment: .leading, spacing: 2) {
                        ForEach(ble.logEntries) { entry in
                            Text("\(entry.time.formatted(date: .omitted, time: .standard))  \(entry.message)")
                                .font(.caption.monospaced())
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .id(entry.id)
                        }
                    }
                }
                .onChange(of: ble.logEntries.count) {
                    // Keep the newest line in view.
                    if let last = ble.logEntries.last {
                        proxy.scrollTo(last.id, anchor: .bottom)
                    }
                }
            }
        }
        .padding(14)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }
}

/// One line per simulated tool: grab state, insertion depth, accumulated twist.
struct SimToolRow: View {
    let name: String
    let color: Color
    let tool: SimulationModel.Tool

    var body: some View {
        HStack(spacing: 8) {
            Circle().fill(color).frame(width: 10, height: 10)
            Text(name).font(.callout.bold())
            Spacer()
            Text(tool.grabbed ? "grabbed" : "released")
                .font(.caption)
                .foregroundStyle(tool.grabbed ? .green : .secondary)
            Text(String(format: "%.1f cm", tool.insertion * 100))
                .font(.caption).monospacedDigit()
            Text(String(format: "%+.0f°", tool.twist * 180 / .pi))
                .font(.caption).monospacedDigit()
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
}

/// One panel per hand: connection state, quaternion, accel, calibration, re-center.
struct TrackerPanel: View {
    @ObservedObject var tracker: TrackerState
    let color: Color

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Circle().fill(color).frame(width: 12, height: 12)
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

            Text("SoftPot").font(.subheadline.bold())
            if tracker.touchActive {
                Text("start \(tracker.touchStart) · now \(tracker.touchCurrent) · Δ \(tracker.touchDelta, format: .number.sign(strategy: .always()))")
                    .font(.callout).monospacedDigit()
            } else {
                Text("no touch").font(.callout).foregroundStyle(.secondary)
            }
            SoftPotBar(start: tracker.touchStart, current: tracker.touchCurrent)

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

    /// The touch strip: green dot = where the touch began, orange dot = finger now.
    /// Both hide when the strip isn't touched (positions are 0).
    private struct SoftPotBar: View {
        let start: UInt8
        let current: UInt8

        var body: some View {
            GeometryReader { geo in
                let usable = geo.size.width - 12   // keep the dots inside the track
                ZStack(alignment: .leading) {
                    Capsule().fill(.quaternary).frame(height: 6)
                    if current > 0 {
                        Circle().fill(.green).frame(width: 12, height: 12)
                            .offset(x: CGFloat(start) / 255 * usable)
                        Circle().fill(.orange).frame(width: 12, height: 12)
                            .offset(x: CGFloat(current) / 255 * usable)
                    }
                }
                .frame(width: geo.size.width, height: geo.size.height)
            }
            .frame(height: 14)
        }
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
    let ble = BLEManager()
    ContentView()
        .environmentObject(ble)
        .environmentObject(SimulationModel(ble: ble))
}

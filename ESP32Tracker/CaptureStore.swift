import SwiftUI
import UIKit

/// Everything the two capture images need, frozen at the instant the pedal fired.
/// Passed by value so rendering can never race the live simulation.
struct CaptureScene {
    let index: Int
    let takenAt: Date
    let catheter: SimulationModel.Tool
    let wire: SimulationModel.Tool
    let xrayOn: Bool
    let dsaActive: Bool
    let dsaRuns: Int
    let catheterMax: Float
    let wireMax: Float
}

/// One capture = TWO photos, the way a real suite records a moment:
///   1. the **X-ray monitor** — the fluoro image the operator is looking at
///   2. the **180° room view** — the wide shot of the room/table around them
///
/// Both are rendered off-screen from SwiftUI (`ImageRenderer`) and written to
/// Documents/Captures as PNGs, so a session leaves a reviewable pair per press.
///
/// NOTE on the room view: visionOS only lets an app read the passthrough cameras with
/// Apple's *Enterprise* "Main Camera Access" entitlement, which this app does not have —
/// so photo 2 cannot be a photograph of the physical room. It is a 180° wide-field render
/// of the simulated suite (table, C-arm, monitor, tools) from the operator's viewpoint.
@MainActor
final class CaptureStore: ObservableObject {

    struct Capture: Identifiable {
        let id = UUID()
        let scene: CaptureScene
        let xray: UIImage?
        let room: UIImage?
        let xrayURL: URL?
        let roomURL: URL?
    }

    @Published private(set) var captures: [Capture] = []
    var latest: Capture? { captures.last }

    /// Where the PNG pairs land. Per-session subfolder keeps runs separate.
    private lazy var folder: URL? = {
        guard let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first
        else { return nil }
        let dir = docs.appendingPathComponent("Captures", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }()

    /// Render + persist both photos for one capture. Returns immediately after rendering;
    /// images are small (2× of ~900 pt), so this is a few milliseconds on device.
    func capture(_ scene: CaptureScene, log: ((String) -> Void)? = nil) {
        let xrayImage = render(XRayMonitorView(scene: scene), size: CGSize(width: 900, height: 700))
        let roomImage = render(RoomView180(scene: scene), size: CGSize(width: 1000, height: 560))

        let stamp = Self.stampFormatter.string(from: scene.takenAt)
        let xrayURL = write(xrayImage, name: "capture-\(scene.index)-\(stamp)-xray.png")
        let roomURL = write(roomImage, name: "capture-\(scene.index)-\(stamp)-room180.png")

        captures.append(Capture(scene: scene, xray: xrayImage, room: roomImage,
                                xrayURL: xrayURL, roomURL: roomURL))
        // Keep memory bounded during long sessions; the PNGs on disk are the archive.
        if captures.count > 40 { captures.removeFirst(captures.count - 40) }

        log?("capture #\(scene.index): X-ray monitor + 180° room view saved")
    }

    private func render<V: View>(_ view: V, size: CGSize) -> UIImage? {
        let renderer = ImageRenderer(content: view.frame(width: size.width, height: size.height))
        renderer.scale = 2
        return renderer.uiImage
    }

    private func write(_ image: UIImage?, name: String) -> URL? {
        guard let image, let data = image.pngData(), let folder else { return nil }
        let url = folder.appendingPathComponent(name)
        do { try data.write(to: url); return url } catch { return nil }
    }

    private static let stampFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd-HHmmss"
        return f
    }()
}

// MARK: - Photo 1: the X-ray monitor

/// A fluoro-style monitor frame: dark field, the vessel running across it, the catheter and
/// guidewire as bright radiopaque lines at their current depths, contrast filling the vessel
/// during a DSA run, and the burned-in corner annotations a real monitor carries.
struct XRayMonitorView: View {
    let scene: CaptureScene

    var body: some View {
        ZStack {
            Color.black
            GeometryReader { geo in
                let w = geo.size.width, h = geo.size.height
                let y = h * 0.5
                let entryX = w * 0.90          // skin entry on the right
                let vesselEnd = w * 0.12       // vessel runs off to the left
                let span = entryX - vesselEnd

                // Vessel: faint when empty, dark and full during a contrast run.
                Path { p in
                    p.move(to: CGPoint(x: entryX, y: y))
                    p.addLine(to: CGPoint(x: vesselEnd, y: y))
                }
                .stroke(scene.dsaActive ? Color.white.opacity(0.85) : Color.white.opacity(0.16),
                        style: StrokeStyle(lineWidth: scene.dsaActive ? 26 : 22, lineCap: .round))

                // Catheter — thicker, brighter; length = its insertion depth.
                Path { p in
                    p.move(to: CGPoint(x: entryX, y: y))
                    p.addLine(to: CGPoint(x: entryX - span * CGFloat(fraction(scene.catheter.insertion,
                                                                              scene.catheterMax)), y: y))
                }
                .stroke(Color.white.opacity(0.95), style: StrokeStyle(lineWidth: 9, lineCap: .round))

                // Guidewire — thinner, runs ahead of the catheter.
                Path { p in
                    p.move(to: CGPoint(x: entryX, y: y))
                    p.addLine(to: CGPoint(x: entryX - span * CGFloat(fraction(scene.wire.insertion,
                                                                             scene.wireMax)), y: y))
                }
                .stroke(Color.white, style: StrokeStyle(lineWidth: 4, lineCap: .round))
            }

            // Burned-in annotations, like a real fluoro monitor.
            VStack {
                HStack(alignment: .top) {
                    VStack(alignment: .leading, spacing: 4) {
                        Text(scene.dsaActive ? "DSA" : (scene.xrayOn ? "FLUORO" : "STANDBY"))
                            .font(.system(size: 26, weight: .bold, design: .monospaced))
                            .foregroundStyle(scene.dsaActive ? .orange : (scene.xrayOn ? .green : .gray))
                        Text("RUN \(scene.dsaRuns)")
                            .font(.system(size: 15, design: .monospaced))
                            .foregroundStyle(.white.opacity(0.65))
                    }
                    Spacer()
                    VStack(alignment: .trailing, spacing: 4) {
                        Text("IMG \(scene.index)")
                        Text(scene.takenAt.formatted(date: .numeric, time: .standard))
                    }
                    .font(.system(size: 15, design: .monospaced))
                    .foregroundStyle(.white.opacity(0.75))
                }
                Spacer()
                HStack {
                    Text(String(format: "CATH %.1f cm", scene.catheter.insertion * 100))
                    Spacer()
                    Text(String(format: "WIRE %.1f cm", scene.wire.insertion * 100))
                }
                .font(.system(size: 18, weight: .semibold, design: .monospaced))
                .foregroundStyle(.white.opacity(0.85))
            }
            .padding(26)
        }
    }

    private func fraction(_ value: Float, _ maxValue: Float) -> Float {
        maxValue > 0 ? min(1, max(0, value / maxValue)) : 0
    }
}

// MARK: - Photo 2: the 180° room view

/// A 180° wide-field ("fisheye") record of the suite around the operator: the circular frame
/// a wide lens produces, with the table, patient, C-arm, ceiling monitor and the tools in
/// hand laid out across it, plus the imaging state at that instant.
///
/// This renders the SIMULATED room — see the note on `CaptureStore` for why the passthrough
/// cameras aren't available to a non-enterprise app.
struct RoomView180: View {
    let scene: CaptureScene

    var body: some View {
        ZStack {
            Color(white: 0.06)
            GeometryReader { geo in
                let w = geo.size.width, h = geo.size.height
                let cx = w / 2, cy = h * 0.62
                let r = min(w * 0.47, h * 0.86)

                // The circular 180° field.
                Circle()
                    .fill(Color(white: 0.13))
                    .frame(width: r * 2, height: r * 2)
                    .position(x: cx, y: cy)
                Circle()
                    .strokeBorder(Color.white.opacity(0.25), lineWidth: 2)
                    .frame(width: r * 2, height: r * 2)
                    .position(x: cx, y: cy)

                // Floor horizon + a couple of depth arcs: reads as a wide lens.
                ForEach([0.45, 0.72, 1.0], id: \.self) { f in
                    Circle()
                        .trim(from: 0.0, to: 0.5)
                        .stroke(Color.white.opacity(0.10), lineWidth: 1.5)
                        .rotationEffect(.degrees(180))
                        .frame(width: r * 2 * f, height: r * 2 * f)
                        .position(x: cx, y: cy)
                }

                // Ceiling-hung X-ray monitor (top of the field), lit when imaging.
                RoundedRectangle(cornerRadius: 6)
                    .fill(scene.xrayOn ? Color.white.opacity(0.85) : Color.white.opacity(0.18))
                    .frame(width: r * 0.42, height: r * 0.26)
                    .position(x: cx, y: cy - r * 0.66)
                Text(scene.dsaActive ? "DSA" : (scene.xrayOn ? "FLUORO" : "OFF"))
                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                    .foregroundStyle(scene.xrayOn ? .black : .white.opacity(0.5))
                    .position(x: cx, y: cy - r * 0.66)

                // C-arm arch straddling the table.
                Circle()
                    .trim(from: 0.58, to: 0.92)
                    .stroke(Color.white.opacity(0.35),
                            style: StrokeStyle(lineWidth: r * 0.055, lineCap: .round))
                    .frame(width: r * 1.15, height: r * 1.15)
                    .position(x: cx, y: cy - r * 0.10)

                // Table + patient, curved by the wide lens.
                Capsule()
                    .fill(Color.white.opacity(0.22))
                    .frame(width: r * 1.5, height: r * 0.17)
                    .position(x: cx, y: cy + r * 0.12)
                Capsule()
                    .fill(Color.white.opacity(0.40))
                    .frame(width: r * 1.15, height: r * 0.11)
                    .position(x: cx, y: cy + r * 0.10)

                // The two tools in the operator's hands, at the access site (right of table).
                toolMark(label: "CATH", cm: scene.catheter.insertion * 100,
                         held: scene.catheter.grabbed, color: .teal)
                    .position(x: cx + r * 0.52, y: cy + r * 0.34)
                toolMark(label: "WIRE", cm: scene.wire.insertion * 100,
                         held: scene.wire.grabbed, color: .purple)
                    .position(x: cx + r * 0.52, y: cy + r * 0.52)
            }

            // Frame annotations.
            VStack {
                HStack {
                    Text("180° ROOM VIEW")
                        .font(.system(size: 20, weight: .bold, design: .monospaced))
                        .foregroundStyle(.white.opacity(0.85))
                    Spacer()
                    Text("IMG \(scene.index) · \(scene.takenAt.formatted(date: .numeric, time: .standard))")
                        .font(.system(size: 14, design: .monospaced))
                        .foregroundStyle(.white.opacity(0.7))
                }
                Spacer()
                Text("simulated suite — passthrough camera requires visionOS enterprise entitlement")
                    .font(.system(size: 12, design: .monospaced))
                    .foregroundStyle(.white.opacity(0.35))
            }
            .padding(22)
        }
    }

    private func toolMark(label: String, cm: Float, held: Bool, color: Color) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(held ? color : color.opacity(0.35))
                .frame(width: 12, height: 12)
            Text("\(label) \(String(format: "%.0f", cm))cm\(held ? " ●" : "")")
                .font(.system(size: 13, weight: .semibold, design: .monospaced))
                .foregroundStyle(.white.opacity(0.85))
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 5)
        .background(Color.black.opacity(0.45), in: Capsule())
    }
}

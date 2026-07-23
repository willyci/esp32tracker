import SwiftUI
import RealityKit
import ARKit
import UIKit

/// Immersive simulation scene: a translucent vessel with the catheter (teal checker,
/// LEFT tracker) and guidewire (purple checker, RIGHT tracker) sliding coaxially
/// through a hub ring — the checkerboard makes twist plainly visible.
///
/// Input (see SimulationModel): SoftPot touch = grab · strip slide + tracker roll =
/// twist · grabbed hand's world-X movement (ARKit hand tracking) = insertion.
/// X-ray ON renders the catheter see-through so the wire inside shows.
struct SimulationView: View {
    @ObservedObject var sim: SimulationModel
    @ObservedObject var ble: BLEManager

    static let catheterLength: Float = 0.60
    static let wireLength: Float = 0.65

    var body: some View {
        RealityView { content, attachments in
            let root = Entity()
            root.name = "root"
            root.position = [0, 1.0, -0.6]   // chest height, in front of the user

            root.addChild(Self.makeVessel())
            root.addChild(Self.makeHubRing())
            root.addChild(Self.makeRod(name: "catheter", length: Self.catheterLength,
                                       radius: 0.012, colorA: .systemTeal, colorB: .white))
            root.addChild(Self.makeRod(name: "wire", length: Self.wireLength,
                                       radius: 0.005, colorA: .systemPurple, colorB: .white))

            // Floating device-status panel above the vessel — visible while immersed,
            // when the playground window (with its own indicators) may be out of view.
            if let status = attachments.entity(for: "deviceStatus") {
                status.position = [0, 0.28, 0]
                root.addChild(status)
            }
            content.add(root)
        } update: { content, _ in
            guard let root = content.entities.first(where: { $0.name == "root" }) else { return }
            if let c = root.findEntity(named: "catheter") {
                c.position.x = Self.catheterLength / 2 - sim.catheter.insertion
                c.orientation = simd_quatf(angle: sim.catheter.twist, axis: [1, 0, 0])
                // X-ray: see through the catheter to the wire running inside it
                c.components.set(OpacityComponent(opacity: ble.xrayOn ? 0.35 : 1.0))
            }
            if let w = root.findEntity(named: "wire") {
                w.position.x = Self.wireLength / 2 - sim.wire.insertion
                w.orientation = simd_quatf(angle: sim.wire.twist, axis: [1, 0, 0])
            }
        } attachments: {
            Attachment(id: "deviceStatus") {
                deviceStatusPanel
            }
        }
        .task { await trackHands() }
    }

    /// Connection dots for all four boards (green = connected, red = not).
    private var deviceStatusPanel: some View {
        HStack(spacing: 18) {
            statusDot("L hand", ble.left.connection == .connected)
            statusDot("R hand", ble.right.connection == .connected)
            ForEach(Pedal.allCases, id: \.self) { pedal in
                statusDot(pedal.label, ble.isConnected(pedal))
            }
        }
        .font(.callout)
        .padding(.horizontal, 18)
        .padding(.vertical, 10)
        .glassBackgroundEffect()
    }

    private func statusDot(_ label: String, _ connected: Bool) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(connected ? Color.green : Color.red)
                .frame(width: 10, height: 10)
            Text(label)
        }
    }

    /// ARKit hand tracking: each hand's world-X movement drives its grabbed tool.
    /// Runs while the immersive space is open; cancelled automatically on dismiss.
    private func trackHands() async {
        guard HandTrackingProvider.isSupported else { return }
        let session = ARKitSession()
        let provider = HandTrackingProvider()
        do { try await session.run([provider]) } catch { return }

        var lastX: [Hand: Float] = [:]
        for await update in provider.anchorUpdates {
            let anchor = update.anchor
            let hand: Hand = anchor.chirality == .left ? .left : .right
            guard anchor.isTracked else { lastX[hand] = nil; continue }
            let x = anchor.originFromAnchorTransform.columns.3.x
            if let last = lastX[hand] { sim.handMoved(hand, deltaX: x - last) }
            lastX[hand] = x
        }
    }

    // MARK: - Entity builders

    /// A rod lying along the X axis: checkered shaft plus an orange tip pointing −X
    /// (the insertion direction). The container's origin is the rod's center, so
    /// position.x = length/2 − insertion puts the tip at −insertion.
    private static func makeRod(name: String, length: Float, radius: Float,
                                colorA: UIColor, colorB: UIColor) -> Entity {
        let container = Entity()
        container.name = name

        var material = SimpleMaterial(color: colorA, isMetallic: false)
        if let checker = checkerTexture(colorA, colorB) {
            material.color = .init(tint: .white, texture: .init(checker))
        }
        let shaft = ModelEntity(mesh: .generateCylinder(height: length, radius: radius),
                                materials: [material])
        shaft.orientation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])   // Y-cylinder → X
        container.addChild(shaft)

        let tip = ModelEntity(mesh: .generateCone(height: radius * 4, radius: radius),
                              materials: [SimpleMaterial(color: .systemOrange, isMetallic: false)])
        tip.orientation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])      // cone +Y → −X
        tip.position = [-length / 2 - radius * 2, 0, 0]
        container.addChild(tip)
        return container
    }

    /// Translucent "vessel" the tools advance into (entry at x = 0, running −X).
    private static func makeVessel() -> Entity {
        let vessel = ModelEntity(
            mesh: .generateCylinder(height: 0.7, radius: 0.018),
            materials: [SimpleMaterial(color: UIColor.systemGray.withAlphaComponent(0.25),
                                       isMetallic: false)]
        )
        vessel.orientation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        vessel.position = [-0.35, 0, 0]
        return vessel
    }

    /// Entry-point marker the rods slide through.
    private static func makeHubRing() -> Entity {
        let ring = ModelEntity(mesh: .generateCylinder(height: 0.03, radius: 0.022),
                               materials: [SimpleMaterial(color: .darkGray, isMetallic: true)])
        ring.orientation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        return ring
    }

    /// Checkerboard texture: columns wrap the circumference, rows run along the rod,
    /// so both twist AND slide read clearly.
    private static func checkerTexture(_ a: UIColor, _ b: UIColor) -> TextureResource? {
        let cols = 4, rows = 24, cell = 24
        let renderer = UIGraphicsImageRenderer(size: CGSize(width: cols * cell, height: rows * cell))
        let image = renderer.image { ctx in
            for r in 0..<rows {
                for c in 0..<cols {
                    ((r + c) % 2 == 0 ? a : b).setFill()
                    ctx.fill(CGRect(x: c * cell, y: r * cell, width: cell, height: cell))
                }
            }
        }
        guard let cg = image.cgImage else { return nil }
        return try? TextureResource.generate(from: cg, options: .init(semantic: .color))
    }
}

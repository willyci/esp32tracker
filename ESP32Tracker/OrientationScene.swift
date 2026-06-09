import SwiftUI
import RealityKit
import UIKit

/// The 3D viewport: two separate objects — left and right hand — each rotating to
/// match its own tracker. They sit at fixed points and only rotate (orientation only;
/// position tracking is out of scope — see SPEC.md).
///
/// Takes the two `TrackerState`s as @ObservedObject so the RealityView `update:`
/// closure re-runs whenever EITHER hand publishes a new sample.
struct OrientationScene: View {
    @ObservedObject var left: TrackerState
    @ObservedObject var right: TrackerState

    var body: some View {
        RealityView { content in
            let leftEntity = Self.makeTrackerEntity(name: "left", color: .systemTeal)
            leftEntity.position = [-0.18, 0, 0]          // left side of the viewport
            content.add(leftEntity)

            let rightEntity = Self.makeTrackerEntity(name: "right", color: .systemPurple)
            rightEntity.position = [0.18, 0, 0]          // right side
            content.add(rightEntity)
        } update: { content in
            if let l = content.entities.first(where: { $0.name == "left" }) {
                l.orientation = left.displayOrientation
            }
            if let r = content.entities.first(where: { $0.name == "right" }) {
                r.orientation = right.displayOrientation
            }
        }
    }

    /// Builds one tracker object. Prefers the bundled probe model; falls back to a
    /// colored box with an orange nose (so rotation — and which hand — is readable).
    private static func makeTrackerEntity(name: String, color: UIColor) -> Entity {
        let container = Entity()
        container.name = name

        if let probe = try? Entity.load(named: "probe") {
            // Normalize the model's size regardless of how the asset was authored.
            container.addChild(probe)
            let bounds = probe.visualBounds(relativeTo: container)
            let maxExtent = max(bounds.extents.x, bounds.extents.y, bounds.extents.z)
            if maxExtent > 0 {
                let s = 0.2 / maxExtent
                probe.scale = SIMD3<Float>(repeating: s)
                probe.position = -bounds.center * s
            }
            return container
        }

        // Placeholder: a flat box (per-hand color) with an orange nose marking +Z (front).
        let body = ModelEntity(
            mesh: .generateBox(width: 0.16, height: 0.035, depth: 0.09, cornerRadius: 0.004),
            materials: [SimpleMaterial(color: color, isMetallic: false)]
        )
        let nose = ModelEntity(
            mesh: .generateBox(width: 0.05, height: 0.05, depth: 0.02, cornerRadius: 0.002),
            materials: [SimpleMaterial(color: .systemOrange, isMetallic: false)]
        )
        nose.position = [0, 0, 0.055]
        body.addChild(nose)
        container.addChild(body)
        return container
    }
}

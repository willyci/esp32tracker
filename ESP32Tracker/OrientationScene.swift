import SwiftUI
import RealityKit
import UIKit

/// The 3D viewport: a single box that rotates to match the sensor.
///
/// A box with a distinct shape (flat, wider than tall) makes rotation easy to read —
/// swap in a probe model later once the orientation mapping looks right.
struct OrientationScene: View {
    @EnvironmentObject var ble: BLEManager

    var body: some View {
        RealityView { content in
            let box = Self.makeTrackerEntity()
            content.add(box)
        } update: { content in
            // Reading ble.displayOrientation here ties this closure to the published
            // updates, so the box re-orients every time a new sample arrives.
            guard let box = content.entities.first(where: { $0.name == "tracker" }) else { return }
            box.orientation = ble.displayOrientation
        }
    }

    private static func makeTrackerEntity() -> Entity {
        // Prefer the bundled probe model; fall back to the placeholder box if it's missing
        // or fails to load (e.g. before the .usdc is added to the target).
        if let probe = try? Entity.load(named: "probe") {
            // USD unit conventions vary (m vs cm), so a raw model often imports far too
            // large. Wrap it, measure its bounds, then scale to ~0.2 m on the longest
            // axis and recenter — independent of how the asset was authored.
            let container = Entity()
            container.name = "tracker"
            container.addChild(probe)

            let bounds = probe.visualBounds(relativeTo: container)
            let maxExtent = max(bounds.extents.x, bounds.extents.y, bounds.extents.z)
            if maxExtent > 0 {
                let s = 0.2 / maxExtent
                probe.scale = SIMD3<Float>(repeating: s)
                probe.position = -bounds.center * s   // recenter on the container origin
            }
            return container
        }
        return makePlaceholderBox()
    }

    /// Original placeholder: a flat box with an orange nose marking +Z (front).
    private static func makePlaceholderBox() -> ModelEntity {
        let mesh = MeshResource.generateBox(width: 0.16, height: 0.035, depth: 0.09, cornerRadius: 0.004)
        let body = ModelEntity(mesh: mesh, materials: [SimpleMaterial(color: .systemTeal, isMetallic: false)])
        body.name = "tracker"

        let nose = ModelEntity(
            mesh: .generateBox(width: 0.05, height: 0.05, depth: 0.02, cornerRadius: 0.002),
            materials: [SimpleMaterial(color: .systemOrange, isMetallic: false)]
        )
        nose.position = [0, 0, 0.055]
        body.addChild(nose)

        return body
    }
}

import SwiftUI
import RealityKit
import UIKit

/// The 3D viewport: two dice-style cubes — left (teal) and right (purple) — each
/// rotating to match its own tracker. They sit at fixed points and only rotate
/// (orientation only; position tracking is out of scope — see SPEC.md).
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

    /// Builds one tracker cube: per-hand color, with a white number on each of the
    /// six faces (dice layout — opposite faces sum to 7) so every rotation is readable.
    private static func makeTrackerEntity(name: String, color: UIColor) -> Entity {
        let container = Entity()
        container.name = name

        let size: Float = 0.12
        let cube = ModelEntity(
            mesh: .generateBox(size: size, cornerRadius: 0.005),
            materials: [SimpleMaterial(color: color, isMetallic: false)]
        )
        container.addChild(cube)

        // Each entry: number, face-center position, rotation turning +Z to face outward.
        let half = size / 2 + 0.001   // float the glyph just above the surface
        let faces: [(label: String, position: SIMD3<Float>, rotation: simd_quatf)] = [
            ("1", [0, 0,  half], simd_quatf(angle: 0,        axis: [0, 1, 0])),  // front
            ("6", [0, 0, -half], simd_quatf(angle: .pi,      axis: [0, 1, 0])),  // back
            ("2", [ half, 0, 0], simd_quatf(angle:  .pi / 2, axis: [0, 1, 0])),  // right
            ("5", [-half, 0, 0], simd_quatf(angle: -.pi / 2, axis: [0, 1, 0])),  // left
            ("3", [0,  half, 0], simd_quatf(angle: -.pi / 2, axis: [1, 0, 0])),  // top
            ("4", [0, -half, 0], simd_quatf(angle:  .pi / 2, axis: [1, 0, 0])),  // bottom
        ]
        for face in faces {
            let text = ModelEntity(
                mesh: .generateText(face.label,
                                    extrusionDepth: 0.002,
                                    font: .systemFont(ofSize: 0.07, weight: .bold)),
                materials: [SimpleMaterial(color: .white, isMetallic: false)]
            )
            // generateText anchors at the baseline's origin, not the glyph center.
            let bounds = text.model!.mesh.bounds
            text.position = [-bounds.center.x, -bounds.center.y, 0]

            let holder = Entity()
            holder.position = face.position
            holder.orientation = face.rotation
            holder.addChild(text)
            cube.addChild(holder)
        }
        return container
    }
}

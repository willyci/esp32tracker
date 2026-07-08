import Foundation
import simd

/// Drives the catheter (LEFT tracker) and guidewire (RIGHT tracker) from hardware input,
/// replicating VascCath's manipulation model with the ESP32 trackers in place of pinches:
///
///   grab    = SoftPot touched (finger on the strip)
///   twist   = SoftPot slide + physically rolling the tracker (BNO08x), both accumulate
///   insert  = Vision Pro hand tracking — the grabbed hand's world-X movement, with
///             VascCath's exact scale factors and depth limits (HandClass.swift)
///
/// Runs entirely on the main thread: BLE callbacks arrive on the main queue and the
/// hand-tracking loop is a MainActor task, so no locking is needed.
final class SimulationModel: ObservableObject {

    struct Tool {
        var grabbed = false
        var insertion: Float = 0   // meters advanced into the vessel
        var twist: Float = 0       // radians about the long (X) axis
    }

    @Published private(set) var catheter = Tool()   // left hand
    @Published private(set) var wire = Tool()       // right hand

    // Same limits/scales as VascCath: catheter 0–0.58 m @ 0.635, wire 0–0.61 m @ 0.63.
    private static let maxInsertion: [Hand: Float] = [.left: 0.58, .right: 0.61]
    private static let insertionScale: [Hand: Float] = [.left: 0.635, .right: 0.63]

    /// Sliding the full 255-unit strip = this many full turns.
    private static let stripFullTurns: Float = 1.0
    /// Sensor-frame axis the user twists the tracker around. Tune to how the boards are
    /// held: (1,0,0) = the BNO's X (long board edge) after the app-frame sign flips.
    private static let twistAxis = SIMD3<Float>(1, 0, 0)

    private var lastTouch: [Hand: UInt8] = [:]
    private var lastQuat: [Hand: simd_quatf] = [:]

    init(ble: BLEManager) {
        ble.left.onSample  = { [weak self] in self?.ingest($0) }
        ble.right.onSample = { [weak self] in self?.ingest($0) }
    }

    func tool(for hand: Hand) -> Tool { hand == .left ? catheter : wire }
    private func setTool(_ t: Tool, for hand: Hand) {
        if hand == .left { catheter = t } else { wire = t }
    }

    /// One BLE sample: update grab state; while grabbed, accumulate twist from both the
    /// strip slide and the tracker's roll. Everything is frame-to-frame incremental, so
    /// re-grabbing or reconnecting never makes the rod jump.
    private func ingest(_ s: TrackerState) {
        var t = tool(for: s.hand)
        t.grabbed = s.touchActive

        if t.grabbed {
            // SoftPot slide → twist (skip the touch-down sample: no previous point yet)
            if let last = lastTouch[s.hand], last > 0, s.touchCurrent > 0 {
                let slide = Float(Int(s.touchCurrent) - Int(last))
                t.twist += slide / 255 * (2 * .pi) * Self.stripFullTurns
            }
            // BNO08x roll → twist: take only the rotation component about the long axis
            // (swing–twist decomposition of the frame-to-frame delta)
            if let lastQ = lastQuat[s.hand] {
                var delta = lastQ.inverse * s.quaternion
                if delta.real < 0 {   // q and −q are the same rotation; keep the short arc
                    delta = simd_quatf(ix: -delta.imag.x, iy: -delta.imag.y,
                                       iz: -delta.imag.z, r: -delta.real)
                }
                t.twist += Self.twistAngle(of: delta, around: Self.twistAxis)
            }
        }
        lastTouch[s.hand] = s.touchCurrent
        lastQuat[s.hand] = s.quaternion
        setTool(t, for: s.hand)
    }

    /// Vision Pro hand moved: while that hand's tool is grabbed, its world-X delta
    /// advances/retracts the tool — VascCath's mapping (negative X = insertion).
    func handMoved(_ hand: Hand, deltaX: Float) {
        var t = tool(for: hand)
        guard t.grabbed else { return }
        t.insertion = min(Self.maxInsertion[hand]!,
                          max(0, t.insertion + -deltaX * Self.insertionScale[hand]!))
        setTool(t, for: hand)
    }

    /// Signed rotation of `q` about `axis` — the "twist" half of swing–twist.
    private static func twistAngle(of q: simd_quatf, around axis: SIMD3<Float>) -> Float {
        let proj = simd_dot(q.imag, axis)
        guard abs(proj) > 1e-9 || abs(q.real) > 1e-9 else { return 0 }
        return 2 * atan2(proj, q.real)
    }
}

# Coordinate-frame mapping: BNO085 → RealityKit

The box rotates "wrong" out of the box because the sensor and RealityKit describe orientation in
**different coordinate frames**. This guide explains the two frames, the one correct operation to
convert between them, and a 10-minute empirical procedure to nail it down.

The relevant code is `displayOrientation` in `ESP32Tracker/BLEManager.swift`.

---

## The two frames

Both are **right-handed**, which is the good news — converting between two right-handed frames is
a pure rotation (no axis negation / handedness flip needed).

### BNO085 (sensor)
Right-handed, and its world reference is **gravity-aligned with +Z up**:
- **+X** — along one edge of the breakout (see the silkscreen arrows)
- **+Y** — 90° from X, in the board plane
- **+Z** — straight up out of the top face of the chip

The Rotation Vector report gives the sensor body's orientation relative to this Earth frame
(Z = up, heading fixed by the magnetometer). We zero out the absolute heading with **Re-center**,
so the only thing that matters for conversion is **Z is up**.

### RealityKit (graphics)
Right-handed, but **+Y is up** (the classic ARKit/OpenGL-style camera convention):
- **+X** — right
- **+Y** — up
- **+Z** — toward the viewer (−Z goes into the screen)

### The mismatch in one line
> Sensor up is **+Z**. RealityKit up is **+Y**. Everything else follows from reconciling those.

---

## The correct operation: conjugation (change of basis)

A common first instinct is to *premultiply* the sensor quaternion by a fixed rotation
(`remap * q`). That **rotates the result** but does not re-express it in a new frame, so it won't
truly fix the axes. To re-express a rotation `q` in a different basis you **conjugate** it by the
change-of-basis rotation `C`:

```
q_realitykit = C · q_sensor · C⁻¹
```

`C` is the rotation that carries the sensor axes onto the RealityKit axes. To send sensor **+Z**
(up) to RealityKit **+Y** (up), `C` is a **−90° rotation about X**:

```swift
let basis = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
```

Under this `C`:

| Sensor axis | → RealityKit axis | Meaning |
|-------------|-------------------|---------|
| +X | +X | right stays right |
| +Y | −Z | sensor "forward" goes *into* the screen |
| +Z | +Y | up becomes up ✓ |

That's why `displayOrientation` is:

```swift
basis * (reference * quaternion) * basis.inverse * mountOffset
```

- `reference * quaternion` — the Re-center step (relative rotation, still in sensor frame).
- `basis * … * basis.inverse` — the conjugation that moves it into RealityKit's frame.
- `mountOffset` — a separate post-rotation for physical mounting (next section).

---

## `mountOffset`: where the sensor is glued, not how the frames differ

`basis` fixes the *coordinate convention*. It does **not** know that you taped the breakout to the
side of your probe model at some angle. That physical offset is a separate rotation applied on the
object side (post-multiply):

```swift
private let mountOffset = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)   // identity = aligned
```

Keep it identity until `basis` is confirmed, then dial it in. Examples:

```swift
// Sensor mounted rotated 90° around the probe's long axis (RealityKit X):
let mountOffset = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])

// Probe model's "tip" points the opposite way from the sensor's +Z:
let mountOffset = simd_quatf(angle: .pi, axis: [0, 1, 0])
```

---

## 10-minute calibration procedure

Do these **in order** — fixing `basis` before `mountOffset` keeps the two concerns separate.

1. **Lay the sensor flat, board level, +X pointing away from you. Tap Re-center.**
   The box should snap to its neutral pose.

2. **Check "up" first (the `basis` axis).** Tilt the *front edge* of the sensor up, like lifting
   the nose of a plane.
   - Box nose tilts **up** on screen → `basis` is correct. Go to step 4.
   - Box rolls or yaws instead → `basis` axis is wrong, go to step 3.

3. **Fix `basis`.** Try the other single-axis 90° rotations until "pitch up in real life" =
   "pitch up on screen":
   ```swift
   simd_quatf(angle:  .pi/2, axis: [1,0,0])   // +90° X
   simd_quatf(angle: -.pi/2, axis: [1,0,0])   // −90° X  (default)
   simd_quatf(angle:  .pi/2, axis: [0,0,1])   // +90° Z
   ```
   Re-test step 2 after each change.

4. **Check direction sense.** Rotate the sensor **clockwise** (yaw right).
   - Box turns the same way → good.
   - Box turns the opposite way → your sensor is mounted mirrored; add a 180° `mountOffset`
     about the up axis, or re-seat the sensor.

5. **Fix mounting with `mountOffset`.** Now that motions map to the right *axes*, if the box's
   geometry is just pointing the wrong way (tip backward, etc.), rotate `mountOffset` until the
   model lines up with the physical sensor. This never affects which axis a motion appears on —
   only the resting orientation of the model.

6. **Re-center again** and confirm tilt/roll/yaw all read intuitively.

---

## Quick reference: building rotation quaternions

```swift
// Angle (radians) about an axis:
simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))   // 90° about X

// Identity (no rotation):
simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

// Compose: a then b  →  b * a   (quaternion multiply is right-to-left)
// Inverse of a unit quaternion:  q.inverse  (== conjugate)
```

If you ever switch to a left-handed target frame, a pure conjugation is no longer enough — you'd
also negate one component. RealityKit is right-handed, so you won't hit that here.

// Shared tracker math for the dashboard pages — extracted verbatim from index.html so
// index.html and pc_connection.html cannot drift apart.
//
// Everything here is a port of the visionOS app; if you change one, change the other:
//   ESP32Tracker/TrackerState.swift     → BASIS, MOUNT_OFFSET, displayOrientation
//   ESP32Tracker/SimulationModel.swift  → SIM constants, simIngest, handMoved, twistAngle
"use strict";

// ---------- quaternion helpers ({w,x,y,z}) ----------
export const qMul = (a, b) => ({
  w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
  x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
  y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
  z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
});
export const qConj = q => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });
export const qAxisAngle = (axis, angle) => {
  const s = Math.sin(angle / 2);
  return { w: Math.cos(angle / 2), x: axis[0]*s, y: axis[1]*s, z: axis[2]*s };
};
export const Q_IDENTITY = { w: 1, x: 0, y: 0, z: 0 };

// ---------- frame mapping — ported 1:1 from ESP32Tracker/TrackerState.swift ----------
// raw packet → negate x,z (mirror fix), then conjugate by basis (BNO Z-up → Y-up),
// then the physical mounting offset. Keep these in sync with the Swift app.
export const BASIS = qMul(qAxisAngle([1,0,0], Math.PI/2), qAxisAngle([0,0,1], Math.PI/2));
export const BASIS_INV = qConj(BASIS);
export const MOUNT_OFFSET = qAxisAngle([1,0,0], Math.PI/2);

export function displayOrientation(sample, reference) {
  const q = { w: sample.w, x: -sample.x, y: sample.y, z: -sample.z };
  return qMul(qMul(qMul(BASIS, qMul(reference, q)), BASIS_INV), MOUNT_OFFSET);
}

// Quaternion (Y-up right-handed, like RealityKit) → CSS matrix3d.
// CSS's screen frame has Y pointing DOWN, so conjugate the rotation matrix with
// diag(1,-1,1) — negate every element with exactly one Y index. Column-major output.
export function quatToMatrix3d(q) {
  const {w, x, y, z} = q;
  const m = [
    1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y),
    2*(x*y + w*z),      1 - 2*(x*x + z*z),  2*(y*z - w*x),
    2*(x*z - w*y),      2*(y*z + w*x),      1 - 2*(x*x + y*y),
  ];
  const F = [1, -1, 1];
  const c = (r, cIdx) => F[r] * m[r*3 + cIdx] * F[cIdx];
  return `matrix3d(${c(0,0)},${c(1,0)},${c(2,0)},0,` +
                  `${c(0,1)},${c(1,1)},${c(2,1)},0,` +
                  `${c(0,2)},${c(1,2)},${c(2,2)},0,0,0,0,1)`;
}

// ---------- catheter/wire simulation — ported 1:1 from ESP32Tracker/SimulationModel.swift ----------
// grab = SoftPot touched · twist = strip slide + tracker roll (swing–twist about the
// board's long axis, in the same app frame the Swift model uses: w,−x,y,−z) ·
// insert = arrow keys / mouse wheel here (the headset uses ARKit hand tracking).
export const SIM = {
  maxInsertion:   { left: 0.58, right: 0.61 },   // meters — VascCath limits
  insertionScale: { left: 0.635, right: 0.63 },
  stripFullTurns: 1.0,            // sliding the full 255-unit strip = this many turns
  twistAxis: [1, 0, 0],
  catheterLength: 0.60, wireLength: 0.65,        // display lengths (SimulationView.swift)
  vesselLength: 0.70,
};
export const tools = {
  left:  { grabbed: false, insertion: 0, twist: 0 },   // catheter
  right: { grabbed: false, insertion: 0, twist: 0 },   // guidewire
};
const simLastTouch = {};   // hand → previous touchCurrent
const simLastQuat  = {};   // hand → previous app-frame quaternion

// Signed rotation of q about axis — the "twist" half of swing–twist.
export function twistAngle(q, axis) {
  const proj = q.x*axis[0] + q.y*axis[1] + q.z*axis[2];
  if (Math.abs(proj) < 1e-9 && Math.abs(q.w) < 1e-9) return 0;
  return 2 * Math.atan2(proj, q.w);
}

// One BLE sample: update grab; while grabbed, accumulate twist from the strip slide
// and the tracker's frame-to-frame roll. Incremental → re-grabbing never jumps.
export function simIngest(hand, s) {
  const t = tools[hand];
  t.grabbed = s.touchCurrent > 0;
  const q = { w: s.w, x: -s.x, y: s.y, z: -s.z };   // app frame, as TrackerState.swift

  if (t.grabbed) {
    const lastT = simLastTouch[hand];
    if (lastT > 0 && s.touchCurrent > 0)
      t.twist += (s.touchCurrent - lastT) / 255 * 2 * Math.PI * SIM.stripFullTurns;

    const lastQ = simLastQuat[hand];
    if (lastQ) {
      let d = qMul(qConj(lastQ), q);
      if (d.w < 0) d = { w: -d.w, x: -d.x, y: -d.y, z: -d.z };   // keep the short arc
      t.twist += twistAngle(d, SIM.twistAxis);
    }
  }
  simLastTouch[hand] = s.touchCurrent;
  simLastQuat[hand] = q;
}

/// Drop a hand's baseline, so a reconnect doesn't register a giant twist delta.
export function simForget(hand) {
  delete simLastQuat[hand];
  delete simLastTouch[hand];
}

// PC stand-in for SimulationModel.handMoved: negative deltaX = insertion.
//
// `force` is a dashboard-only escape hatch (the demo page's "insert without grab" toggle)
// so the page can be rehearsed with no tracker attached. The app always requires the grab.
export function handMoved(hand, deltaX, force = false) {
  const t = tools[hand];
  if (!t.grabbed && !force) return;
  t.insertion = Math.min(SIM.maxInsertion[hand],
                Math.max(0, t.insertion + -deltaX * SIM.insertionScale[hand]));
}

/// Add twist directly, in radians.
///
/// Dashboard-only: on the headset every radian of twist comes from the SoftPot or the
/// tracker's own roll. This is the keyboard stand-in for rolling the board, so the demo
/// page can be driven with no hardware on the table.
export function twistBy(hand, radians) {
  tools[hand].twist += radians;
}

export function resetTools() {
  for (const t of Object.values(tools)) { t.insertion = 0; t.twist = 0; }
}

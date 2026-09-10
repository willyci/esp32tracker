// VascCath's X-ray frame math, ported 1:1 to JavaScript.
//
// This is what lets pc_connection.html show the SAME picture the Vision Pro shows: the app
// picks a PNG out of a 2700-frame sequence from (insertion depth, branch), and picks the
// branch from accumulated rotation by quadrant. Both are pure arithmetic with no Apple
// dependency, so the port is exact rather than approximate.
//
// Sources — keep in sync if the app changes:
//   Managers/VideoControllerClass.swift  → distances, getCatheterFrameByDistance, getLastCatheterPathFrame
//   Model/AppModel.swift                 → branch(forUnitRotation:), pathCommitted, updateCatheterPath
//   Views/Main/CatheterTipDial.swift     → tip-rotation dial frame
//   Views/Simulation/XRay/DyeLayerView.swift → contrast injection frame ranges
"use strict";

// ---------- distance landmarks (VideoControllerClass.swift:130-138) ----------
// Units are meters of tracked hand travel (the Swift comments say "cm", the values are m).
export const MIN_CATHETER_DISTANCE    = 0.0;
export const CELIAC_TRUNK_DISTANCE    = 0.50;   // celiac trunk origin / branching point
export const SPLENIC_ARTERY_DISTANCE  = 0.55;   // splenic artery catheterization target
export const MAX_CATHETER_DISTANCE    = 0.58;   // deepest safe catheter/guidewire depth
export const MAX_WIRE_DISTANCE        = 0.61;

export const PATHS = ["CeliacTrunk", "SplenicArtery", "HepaticArtery", "GastricArtery"];

/// Plain linear remap — AppModel.convertValueToNewRange.
const remap = (v, oldMin, oldMax, newMin, newMax) =>
  ((v - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;

/// The deepest frame each branch reaches. Non-splenic branches dead-end here.
export function getLastCatheterPathFrame(path) {
  switch (path) {
    case "CeliacTrunk":   return 899;
    case "GastricArtery": return 1639;
    case "HepaticArtery": return 1259;
    case "SplenicArtery": return 2699;
    default:              return 899;
  }
}

/// Depth → frame index in the 2700-frame catheter/guidewire sequences.
///
/// Three segments: a shared shaft before the celiac trunk (every branch draws the same
/// frames), a per-branch segment from the celiac to the splenic target, and a deep-splenic
/// tail that only the splenic branch can enter. The unused gaps between branch bands
/// (900-1079, 1260-1439, 1640-1799) are never addressed — that's the app's layout, not a bug.
export function getCatheterFrameByDistance(distance, path = "CeliacTrunk") {
  if (distance < MIN_CATHETER_DISTANCE) return 0;                       // tool removed
  if (distance > MAX_CATHETER_DISTANCE && path === "SplenicArtery") return 2699;

  if (distance < CELIAC_TRUNK_DISTANCE) {
    // 0-50cm → frames 0-719, shared by every branch
    return Math.floor(remap(distance, MIN_CATHETER_DISTANCE, CELIAC_TRUNK_DISTANCE, 0, 719));
  }
  if (distance < SPLENIC_ARTERY_DISTANCE) {
    switch (path) {
      case "CeliacTrunk":   return Math.floor(remap(distance, CELIAC_TRUNK_DISTANCE, SPLENIC_ARTERY_DISTANCE,  720,  899));
      case "HepaticArtery": return Math.floor(remap(distance, CELIAC_TRUNK_DISTANCE, SPLENIC_ARTERY_DISTANCE, 1080, 1259));
      case "GastricArtery": return Math.floor(remap(distance, CELIAC_TRUNK_DISTANCE, SPLENIC_ARTERY_DISTANCE, 1440, 1639));
      case "SplenicArtery": return Math.floor(remap(distance, CELIAC_TRUNK_DISTANCE, SPLENIC_ARTERY_DISTANCE, 1800, 2399));
    }
  }
  // Past the splenic target: only the splenic branch has frames left to show.
  if (path === "SplenicArtery")
    return Math.floor(remap(distance, SPLENIC_ARTERY_DISTANCE, MAX_CATHETER_DISTANCE, 2400, 2699));
  return getLastCatheterPathFrame(path);
}

/// The guidewire shares the catheter's frame layout, driven by the RIGHT hand's depth and
/// clamped to the sequence (GuideWirePathView.swift).
export const getWirePathFrame = (distance, path) =>
  Math.max(0, Math.min(getCatheterFrameByDistance(distance, path), 2699));

// ---------- rotation → branch (AppModel.swift:225) ----------

/// Which branch an accumulated rotation aims at, by quadrant:
///   [315,45) celiac · [45,135) splenic · [135,225) hepatic · [225,315) gastric
export function branch(forUnitRotation) {
  let r = forUnitRotation % (Math.PI * 2);
  if (r < 0) r += Math.PI * 2;
  if (r < Math.PI * 0.25 || r >= Math.PI * 1.75) return "CeliacTrunk";
  if (r < Math.PI * 0.75)  return "SplenicArtery";
  if (r < Math.PI * 1.25)  return "HepaticArtery";
  return "GastricArtery";
}

/// True once any in-vessel tool is past the celiac branching depth. While true the app
/// refuses to re-evaluate the branch — you are committed to whichever one you turned to.
export const pathCommitted = (catheterDistance, wireDistance) =>
  catheterDistance > CELIAC_TRUNK_DISTANCE || wireDistance > CELIAC_TRUNK_DISTANCE;

/// The app's continuous path preview (AppModel.updateCatheterPath): rotation only chooses
/// the branch while the catheter is still short of the celiac trunk. Returns the path to
/// use now, given the one already active.
export function updateCatheterPath(currentPath, rotation, catheterDistance, wireDistance) {
  if (pathCommitted(catheterDistance, wireDistance)) return currentPath;
  if (catheterDistance < CELIAC_TRUNK_DISTANCE) return branch(rotation);
  return currentPath;
}

// ---------- catheter tip dial (CatheterTipDial.swift) ----------

/// 360 frames, one per whole degree.
///
/// NOTE the app takes abs() *after* the remainder, so negative rotations MIRROR rather than
/// wrap: −90° lands on frame 90, not 270. Reproduced deliberately — the dial here has to
/// match the dial in the headset, quirk included.
export const tipRotationFrame = (radians) =>
  Math.abs(Math.trunc((radians * 180 / Math.PI) % 360));

// ---------- contrast injection (DyeLayerView.swift) ----------

/// Frame range the dye animation sweeps for each injection target.
export const DYE_RANGES = {
  CeliacTrunk:   [0, 1080],      // abdominal aorta
  HepaticArtery: [1080, 1440],
  GastricArtery: [1440, 1800],
  SplenicArtery: [1800, 2699],
};
export const DYE_DURATION_S = 3.0;   // 3 s at 30 fps, as in DyeLayerView

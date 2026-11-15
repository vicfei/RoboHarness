# Postmortem: four real-robot incidents, and the gates they shipped

Every safety feature in this repository exists because something physically
went wrong on a real arm. This is the English summary of the working log
([docs/createblock-integration-log-zh.md](docs/createblock-integration-log-zh.md),
§11–§13). No incident below was observed in simulation.

Hardware context: a $200-class CreateBlock 5-DOF arm (CLB-S325 bus servos,
±5–10 mm repeatability, no force/torque sensing, no compliance) driven through
a 4GB Jetson Nano, with a single 640×480 wrist camera as the only exteroception.
Task under test: pick a white charging plug, place it on a pink plate.

---

## Incident 1 — the vendor macro that flung the plug

**What happened.** Early trials called the vendor's one-shot convenience
macro `grab_cubic()`. Internally it chains fast unsynchronized joint swings.
The gripper hit the white plug at speed and flung it off the table; the
return swing knocked a second (black) plug off the desk. Nothing watched the
motion while it happened.

**Root cause.** Opaque macros are invisible to any observer: no per-step
observation, no current telemetry, no abort point. On cheap bus servos a
"convenience" macro is an open-loop blind swing with mass behind it.

**The gate that shipped.**
- `MACRO_BAN` in `discipline.py`: one-shot vendor macros are banned at the
  plan level; a plan containing one is rejected before any motion.
- All motion is now controlled micro-steps (≤ 6 mm Cartesian per step), each
  step observed (FK + vision) and gated on live current telemetry.

**Status:** enforced since 2026-11-15; no recurrence.

---

## Incident 2 — the Cartesian lift that descended

**What happened.** With the tool at z = 37.5 mm, the executor commanded a
+5 mm Cartesian lift. The arm's real motion was **−7.5 mm downward** (requested
tool z ≈ 45.0 mm, measured 32.5 mm). Motion peak current was 0.096 A — normal
loaded motion — so nothing looked wrong until the postcondition check caught
the direction reversal.

**Root cause.** Near the workspace floor, this arm's IK plus servo compliance
do not preserve commanded Cartesian direction; "up" commands can map to
downward joint paths. The old executor trusted the command, checked only
afterward, and had no low-pose policy at all.

**The gate that shipped.**
- Low-pose Cartesian lock: below 70 mm tool height, Cartesian IK is refused.
- In-motion watchdog: FK is tracked *during* motion (not only after), with
  direction/progress checks per step and a hard 6 mm step ceiling.
- Cartesian postconditions: z-lock tolerance, per-step displacement bounds;
  a "close enough" joint readout is never accepted as Cartesian success.

**Status:** the failing motion was frozen mid-task and the arm parked safely;
the lock is permanent design, not configuration.

---

## Incident 3 — the recovery that made things worse

**What happened.** The arm was left in a legal-but-collapsed pose (joint 2
past its soft lower limit after a manual move). Two recovery attempts:

1. A −0.020 rad joint-2 nudge produced **no motion at all** — well inside the
   servo's 3° deadzone. A naive controller would have escalated the command
   blindly.
2. A coordinated multi-joint recovery, verified safe in offline kinematics
   (predicted +3.95 mm lift), **descended on the real arm**: the bus servos
   execute asynchronously, joint 3 led the pack, and the FK watchdog halted
   the motion at ≈ −2.6 mm before contact.

**Root cause.** Cheap bus servos do not execute coordinated trajectories
synchronously, and small commands vanish into the deadzone. Offline
kinematic verification says nothing about asynchronous execution order.

**The gate that shipped.**
- `LOW_POSE_RECOVERY_AVAILABLE = False`: coordinated low-pose recovery is
  disabled outright; the documented procedure is power-cycle and manual safe
  posing. An honest "requires a human" beats an elegant recovery that
  occasionally drives the arm into the table.
- Startup self-check refuses tasks from collapsed poses (tool < 30 mm),
  because the vendor `home()` would press the arm harder into its own jam.
- Deadzone-aware acceptance: a no-op is accepted only when tracking tolerance
  is already satisfied; low-current motion is accepted only when motion is
  actually measured (a stalled motor reading 0.03 A is a fuse, not a pass).

**Status:** permanent; revisited only with hardware that can prove
synchronous execution.

---

## Incident 4 — the grasp that closed on nothing

**What happened.** Multiple grasp attempts on the real plug closed cleanly:
grip angle ≈ 0.026–0.030 rad, normal close currents, no errors. The gripper
was **empty every time**. The plug's position was verified in-frame before
descent; the jaw pads closed around where the plug should have been.

**Root cause.** The harness has no jaw-contact model: with a two-finger
parallel gripper, no F/T sensing, and ±5–10 mm repeatability, "centered in
frame" does not imply "between the pads". Grasp success cannot be inferred
from grip angle + current alone when the object is small.

**The gate that shipped.**
- `CONTACT_APPROACH_AVAILABLE = False`: contact Cartesian deltas and low
  joint-pose approaches are disabled until jaw-contact geometry is
  calibrated (milestone M1.5).
- Grasp evidence standard: grip angle + close-current peak + jaw-region
  vision before/after comparison; anything less reports "unverified", never
  "success".
- The 2026-11-14 fully-automatic grasp (see the homepage video) passed this
  standard; the plug-to-plate task remains open until M1.5 lands and contact
  approaches are re-enabled on evidence.

**Status:** locked by design; the honest status is "grasp works, transport
contact does not yet".

---

## What this project does *not* claim

- No trained policy of any kind is in the execution path; residual RL is a
  planned, bounded, optional plugin (M1.7) that never bypasses the executor.
- Contact-rich manipulation is locked, not solved.
- SO-ARM100/101 is a read-only state adapter; motion is intentionally
  disabled until that embodiment earns the same evidence the CreateBlock
  gates were built on.

Each incident above maps to code you can read: `discipline.py`,
`executor/createblock.py`, `tests/` (64/64 offline tests). The full raw
timeline, including every intermediate failure, is in the Chinese working
log.

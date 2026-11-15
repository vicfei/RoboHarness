# M1.7 — Residual RL pilot (planned)

Status: **planning only; no policy training has started.** This milestone is an
optional skill plugin for the existing harness, not a replacement for it.

## Motivation

The 2026-11-15 plug-to-plate rerun showed a specific gap: the plug could appear
between the fingers in the wrist image while remaining outside the physical
jaw-pad volume. Repeated closures ended at 0.026–0.030 rad (empty grip), so the
missing capability is contact-depth / jaw-alignment residual control, not another
high-level planner.

A small residual policy is worth piloting because the failure is local,
repetitive, and observable. End-to-end RL is explicitly out of scope: it would
bypass the deterministic primitives and safety accounting that define this repo.

## Non-negotiable architecture

```
cloud / scripted planner
        ↓
deterministic primitives
        ↓
residual policy            ← optional, task-phase only
        ↓
safety executor            ← clipping, limits, current/FK gates, cancellation
        ↓
GoalVerifier
```

The policy never talks to servos directly. It proposes bounded residuals, and
`CreateBlockExecutor` remains the only actuation authority:

| Policy output | Bound | Executor behavior |
|---|---|---|
| joint residual | ≤0.05 rad per axis | clip or reject, then angle/FK feedback gate |
| contact Cartesian residual | ≤3 mm per step | rejected while contact approaches are locked |
| gripper command | open/close only | current peak + non-empty angle evidence required |
| terminate | allowed | returns control to primitive planner |

No residual action may disable `MACRO_BAN`, current spike protection, FK
clearance, no-op detection, or task fuses. Policy checkpoints are versioned with
their action space and training distribution.

## Stage A — system identification before training

Inputs:

- The owner-recorded 2026-11-14 successful grasp/release clip.
- Existing evidence frames and `STATE` logs from 2026-11-14/15.
- Servo deadband, asynchronous-response, and height-undershoot observations from
  `docs/createblock-integration-log-zh.md` §11–13.

Deliverables:

1. Finger-tip and jaw-pad projection model in the wrist image.
2. Estimated tool-height offset at the contact phase.
3. Empty-close vs object-close distributions for gripper angle and close-current
   peak.
4. PyBullet twin updated with calibrated gripper geometry, 3° deadband, delayed
   joint responses, and measured height error.

Exit gate: the simulator must reproduce both an empty closure and the successful
2026-11-14 contact using the same recorded action sequence, within the measured
angle/current tolerances. Training without this gate would optimize an idealized
arm rather than the hardware.

## Stage B — curriculum in the twin

The runnable local twin now lives at
`~/services/robotics/createblock-arm/sim/` (original vendor STL meshes, Nano
URDF, HEADLESS PyBullet port). Its seven-frame regression reaches home within
2.40 mm and `object_release` within 0.30 mm on WSL/PyBullet 3.2.5, so it is the
default Stage-B environment. Run `sim/run_smoke.py` before each training change.

Curriculum:

1. Hover above a randomized plug pose.
2. Align jaw-pad volume with the graspable plug body.
3. Close and retain the plug (angle + simulated contact force).
4. Lift while the plug remains in the jaw volume.
5. Move above the pink plate and release.

Reward sketch:

- negative distance to a graspable body point;
- non-empty gripper angle and retained-object bonus after lift;
- plate-release success bonus;
- penalties for collision, overcurrent, limit proximity, large residuals, and
  empty closures;
- terminal penalty for losing the object or requesting an unsafe action.

Domain randomization:

- plug and plate poses;
- camera extrinsics / detector bbox jitter;
- servo deadband, latency, and asynchronous start;
- joint read noise and tool-height offset;
- friction, plug dimensions, and illumination/HSV disturbance.

Sim exit target: ≥80% grasp-and-place success over held-out randomized episodes,
with zero simulated safety violations and a lower empty-closure rate than the
deterministic baseline.

## Stage C — harness-wrapped real A/B trial

Trial protocol:

- Baseline arm A: existing deterministic primitives only.
- Arm B: identical primitives plus the residual policy in the contact phase.
- Same start poses, same plug/plate layout distribution, same evidence recorder.
- Contact approaches remain locked until M1.5 calibration provides the executor
  with a valid jaw-contact gate; RL does not bypass that lock.

Acceptance:

| Metric | Minimum |
|---|---|
| Real plug-to-plate success | ≥6/10 residual arm episodes |
| Improvement over baseline | statistically visible in 10+10 episodes |
| Empty closures | materially below baseline |
| Safety violations / driver protection | 0 |
| Unexplained actions | 0 — every action logged with phase and bound |

Failure is reportable: if the policy does not beat the calibrated deterministic
baseline, it stays out of the default path and the result is documented.

## Evidence and public demo policy

The 2026-11-14 grasp/release clip is valuable milestone evidence, but it must not
be presented as plug-to-plate success. Until an uncut pipeline recording shows
detection, alignment, closure evidence, and GoalVerifier output, use it in a
“first real closed-loop grasp” milestone section. A homepage hero requires:

- continuous footage with timestamps;
- scan/center/grasp/verify phase overlay;
- gripper angle or current trace;
- explicit task-scope caption;
- link to the failure postmortem and safety layer.

## Repository impact

Planning documents only in this milestone. No new runtime dependency, no model
checkpoint, and no claim of trained capability until Stage C passes.

# RoboHarness — Design v1 (2026-11-14)

> Inherits the Robot Use Agent route (RobotAgent docs/06) and the real-hardware
> safety spec hardened on 2026-11-14 (see the Chinese integration log for the
> incident chain that produced it). This is the English design baseline.

## Goal

Tabletop tasks (pick / place / sort / push / compose) on low-cost real arms,
with **no model training**: a cloud LLM plans at low frequency, deterministic
primitives actuate, vision closes every loop, evidence records every step.

## Three-tier brain

```
Brain   (cloud, 1–3 calls/task)  DeepSeek-Vision / GLM-4V / Kimi — task → primitive JSON
Midbrain (on-device, resident)   Python harness ~300 MB: state machine, primitives,
                                 perception, verifier, safety, evidence
Reflex  (local, never LLM)       joint commands + current watchdog, ms-level
```

The harness runs resident on the robot computer (here a 4 GB Jetson Nano,
headless): serial is direct, and an already-issued sequence keeps executing
through Wi-Fi loss. The dev box only rsyncs code and pulls evidence.

`FrameSource` abstracts both camera placements validated so far: device-name
FFmpeg capture when the wrist cam is on the laptop, `/dev/videoN` direct read
when it is on the Jetson.

## Primitives (hardware-proven pieces)

| Primitive | What it does | 2026-11-15 status |
|---|---|---|
| `scan_for(spec)` | multi-stance sweep: joint ladder × yaw set, early exit on FOUND | validated |
| `center_on(spec)` | anchor-point visual servoing with **measured** sign probes | validated for observation alignment |
| `grasp()` | open → gated approach → close → angle/current/vision hold check → lift | **contact path locked**: image overlap did not prove jaw-pad depth; jaw-contact calibration required |
| `release(pose)` / `store_locked()` | place-and-retract; storage keeps torque **on** (freewheel storage collapsed the arm once) | release sequence proven; no object retained in current rerun |

## Verifier (three-state)

PASS / FAIL / UNCERTAIN from three channels: gripper angle (>0.12 rad while
closing = object held), close-phase current (fighting an object), and
before/after frame diff. A VLM may add an advisory note but is **never the
judge** — twice in one day it hallucinated a plug onto desk reflections.

## Failure modes

| Failure | Response | Self-healing |
|---|---|---|
| Driver protection latched (≈0 A) | task fuse; power-cycle required | no — human |
| Detection NONE / UNCERTAIN | re-scan from alternate stance (≤3) | yes |
| LLM timeout / quota | scripted fallback planner | yes |
| Wi-Fi loss | finish issued sequence, queue planning | yes |
| Grasp missed (4-evidence fail) | lift → re-scan → re-center → re-grasp (≤2) | yes |
| Residual skill unsafe or no-gain | disable plugin; deterministic primitive remains default | yes |

## Milestones

- **M1 primitive layer (no LLM)** — offline suite green; free-space autonomy and
  three-state perception validated on hardware. Hardware completion is blocked on
  the jaw-contact model (contact primitives locked rather than retried blindly).
- **M1.5 jaw-contact calibration** — finger segmentation + two-view pad-volume
  check + repeatable contact-current/z-offset identification; unlock contact.
- **M1.6 second backend** — SO-100/101 state adapter now exists read-only; motion
  opens only after telemetry/FK/jaw gates are calibrated on that embodiment.
- **M1.7 residual RL pilot** — optional skill plugin for the contact-depth gap:
  first system-identify jaw geometry from the 2026-11-14 evidence, then train a
  bounded residual policy in the twin, then run a harness-wrapped real A/B trial.
  It never bypasses the executor or GoalVerifier. See
  [residual-rl-plan.md](residual-rl-plan.md).
- **M2 cloud brain** — DeepSeek planner + full supervisor/evidence chain;
  acceptance = "pick up the white object" end-to-end from one sentence.
- **M3 sim-rehearsed default** — every action sequence walks the local pybullet
  twin at `~/services/robotics/createblock-arm/sim/` (original meshes, HEADLESS
  WSL port) before the real arm; 5-task suite scoring.
- **M4 perception upgrade** — TensorRT detector replaces HSV; servo repeatability
  survey (predicted ±5–10 mm, to be measured).
- **M5 sim2real exhibit** — train a small DP alignment policy in the twin,
  deploy on the real arm (the sim-*rehearsed* claim upgrades to full sim2real).

## Backend contract

Executors are interchangeable only when their **capability truth** travels with
them (`roboharness.executor.backends`):

| Backend | State read | Motion | FK clearance | Current telemetry |
|---|---|---|---|---|
| CreateBlock 5-DOF | yes | yes, task-gated | yes | yes |
| SO-ARM100/101 | yes (LeRobot bus semantics) | **locked** | no shared model yet | no |

SO-100/101 is therefore a read-only adapter today. Motion unlocks only after all
three missing gates are calibrated for that embodiment: per-joint current
telemetry, an FK/free-space clearance model, and a jaw-contact threshold with
visual evidence. A normalized 0–100 gripper value never implies `held`; object
retention needs independent evidence.

## Repository layout

`roboharness/executor/` safety layer + code runners ·
`roboharness/perception/` three-state detector + by-name frame sources ·
`roboharness/primitives/` the validated pipelines · `tests/` offline suite.

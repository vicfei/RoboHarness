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

## Primitives (all validated on hardware)

| Primitive | What it does |
|---|---|
| `scan_for(spec)` | multi-stance sweep: 6-step joint ladder × yaw set, early exit on FOUND |
| `center_on(spec)` | anchor-point visual servoing with **measured** sign probes (one +Δ probe per axis, then ≤7 correction steps) |
| `grasp()` | open → stepwise descent (occlusion or current gate) → close → 4-evidence hold check → lift |
| `release(pose)` / `store_locked()` | place-and-retract; storage folds to the self-stable zero pose with torque **on** (freewheel storage collapsed the arm once — never again) |

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

## Milestones

- **M1 primitive layer (no LLM)** — code complete, 16/16 offline tests; hardware
  acceptance = the plug task reruns 3/3 fully automatic. *(in progress)*
- **M2 cloud brain** — DeepSeek planner + full supervisor/evidence chain;
  acceptance = "pick up the white object" end-to-end from one sentence.
- **M3 sim-rehearsed default** — every action sequence walks the pybullet twin
  (identical interface) before the real arm; 5-task suite scoring.
- **M4 perception upgrade** — TensorRT detector replaces HSV; servo repeatability
  survey (predicted ±5–10 mm, to be measured).
- **M5 sim2real exhibit** — train a small DP alignment policy in the twin,
  deploy on the real arm (the sim-*rehearsed* claim upgrades to full sim2real).

## Repository layout

`roboharness/executor/` safety layer + code runners ·
`roboharness/perception/` three-state detector + by-name frame sources ·
`roboharness/primitives/` the validated pipelines · `tests/` offline suite.

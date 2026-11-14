# RoboHarness

[![CI](https://github.com/vicfei/RoboHarness/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/vicfei/RoboHarness/actions/workflows/ci.yml)

**A general robotic harness: LLM agents operate real robot arms through
deterministic primitives with a hardware safety layer. Zero training.**

Same bet as [RoboHarness (arXiv:2610.14018)](https://arxiv.org/abs/2610.14018) —
*a simple harness outperforms VLA and world action models* — but running on
hardware the papers don't touch: a **$200-class 5-DOF bus-servo arm + a 4GB
Jetson Nano**, controlled end-to-end in the real world.

> Status: **first fully-automatic closed-loop grasp completed on real hardware**
> (scan → center → grasp → verify, 2026-11-14). M1 primitive layer code is in
> this repo with 16/16 offline tests; hardware acceptance reruns in progress.
> Demo video lands with the M1 milestone.

## Why a harness (and not another VLA)

- **Zero training.** A cloud LLM (DeepSeek / GLM / Kimi — any of them) plans at
  1–3 calls per task; deterministic code does 100% of the actuation.
- **Cheap hardware, honest engineering.** ±5–10 mm servo repeatability and a
  640×480 wrist cam mean open-loop is unusable — everything here is
  vision-closed-loop by construction.
- **Safety from real incidents.** The safety layer encodes lessons from actual
  failures: servo stalls, driver brown-outs, gravity-collapsed storage poses,
  camera-index drift, detector false positives.

## The safety layer (built into the executor)

| Gate | Behavior |
|---|---|
| Dual feedback | Every joint command verified by *current > 0.2 A* **and** angle tracking; ≈0 A means driver protection latched → task fuse, human attention required |
| Current spike fuse | Any loaded joint > 1.2 A → auto-retract two steps |
| Startup self-check | Tool height < 30 mm (collapsed pose) → refuse task (a vendor `home()` would press the arm harder into its jam) |
| Joint-space descent | Small steps + occlusion detection trigger the grasp; explicit-pitch IK is silently rejected on this class of arms |
| Three-state detection | FOUND / UNCERTAIN / **NONE** — weak candidates are never "best-effort" picked |
| Camera by device name | DirectShow indices drift across processes; device-name addressing is the only reliable path |

## Architecture

```
Cloud brain (1-3 calls/task)      Local harness (Python, ~300 MB)      Reflex (never LLM)
DeepSeek / GLM / Kimi API   ──►   state machine · primitives ·      ──►  uServo joint commands
task → primitive sequence         perception · verifier · evidence        + current watchdog
```

Four deterministic primitives, all validated on the real arm:
`scan_for` (multi-stance sweep) · `center_on` (sign-probe visual servoing) ·
`grasp` (descent + occlusion trigger + 4-evidence verification) · `release`.

Swap executors, not code: `SSHCodeRunner` (laptop→arm link, validated) and
`LocalCodeRunner` (harness resident on the Jetson) behind one interface.

## Hardware support

| Arm | Status |
|---|---|
| CreateBlock 5-DOF (loborobot, CLB-S325 bus servos) | ✅ active development |
| SO-ARM100/101 (Feetech) | 🗓 planned — see [RobotAgent](https://github.com/vicfei/RobotAgent) for the agent stack |
| Mainstream arms (xArm / ARX / …) | 🗓 roadmap |

## Quickstart (offline, no hardware needed)

```bash
pip install numpy opencv-python-headless
python -m unittest discover -s tests -v   # 16/16 offline tests
```

## Docs

- [docs/harness-design.md](docs/harness-design.md) — design v1 (three-tier brain, primitives, failure modes, M1–M4)
- [docs/createblock-integration-log-zh.md](docs/createblock-integration-log-zh.md) — raw Chinese working log: full bring-up, two incidents, unjam recipe, first grasp

Supervisor/evidence pattern borrowed from the ORA line of work — credit where due.

## License

MIT

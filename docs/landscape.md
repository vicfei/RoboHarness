# Landscape & positioning (GitHub snapshot: 2026-11-15)

This repo is **not** the benchmark project
[BinceQu/RoboHarness](https://github.com/BinceQu/RoboHarness). The names collide;
the bets are related but the deployment surface is different. This page keeps the
comparison factual and cites what each public README actually claims.

## Where this repo sits

**Incident-hardened harness for $200-class 5-DOF bus-servo arms.**

- Real CreateBlock arm: 5 DOF + gripper, CLB-S325 bus servos, 4 GB Jetson Nano.
- Zero training, zero demonstrations: deterministic primitives + three-state vision.
- The safety executor is the product: current/angle/FK gates, cancellation, no-op
  handling, task fuses, and post-incident lockouts.
- Honest failure accounting: the 2026-11-15 plug task found a missing jaw-contact
  model, so contact primitives are disabled rather than advertised as working.

The neighboring projects below are stronger at benchmarks, model serving, and
multi-embodiment breadth. None of their READMEs claims this repo's particular
combination: cheap bus-servo hardware, a resident sub-GPU harness, and a safety
executor written from real hardware incidents.

## Snapshot used for this page

| Repository | Stars | Last push (UTC) |
|---|---:|---|
| [BinceQu/RoboHarness](https://github.com/BinceQu/RoboHarness) | 517 | 2026-11-05 |
| [showlab/Show-Harness](https://github.com/showlab/Show-Harness) | 626 | 2026-11-06 |
| [Open-RUA/uni-robot-agent](https://github.com/Open-RUA/uni-robot-agent) | 409 | 2026-11-11 |
| [HexaFuture/HexaShell](https://github.com/HexaFuture/HexaShell) | 381 | 2026-11-03 |
| [robot-agent-exploration/rax](https://github.com/robot-agent-exploration/rax) | 349 | 2026-10-24 |
| [autonomous-ai/openharness](https://github.com/autonomous-ai/openharness) | 1,489 | 2026-11-15 |
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | 8,123 | 2026-10-06 |
| [huggingface/lerobot](https://github.com/huggingface/lerobot) | 28,888 | 2026-11-15 |

Numbers move; the qualitative comparison below is the durable part.

## Closest neighbors

| Project | Public focus (from README) | Relationship to this repo |
|---|---|---|
| [BinceQu/RoboHarness](https://github.com/BinceQu/RoboHarness) | LLM control panel using visual clicks, keypoint tracking, geometric constraints; BEHAVIOR / RoboDojo evaluation; R1 Pro and ARX X5 workspaces; GPU-class validation host | Same "harness beats VLA" thesis, benchmark/agent-tool orientation. This repo is the cheap-real-hardware executor complement, not a reproduction. |
| [showlab/Show-Harness](https://github.com/showlab/Show-Harness) | Compact semantic action units for a VLM; deterministic embodiment interpreters; Franka, AgileX Piper, Isaac Lab, LIBERO; optional small-VLM fine-tuning | Useful vocabulary layer. It still assumes capable arms; this repo supplies cheap-bus-servo reality and safety accounting. |
| [Open-RUA/uni-robot-agent](https://github.com/Open-RUA/uni-robot-agent) | Unified robot-use agent harness with Codex/Claude runtimes, actor backends, R1Pro/G1/Go2/ARX ONE integrations | Architecture reference for runtime/tool separation. This repo deliberately stays tiny and resident on a 4 GB Nano. |
| [HexaFuture/HexaShell](https://github.com/HexaFuture/HexaShell) | Task-agnostic 14-tool harness; SAM 3 + geometry; four simulation benchmarks and real dual-arm results | Strong general-tool benchmark competitor. This repo is not chasing benchmark breadth; it hardens one affordable arm end to end. |
| [robot-agent-exploration/rax](https://github.com/robot-agent-exploration/rax) | Online exploration for robot agents; branch-per-environment harnesses and released real-robot session logs | Useful evidence/logging model. Its real hardware is Franka/RB-Y1 class, not $200 bus servos. |
| [autonomous-ai/openharness](https://github.com/autonomous-ai/openharness) | General coding-agent command center spanning CAD, circuits, robots, games, music | Adjacent infrastructure, not a robot manipulation harness. |

## Low-cost-arm ecosystem

| Project | Role here |
|---|---|
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | Hardware standard for SO-100/101/102; target second backend. |
| [huggingface/lerobot](https://github.com/huggingface/lerobot) | Data/imitation/VLA ecosystem and SO-100/101 bus support; source of the state adapter contract. |
| [AlexanderKoch-Koch/low_cost_robot](https://github.com/AlexanderKoch-Koch/low_cost_robot) | Ancestral low-cost arm project; useful hardware precedent. |
| [IliaLarchenko/robot_MCP](https://github.com/IliaLarchenko/robot_MCP) | SO-ARM100 MCP server; possible remote-interface inspiration. |

## Deliberate non-goals

- No claim of BEHAVIOR/RoboDojo/LIBERO leaderboard parity.
- No trained policy, demonstration corpus, or VLA baseline in this repo.
- No multi-arm benchmark fleet. The near-term goal is two honest backends:
  CreateBlock (active) and SO-100/101 (state-read adapter first, motion locked).

## Interop path

1. Keep the local deterministic primitive API stable.
2. Map Show-Harness-style semantic action units onto `scan/center/grasp/release`.
3. Expose BinceQu-RoboHarness-style visual keypoints as an optional planner input,
   while this executor remains the safety authority.
4. Use LeRobot's SO-100/101 bus semantics for the second hardware backend, but do
   not enable motion until current telemetry, FK clearance, and jaw-contact gates
   exist for that embodiment.

"""Hardware backend capability matrix.

Prevents README optimism from becoming executable truth: a backend may expose
state reads long before it is allowed to move. Motion requires the same class
of evidence that hardened the CreateBlock executor.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BackendCapability:
    backend_id: str
    executor: str
    state_read: bool
    motion: bool
    fk_clearance: bool
    current_telemetry: bool
    hardware_validation: str


CREATEBLOCK_BACKEND = BackendCapability(
    backend_id="createblock-5dof",
    executor="CreateBlockExecutor",
    state_read=True,
    motion=True,
    fk_clearance=True,
    current_telemetry=True,
    hardware_validation="real arm, 2026-11-08..15; contact primitives locked separately",
)

SO_ARM_BACKEND = BackendCapability(
    backend_id="so-arm-100-101",
    executor="SOArmExecutor",
    state_read=True,
    motion=False,
    fk_clearance=False,
    current_telemetry=False,
    hardware_validation="state adapter offline-tested; real motion intentionally locked",
)

BACKENDS = {
    capability.backend_id: capability
    for capability in (CREATEBLOCK_BACKEND, SO_ARM_BACKEND)
}

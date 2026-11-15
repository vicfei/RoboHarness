"""SO-ARM100/101 read-only adapter.

LeRobot's SO follower bus exposes six calibrated positions:
shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll (degrees)
and gripper (0-100). This adapter normalizes those observations into the common
`ArmState`, but intentionally refuses motion: the Feetech bus path currently has
no per-joint current telemetry, no shared FK clearance model, and no calibrated
jaw-contact threshold in this harness.
"""

from __future__ import annotations

import math
from typing import Protocol

from .base import Executor, UnsafeStateError
from .createblock import ArmState
from ..types import Primitive

SO_ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll")
GRIPPER_JOINT = "gripper"


class SOArmBus(Protocol):
    """Minimal LeRobot FeetechMotorsBus-compatible read interface."""

    def sync_read(self, register: str) -> dict: ...


class SOArmExecutor(Executor):
    """State-read adapter for SO-100/101; motion is deliberately unavailable."""

    def __init__(self, bus: SOArmBus, model: str = "so101"):
        if model not in ("so100", "so101"):
            raise ValueError("model must be so100 or so101")
        self.bus = bus
        self.model = model

    def read_state(self) -> ArmState:
        try:
            positions = self.bus.sync_read("Present_Position")
        except Exception as exc:
            raise UnsafeStateError("SO arm state read failed: %s" % exc) from exc
        missing = [name for name in (*SO_ARM_JOINTS, GRIPPER_JOINT)
                   if name not in positions]
        if missing:
            raise UnsafeStateError("SO arm state incomplete, missing %s" % missing)
        try:
            arm_deg = [float(positions[name]) for name in SO_ARM_JOINTS]
            gripper_percent = float(positions[GRIPPER_JOINT])
        except (TypeError, ValueError) as exc:
            raise UnsafeStateError("SO arm state values are not numeric") from exc
        if not 0.0 <= gripper_percent <= 100.0:
            raise UnsafeStateError("SO gripper outside 0-100 calibration: %.3f"
                                   % gripper_percent)
        joints = [math.radians(value) for value in arm_deg]
        # Keep the six-slot shape without claiming CreateBlock gripper semantics.
        joints.append(gripper_percent / 100.0 * (math.pi / 2.0))
        return ArmState(joints=joints, currents={}, tool_z_mm=None,
                        tool_pose=None, gripper_percent=gripper_percent)

    def execute(self, primitive: Primitive) -> dict:
        raise UnsafeStateError(
            "SO-%s motion is locked: requires current telemetry, FK clearance, "
            "and a calibrated jaw-contact gate before any command"
            % self.model[-3:])

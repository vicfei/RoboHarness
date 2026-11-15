"""SO-100/101 read-only backend tests: state normalization and locked motion."""

import math
import unittest

from roboharness.executor.backends import BACKENDS, CREATEBLOCK_BACKEND, SO_ARM_BACKEND
from roboharness.executor.createblock import UnsafeStateError
from roboharness.executor.so_arm import SOArmExecutor
from roboharness.types import MoveTo


class FakeSOBus:
    def __init__(self, positions=None, error=None):
        self.positions = positions if positions is not None else {
            "shoulder_pan": 0.0,
            "shoulder_lift": -15.0,
            "elbow_flex": -10.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 30.0,
        }
        self.error = error
        self.reads = 0

    def sync_read(self, register):
        self.reads += 1
        if self.error is not None:
            raise self.error
        assert register == "Present_Position"
        return dict(self.positions)


class TestSOArmBackend(unittest.TestCase):
    def test_state_normalizes_degrees_and_gripper_percent(self):
        bus = FakeSOBus()
        state = SOArmExecutor(bus, "so101").read_state()
        self.assertEqual(len(state.joints), 6)
        self.assertAlmostEqual(state.joints[1], math.radians(-15.0))
        self.assertAlmostEqual(state.gripper_percent, 30.0)
        self.assertAlmostEqual(state.joints[5], 30.0 / 100.0 * (math.pi / 2.0))
        self.assertIsNone(state.tool_pose)
        self.assertFalse(state.held)

    def test_incomplete_state_fuses(self):
        positions = FakeSOBus().positions
        del positions["elbow_flex"]
        ex = SOArmExecutor(FakeSOBus(positions))
        with self.assertRaisesRegex(UnsafeStateError, "elbow_flex"):
            ex.read_state()

    def test_out_of_range_gripper_fuses(self):
        positions = FakeSOBus().positions
        positions["gripper"] = 120.0
        ex = SOArmExecutor(FakeSOBus(positions))
        with self.assertRaisesRegex(UnsafeStateError, "0-100"):
            ex.read_state()

    def test_bus_failure_becomes_unsafe_state(self):
        ex = SOArmExecutor(FakeSOBus(error=TimeoutError("bridge gone")))
        with self.assertRaisesRegex(UnsafeStateError, "state read failed"):
            ex.read_state()

    def test_motion_is_locked_before_any_primitive(self):
        bus = FakeSOBus()
        ex = SOArmExecutor(bus, "so100")
        with self.assertRaisesRegex(UnsafeStateError, "motion is locked"):
            ex.execute(MoveTo("HOME"))
        self.assertEqual(bus.reads, 0)

    def test_capability_matrix_separates_state_and_motion(self):
        self.assertTrue(CREATEBLOCK_BACKEND.motion)
        self.assertTrue(CREATEBLOCK_BACKEND.current_telemetry)
        self.assertTrue(SO_ARM_BACKEND.state_read)
        self.assertFalse(SO_ARM_BACKEND.motion)
        self.assertFalse(SO_ARM_BACKEND.current_telemetry)
        self.assertEqual(set(BACKENDS), {"createblock-5dof", "so-arm-100-101"})

    def test_rejects_unknown_model(self):
        with self.assertRaises(ValueError):
            SOArmExecutor(FakeSOBus(), "so102")


if __name__ == "__main__":
    unittest.main()

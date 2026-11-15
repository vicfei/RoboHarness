"""M1 单元测试：检测器三态 / 符号修正 / 执行器安全层 / 持物判定。
全部离线可跑（合成帧 + Mock 运行器），CI 与本地均执行。"""

import unittest
from unittest import mock

try:
    import cv2
    import numpy as np
    HAS_CV = True
except ImportError:  # pragma: no cover
    HAS_CV = False

from roboharness.executor.createblock import (
    ArmState, CreateBlockExecutor, UnsafeStateError, parse_state,
    validate_cartesian_step)
from roboharness.primitives.createblock import correction_step
from roboharness.perception.hsv_detector import (
    DetectionStatus, CLOSE_WHITE_PLUG, PINK_PLATE, WHITE_PLUG, detect)


def make_state(joints, currents={2: 0.02, 3: 0.03, 5: 0.0}, z=100.0):
    return "STATE %s CUR %s Z %s" % (joints, currents, z)


def make_pose_state(pose, joints=None, current_ok=True, peaks={1: 0.3, 2: 0.4}):
    joints = joints or [0.0, -1.0, 1.05, 0.7, 0.0, 0.3]
    return "STATE %s CUR %s POSE %s MAXI %s CURRENT_OK %s" % (
        joints, {2: 0.02, 3: 0.03, 5: 0.0}, pose, peaks, current_ok)


class TestParseState(unittest.TestCase):
    def test_ok(self):
        s = parse_state(make_state([0.0, -1.0, 1.05, 0.7, 0.0, 0.169], {2: 0.32, 3: 0.5, 5: 0.1}))
        self.assertEqual(len(s.joints), 6)
        self.assertTrue(s.held)
        self.assertEqual(s.tool_z_mm, 100.0)

    def test_garbage_raises(self):
        with self.assertRaises(UnsafeStateError):
            parse_state("random noise\nno state line")

    def test_full_pose_and_motion_current(self):
        s = parse_state(make_pose_state([240.0, -37.0, 100.0, 2.2, 3.13],
                                        peaks={1: 0.31}, current_ok=False))
        self.assertEqual(s.tool_pose[:3], [240.0, -37.0, 100.0])
        self.assertEqual(s.tool_z_mm, 100.0)
        self.assertEqual(s.max_currents, {1: 0.31})
        self.assertFalse(s.motion_current_ok)


class TestCorrectionStep(unittest.TestCase):
    """11-14 实测语义：yaw+ 使目标右移(sign=+1)时，目标偏右(dx>0)需 yaw 负步。"""

    def test_yaw_left_correction(self):
        self.assertLess(correction_step(50, sign=1, step=0.05, tol=26), 0)

    def test_yaw_right_correction(self):
        self.assertGreater(correction_step(-50, sign=1, step=0.05, tol=26), 0)

    def test_within_tolerance_zero(self):
        self.assertEqual(correction_step(20, sign=1, step=0.05, tol=26), 0.0)

    def test_sign_inverted_flips(self):
        self.assertGreater(correction_step(50, sign=-1, step=0.05, tol=26), 0)


class FakeRunner:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def run(self, code, args):
        self.calls.append(args)
        if not self.responses:
            raise AssertionError("unexpected extra call: %s" % args)
        r = self.responses.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


class TestExecutorSafety(unittest.TestCase):
    def _ex(self, responses):
        return CreateBlockExecutor(FakeRunner(responses))

    def test_set_joint_tracks(self):
        ex = self._ex([make_state([0.0, -1.0, 1.05, 0.7, 0.0, 0.3])])
        st = ex.set_joint("joint3", 1.05)
        self.assertAlmostEqual(st.joints[2], 1.05, places=3)

    def test_joint_tracked_low_current_motion_accepted(self):
        ex = self._ex([make_pose_state([122.8, -72.1, 108.4, 2.197, 3.13],
                                       current_ok=True, peaks={2: 0.032})])
        state = ex.set_joint("joint3", 1.05)
        self.assertAlmostEqual(state.tool_pose[2], 108.4)

    def test_tracking_failure_aborts(self):
        ex = self._ex([make_state([0.0, -1.0, 1.45, 0.7, 0.0, 0.3])])  # 回读 1.45 ≠ 命令 1.05
        with self.assertRaises(UnsafeStateError):
            ex.set_joint("joint3", 1.05)

    def test_limit_check(self):
        ex = self._ex([])
        with self.assertRaises(UnsafeStateError):
            ex.set_joint("joint2", -0.10)  # 超 joint2 上限 -0.2269

    def test_current_spike(self):
        st = ArmState([0, -1, 1.05, 0.7, 0, 0.3], {2: 1.5, 3: 0.1})
        ex = self._ex([])
        with self.assertRaises(UnsafeStateError):
            ex.check_currents(st)

    def test_startup_rejects_collapsed(self):
        ex = self._ex([make_state([0.0, -1.9, 1.2, 1.9, 0.0, 0.5], z=10.0)])
        with self.assertRaises(UnsafeStateError):
            ex.startup_check()

    def test_startup_passes(self):
        ex = self._ex([make_state([0.0, -1.57, 1.57, 0.0, 0.0, 0.3], z=180.0)])
        st = ex.startup_check()
        self.assertTrue(st.tool_z_mm > 30)

    def test_startup_rejects_joint_outside_limits_even_when_high(self):
        ex = self._ex([make_state([-0.156, -2.766, 1.007, 1.787, -0.02, -0.124], z=169.0)])
        with self.assertRaisesRegex(UnsafeStateError, "超出限位"):
            ex.startup_check()

    def test_joint_motion_current_gate(self):
        ex = self._ex([make_pose_state([150.0, 0.0, 180.0, 2.0, 3.13],
                                       current_ok=False, peaks={2: 0.096})])
        with self.assertRaisesRegex(UnsafeStateError, "运动电流门"):
            ex.set_joint("joint3", 1.05)

    def test_gripper_close_reports_current_peak_and_held(self):
        joints = [0.0, -1.0, 1.05, 0.7, 0.0, 0.169]
        ex = self._ex([make_pose_state([122.8, -72.1, 50.0, 2.5, 3.13],
                                       joints=joints, current_ok=None, peaks={5: 0.5})])
        state = ex.set_gripper(0.0)
        self.assertTrue(state.held)
        self.assertEqual(state.max_currents, {5: 0.5})

    def test_coordinated_recovery_disabled_after_real_failure(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "真机验收否决"):
            ex.recover_low_pose(-0.062, 0.090, -0.062)
        self.assertEqual(ex.runner.calls, [])

    def test_contact_approach_disabled_until_jaw_calibration(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "默认禁用"):
            ex.move_contact_approach([-0.532, -1.848, 0.978, 1.769, -0.019])
        self.assertEqual(ex.runner.calls, [])

    def test_contact_delta_disabled_until_jaw_calibration(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "默认禁用"):
            ex.move_contact_delta(0.0, 0.0, -5.0)
        self.assertEqual(ex.runner.calls, [])


class TestFreeSpacePreparation(unittest.TestCase):
    OUT_JOINTS = [-0.156, -2.766, 1.007, 1.787, -0.020, -0.124]
    LEGAL_JOINTS = [-0.156, -2.650, 1.007, 1.787, -0.020, -0.124]
    OUT_POSE = [95.49, -14.86, 169.02, 1.5985, 3.1215]
    LEGAL_POSE = [106.16, -16.70, 157.11, 1.715, 3.122]

    def setUp(self):
        patcher = mock.patch("roboharness.executor.createblock.CONTACT_APPROACH_AVAILABLE", True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _ex(self, responses):
        return CreateBlockExecutor(FakeRunner(responses))

    def test_prepare_automatically_legalizes_high_joint(self):
        ex = self._ex([
            make_pose_state(self.OUT_POSE, joints=self.OUT_JOINTS, current_ok=None),
            make_pose_state(self.LEGAL_POSE, joints=self.LEGAL_JOINTS, peaks={1: 0.032}),
            make_pose_state(self.LEGAL_POSE, joints=self.LEGAL_JOINTS, current_ok=None),
        ])
        state = ex.prepare()
        self.assertEqual(state.joints[1], -2.650)
        self.assertEqual(ex.runner.calls[1][0], "free_space_move")
        self.assertEqual(ex.runner.calls[1][2], "-2.65000")
        self.assertEqual(ex.runner.calls[1][-2], "3000")

    def test_prepare_accepts_boundary_read_within_deadband(self):
        joints = [-0.156, -2.665, 1.022, 1.794, -0.020, -0.126]
        pose = [105.53, -16.62, 154.58, 1.7235, 3.123]
        ex = self._ex([make_pose_state(pose, joints=joints, current_ok=None)])
        state = ex.prepare()
        self.assertAlmostEqual(state.tool_pose[2], 154.58)
        self.assertEqual(ex.runner.calls, [[]])

    def test_prepare_rejects_out_of_range_joint_near_table(self):
        pose = [95.49, -14.86, 50.0, 1.5985, 3.1215]
        ex = self._ex([make_pose_state(pose, joints=self.OUT_JOINTS, current_ok=None)])
        with self.assertRaisesRegex(UnsafeStateError, "低于 120mm"):
            ex.prepare()
        self.assertEqual(ex.runner.calls, [[]])

    def test_free_space_current_failure_aborts(self):
        start_joints = [0.0, -2.0, 1.0, 1.0, 0.0, 0.975]
        target = [0.0, -1.97, 0.47, 1.75, 0.0]
        end_joints = target + [0.975]
        responses = [make_pose_state([150, 0, 180, 2.0, 3.13], joints=start_joints,
                                     current_ok=None),
                     make_pose_state([150, 0, 175, 2.0, 3.13], joints=end_joints,
                                     current_ok=False, peaks={1: 0.08})]
        ex = self._ex(responses)
        with self.assertRaisesRegex(UnsafeStateError, "反馈门未过"):
            ex.move_free_space(target)

    def test_free_space_noop_within_tracking_tolerance_passes(self):
        joints = [-0.020, -1.948, 0.515, 1.783, -0.018, -0.124]
        target = [0.0, -1.97, 0.47, 1.75, 0.0]
        pose = [155.08, -3.06, 156.15, 1.9212, 3.123]
        ex = self._ex([make_pose_state(pose, joints=joints, current_ok=None),
                       make_pose_state(pose, joints=joints, current_ok=True,
                                       peaks={0: 0.016})])
        state = ex.move_free_space(target)
        self.assertAlmostEqual(state.tool_pose[2], 156.15)

    def test_observation_move_supports_90mm_clearance(self):
        start_joints = [-0.013, -1.948, 0.515, 1.783, -0.020, -0.124]
        target = [0.0, -1.8, 0.6, 1.9, 0.0]
        responses = [make_pose_state([155.1, -1.95, 156.15, 1.921, 3.123],
                                     joints=start_joints, current_ok=None),
                     make_pose_state([151.2, 0.0, 99.4, 2.271, 3.13],
                                     joints=target + [-0.124], peaks={1: 0.032})]
        ex = self._ex(responses)
        state = ex.move_free_space(target, min_clearance_mm=90.0)
        self.assertAlmostEqual(state.tool_pose[2], 99.4)
        self.assertEqual(ex.runner.calls[1][-1], "90.0")

    def test_observation_clearance_below_90_rejected_before_read(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "clearance"):
            ex.move_free_space([0.0, -1.8, 0.6, 1.9, 0.0], min_clearance_mm=80.0)
        self.assertEqual(ex.runner.calls, [])

    def test_contact_approach_supports_task_clearance(self):
        start_joints = [-0.531, -1.967, 0.890, 1.915, -0.018, 0.973]
        target = [-0.532, -1.848, 0.978, 1.769, -0.019]
        responses = [make_pose_state([111.45, -65.45, 73.05, 2.409, 3.123],
                                     joints=start_joints, current_ok=None),
                     make_pose_state([122.9, -72.3, 60.0, 2.47, 3.123],
                                     joints=target + [0.973], peaks={2: 0.032})]
        ex = self._ex(responses)
        state = ex.move_contact_approach(target, min_clearance_mm=35.0)
        self.assertAlmostEqual(state.tool_pose[2], 60.0)
        self.assertEqual(ex.runner.calls[1][-1], "35.0")

    def test_contact_approach_rejects_below_absolute_floor(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "clearance"):
            ex.move_contact_approach([-0.532, -1.848, 0.978, 1.769, -0.019],
                                     min_clearance_mm=25.0)
        self.assertEqual(ex.runner.calls, [])

    def test_free_space_final_height_failure_aborts(self):
        start_joints = [0.0, -2.0, 1.0, 1.0, 0.0, 0.975]
        target = [0.0, -1.97, 0.47, 1.75, 0.0]
        responses = [make_pose_state([150, 0, 180, 2.0, 3.13], joints=start_joints,
                                     current_ok=None),
                     make_pose_state([150, 0, 119, 2.0, 3.13], joints=target + [0.975])]
        ex = self._ex(responses)
        with self.assertRaisesRegex(UnsafeStateError, "高度门"):
            ex.move_free_space(target)

    def test_free_space_target_limit_rejected_before_read(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "超出限位"):
            ex.move_free_space([0.0, -3.0, 0.47, 1.75, 0.0])
        self.assertEqual(ex.runner.calls, [])


class TestCartesianSafety(unittest.TestCase):
    START = [240.0, -37.0, 100.0, 2.2, 3.13]

    def setUp(self):
        patcher = mock.patch("roboharness.executor.createblock.CONTACT_APPROACH_AVAILABLE", True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _ex(self, responses):
        return CreateBlockExecutor(FakeRunner(responses))

    def test_low_pose_locks_before_motion(self):
        ex = self._ex([make_pose_state([240.0, -37.0, 40.0, 2.2, 3.13],
                                       current_ok=None)])
        with self.assertRaisesRegex(UnsafeStateError, "低于 70mm"):
            ex.move_tool_delta(0.0, 0.0, 5.0)
        self.assertEqual(ex.runner.calls, [[]])

    def test_high_lift_passes(self):
        end = [240.0, -37.0, 105.0, 2.2, 3.13]
        ex = self._ex([make_pose_state(self.START), make_pose_state(end)])
        state = ex.move_tool_delta(0.0, 0.0, 5.0)
        self.assertEqual(state.tool_z_mm, 105.0)
        self.assertEqual(ex.runner.calls[1], ["tool_step", "0.0000", "0.0000", "5.0000"])

    def test_reversed_lift_aborts(self):
        end = [240.0, -37.0, 99.0, 2.2, 3.13]
        with self.assertRaisesRegex(UnsafeStateError, "抬升方向反转"):
            validate_cartesian_step(self.START, end, [0.0, 0.0, 5.0])

    def test_lateral_step_keeps_z(self):
        end = [240.0, -32.0, 100.2, 2.2, 3.13]
        ex = self._ex([make_pose_state(self.START), make_pose_state(end)])
        state = ex.move_tool_delta(0.0, 5.0, 0.0)
        self.assertAlmostEqual(state.tool_pose[1], -32.0)

    def test_lateral_z_drift_aborts(self):
        end = [240.0, -33.0, 96.8, 2.2, 3.13]
        with self.assertRaisesRegex(UnsafeStateError, "z 锁失效"):
            validate_cartesian_step(self.START, end, [0.0, 4.0, 0.0])

    def test_cartesian_current_failure_aborts(self):
        end = [240.0, -37.0, 105.0, 2.2, 3.13]
        ex = self._ex([make_pose_state(self.START),
                       make_pose_state(end, current_ok=False, peaks={2: 0.096})])
        with self.assertRaisesRegex(UnsafeStateError, "电流门未过"):
            ex.move_tool_delta(0.0, 0.0, 5.0)

    def test_request_over_six_mm_rejected_before_read(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "6mm"):
            ex.move_tool_delta(0.0, 0.0, 7.0)
        self.assertEqual(ex.runner.calls, [])

    def test_contact_delta_descends_with_low_current_tracking(self):
        start = [108.6, -63.8, 50.0, 2.1, 3.13]
        end = [108.6, -63.8, 45.0, 2.1, 3.13]
        ex = self._ex([make_pose_state(start, current_ok=None),
                       make_pose_state(end, current_ok=True, peaks={2: 0.032})])
        state = ex.move_contact_delta(0.0, 0.0, -5.0)
        self.assertEqual(state.tool_pose[2], 45.0)
        self.assertEqual(ex.runner.calls[1][-1], "35.0")

    def test_contact_delta_rejects_target_below_clearance(self):
        ex = self._ex([make_pose_state([108.6, -63.8, 38.0, 2.1, 3.13], current_ok=None)])
        with self.assertRaisesRegex(UnsafeStateError, "clearance"):
            ex.move_contact_delta(0.0, 0.0, -5.0)
        self.assertEqual(ex.runner.calls, [[]])


class TestCoordinatedRecovery(unittest.TestCase):
    START_JOINTS = [-0.154, -1.305, 1.250, 0.745, -0.012, 0.975]
    END_JOINTS = [-0.154, -1.367, 1.340, 0.683, -0.012, 0.975]
    START = [237.04, -36.45, 32.94, 2.2626, 3.13]
    END = [236.24, -36.33, 36.89, 2.227, 3.13]
    DELTAS = (-0.062, 0.090, -0.062)

    def setUp(self):
        patcher = mock.patch("roboharness.executor.createblock.LOW_POSE_RECOVERY_AVAILABLE", True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _ex(self, responses):
        return CreateBlockExecutor(FakeRunner(responses))

    def _responses(self, end=None, joints=None, current_ok=True, peaks={1: 0.4}):
        return [make_pose_state(self.START, joints=self.START_JOINTS, current_ok=None),
                make_pose_state(end or self.END, joints=joints or self.END_JOINTS,
                                current_ok=current_ok, peaks=peaks)]

    def test_coordinated_recovery_passes(self):
        ex = self._ex(self._responses())
        state = ex.recover_low_pose(*self.DELTAS)
        self.assertAlmostEqual(state.tool_pose[2], 36.89)
        self.assertEqual(ex.runner.calls[1],
                         ["recovery_step", "-0.0620", "0.0900", "-0.0620"])

    def test_recovery_current_failure_aborts(self):
        ex = self._ex(self._responses(end=self.START, joints=self.START_JOINTS,
                                      current_ok=False, peaks={1: 0.0}))
        with self.assertRaisesRegex(UnsafeStateError, "电流门未过"):
            ex.recover_low_pose(*self.DELTAS)

    def test_recovery_reversed_motion_aborts(self):
        end = [237.04, -36.45, 32.50, 2.2626, 3.13]
        ex = self._ex(self._responses(end=end))
        with self.assertRaisesRegex(UnsafeStateError, "未有效抬升"):
            ex.recover_low_pose(*self.DELTAS)

    def test_recovery_tracking_failure_aborts(self):
        joints = [-0.154, -1.367, 1.450, 0.683, -0.012, 0.975]
        ex = self._ex(self._responses(joints=joints))
        with self.assertRaisesRegex(UnsafeStateError, "0.065rad"):
            ex.recover_low_pose(*self.DELTAS)

    def test_recovery_rejects_high_start(self):
        start = [237.04, -36.45, 70.0, 2.2626, 3.13]
        ex = self._ex([make_pose_state(start, joints=self.START_JOINTS, current_ok=None)])
        with self.assertRaisesRegex(UnsafeStateError, "25,70"):
            ex.recover_low_pose(*self.DELTAS)
        self.assertEqual(ex.runner.calls, [[]])

    def test_recovery_rejects_large_joint_request_before_read(self):
        ex = self._ex([])
        with self.assertRaisesRegex(UnsafeStateError, "0.12rad"):
            ex.recover_low_pose(0.13, 0.09, -0.062)
        self.assertEqual(ex.runner.calls, [])


@unittest.skipUnless(HAS_CV, "numpy/cv2 不可用")
class TestDetectorThreeState(unittest.TestCase):
    def _frame(self, blobs, size=(480, 640), bg=(45, 60, 90)):  # BGR 褐桌暗底
        img = np.full((size[0], size[1], 3), bg, np.uint8)
        for (cx, cy, w, h, color) in blobs:
            cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2), color, -1)
        return img

    def test_found_strong_plug(self):
        img = self._frame([(373, 298, 90, 33, (255, 250, 245))])
        det = detect(img, WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.FOUND)
        self.assertAlmostEqual(det.center[0], 373, delta=4)

    def test_found_large_close_plug(self):
        img = self._frame([(320, 240, 140, 160, (255, 250, 245))])
        det = detect(img, WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.FOUND)
        self.assertGreater(det.area, 15000)

    def test_close_contact_plug_may_touch_top(self):
        img = self._frame([(350, 120, 240, 240, (255, 250, 245))])
        det = detect(img, CLOSE_WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.FOUND)
        self.assertGreater(det.area, 30000)

    def test_none_empty_scene(self):
        det = detect(self._frame([]), WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.NONE)

    def test_uncertain_weak_blob(self):
        img = self._frame([(300, 300, 34, 22, (200, 195, 190))])  # 面积小分低
        det = detect(img, WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.UNCERTAIN)

    def test_border_glare_excluded(self):
        img = self._frame([(620, 30, 40, 40, (255, 255, 255))])  # 贴右上边
        det = detect(img, WHITE_PLUG)
        self.assertEqual(det.status, DetectionStatus.NONE)

    def test_large_plate_may_touch_border(self):
        img = self._frame([(0, 150, 220, 220, (200, 120, 200))])  # 粉盘触左边
        det = detect(img, PINK_PLATE)
        self.assertEqual(det.status, DetectionStatus.FOUND)


if __name__ == "__main__":
    unittest.main()

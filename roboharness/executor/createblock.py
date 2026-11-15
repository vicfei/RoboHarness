"""CreateBlockExecutor：真机执行器 + 安全层（docs/08 §10.3 六条铁律的代码化）。

通道（11-08~14 验证）：SSH(paramiko) → Nano → Arm5DoFUServo(/dev/ttyTHS1)。
M1 用 SSHCodeRunner（今日实战模式：上传任务码→远端执行→解析 STATE 行）；
harness 迁上 Nano 后换 LocalCodeRunner，其余零改动。
"""

from __future__ import annotations

import ast
import math
import re
from dataclasses import dataclass, field
from typing import Protocol

from .base import Executor, UnsafeStateError
from ..types import Primitive


# ---- 关节限位（厂商 arm5dof_kinematic 实测值，rad）----
JOINT_LIMITS = {
    "joint1": (-1.5708, 1.5708),
    "joint2": (-2.65, -0.2269),
    "joint3": (-0.7854, 2.18166),
    "joint4": (-2.0944, 2.0944),
    "joint5": (-1.5708, 1.5708),
    "left_gripper_joint": (0.0, 1.5708),
}
JOINT_ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "left_gripper_joint"]

# 安全层常数（11-14 实战标定）
CURRENT_SPIKE_A = 1.2        # 任一关节电流超此值 → 立即回退
DRIVER_DEAD_A = 0.005        # 电流低于此值（带载关节）→ 驱动保护判定
ANGLE_TRACK_TOL = 0.25       # 关节回读与命令差超此值 → 跟踪失败
STARTUP_MIN_TOOL_Z_MM = 30.0 # 启动自检：工具低于此高度 = 趴伏构型拒任务
CARTESIAN_MIN_TOOL_Z_MM = 70.0  # 近桌禁用笛卡尔 IK 微步；恢复走独立关节空间 TaskSpec
FREESPACE_MIN_TOOL_Z_MM = 120.0  # 高位自由空间：允许自动合法化/转场，整段路径不得跌破此高度
OBSERVATION_MIN_TOOL_Z_MM = 90.0  # 纯观察可降到 90mm；抓取接触另走原语
CONTACT_MIN_TOOL_Z_MM = 30.0  # 接触接近的绝对下限，具体任务只能更高
CONTACT_APPROACH_AVAILABLE = False  # 11-15 多次空合且高度欠冲；爪口几何标定前默认禁用
JOINT_LIMIT_READ_TOL_RAD = 0.03  # 边界回读容差；避免舵机死区内反复合法化
CARTESIAN_MAX_STEP_MM = 6.0
CARTESIAN_Z_LOCK_TOL_MM = 3.0
RECOVERY_ANGLE_TRACK_TOL = 0.065  # 约 3°；恢复步另看笛卡尔结果，不用旧 0.25rad 宽容差
LOW_POSE_RECOVERY_AVAILABLE = False  # 11-15 真机异步响应导致反降；默认禁用，禁止自动重试
HELD_MIN_GRIP_RAD = 0.12     # 夹爪角高于此值 = 持物（空抓约 0.03）


@dataclass
class ArmState:
    joints: list = field(default_factory=list)            # 6 关节 rad
    currents: dict = field(default_factory=dict)          # {servo_id: A}
    tool_z_mm: float | None = None
    tool_pose: list | None = None                         # [x, y, z, pitch, roll]，mm/rad
    max_currents: dict = field(default_factory=dict)      # 本次运动采样峰值 {servo_id: A}
    motion_current_ok: bool | None = None                 # 真机运动电流门；离线旧记录可为 None
    gripper_percent: float | None = None                  # 0-100 归一化爪（SO 臂）；未标定持物阈值

    @property
    def grip_angle(self) -> float:
        return self.joints[5] if len(self.joints) > 5 else 0.0

    @property
    def held(self) -> bool:
        if self.gripper_percent is not None:
            return False  # SO 0-100 码没有物体阈值；持物必须另给电流/视觉证据
        return self.grip_angle > HELD_MIN_GRIP_RAD


# ---- 远端执行的任务码模板（在 Nano 的 nvdli-data SDK 环境里运行）----
ARM_CODE = r'''
import os, sys, time
os.chdir("/home/jetson/nvdli-data")
sys.path.insert(0, os.getcwd())
import numpy as np
from arm5dof_uservo import Arm5DoFUServo
from config import JOINT2SERVO_K


def main():
    arm = Arm5DoFUServo(config_folder="config", is_init_pose=False)
    arm.set_servo_velocity(__VELOCITY__)
    mgr = arm.uservo
    for sid in range(6):
        mgr.ping(sid)
    motion = {"max": {}, "last": 0.0}

    def sample(sids, force=False):
        now = time.time()
        if not force and now - motion["last"] < 0.15:
            return
        motion["last"] = now
        for sid in set(sids):
            value = mgr.query_current(sid)
            motion["max"][sid] = max(value, motion["max"].get(sid, 0.0))
            if value > 1.2:
                raise RuntimeError("overcurrent servo=%d current=%.3f" % (sid, value))

    def emit_state(current_ok=None):
        joints = np.round(arm.get_joint_angle_list(), 3).tolist()
        currents = {}
        for sid in (2, 3, 5):
            try:
                currents[sid] = round(mgr.query_current(sid), 3)
            except Exception:
                currents[sid] = -1.0
        pose = [round(float(v), 4) for v in arm.get_tool_pose()]
        peaks = {sid: round(value, 3) for sid, value in motion["max"].items()}
        print("STATE %s CUR %s POSE %s MAXI %s CURRENT_OK %s" % (
            joints, currents, pose, peaks, current_ok))

    cmd = sys.argv[1] if len(sys.argv) > 1 else "read"
    current_ok = None
    if cmd == "set":
        name, target = sys.argv[2], float(sys.argv[3])
        interval = int(sys.argv[4]) if len(sys.argv) > 4 else 1000
        sid = arm.joint_name2servo_id(name)
        start = arm.get_joint_angle(name)
        arm.set_joint_angle(name, target, interval=interval, is_wait=True,
                            callback=lambda: sample((sid, 2, 3)))
        sample((sid, 2, 3), force=True)
        time.sleep(1.0)
        end = arm.get_joint_angle(name)
        requested = abs(target - start)
        moved = abs(end - start)
        tracked = abs(end - target) < 0.12
        current_ok = True if requested < 0.01 else bool(moved > 0.005 and tracked)
    elif cmd == "tool_step":
        delta = np.array([float(v) for v in sys.argv[2:5]], dtype=float)
        if np.linalg.norm(delta) <= 0.0 or np.linalg.norm(delta) > 6.0:
            raise RuntimeError("tool_step request violates 6mm gate: %s" % delta.tolist())
        start_pose = np.array(arm.get_tool_pose(), dtype=float)
        start_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        min_z = float(sys.argv[5]) if len(sys.argv) > 5 else 70.0
        if not 35.0 <= min_z <= 70.0:
            raise RuntimeError("tool_step clearance outside [35,70]mm")
        target_pose = start_pose.copy()
        target_pose[:3] += delta
        if start_pose[2] < min_z or target_pose[2] < min_z:
            raise RuntimeError("tool_step violates clearance %.1fmm" % min_z)
        ok, _ = arm.inverse_kinematic(
            arm.adjust_tool_posi(target_pose[:3]),
            tool_pitch=start_pose[3], tool_roll=start_pose[4])
        if not ok:
            raise RuntimeError("tool_step IK rejected")

        def monitor_tool():
            sample((1, 2, 3))
            joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
            pose = np.array(arm.kinematic.forward_kinematic_v2(joints), dtype=float)
            actual = pose[:3] - start_pose[:3]
            if np.linalg.norm(actual) > 6.0:
                raise RuntimeError("tool_step exceeded 6mm: %s" % actual.tolist())
            if delta[2] > 0 and actual[2] < -1.0:
                raise RuntimeError("lift reversed: %s" % actual.tolist())
            if delta[2] < 0 and actual[2] > 1.0:
                raise RuntimeError("descent reversed: %s" % actual.tolist())
            if delta[2] == 0 and abs(actual[2]) > 3.0:
                raise RuntimeError("z-lock drift: %s" % actual.tolist())
            if delta[:2].any() and delta[2] == 0:
                along = float(np.dot(actual[:2], delta[:2]) / np.linalg.norm(delta[:2]))
                if along < -1.0:
                    raise RuntimeError("lateral step reversed: %s" % actual.tolist())
            if not delta[:2].any() and np.linalg.norm(actual[:2]) > 3.0:
                raise RuntimeError("vertical step lateral drift: %s" % actual.tolist())

        arm.set_tool_pose(target_pose[:3], tool_pitch=start_pose[3],
                          tool_roll=start_pose[4], T=1.2,
                          is_soft_move=True, callback=monitor_tool)
        sample((1, 2, 3), force=True)
        time.sleep(1.0)
        end_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        moved = float(np.max(np.abs(end_joints - start_joints)))
        current_ok = bool(moved > 0.005)
    elif cmd == "free_space_move":
        target_joints = np.array([float(v) for v in sys.argv[2:7]], dtype=float)
        interval = int(sys.argv[7]) if len(sys.argv) > 7 else 3000
        min_z = float(sys.argv[8]) if len(sys.argv) > 8 else 120.0
        if not 30.0 <= min_z <= 120.0:
            raise RuntimeError("unsupported clearance %.1fmm" % min_z)
        start_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        start_pose = np.array(arm.kinematic.forward_kinematic_v2(start_joints), dtype=float)
        if start_pose[2] < min_z:
            raise RuntimeError("move requires start z>=%.1fmm" % min_z)
        for index in range(5):
            if not arm.theta_lowerb[index] <= target_joints[index] <= arm.theta_upperb[index]:
                raise RuntimeError("free-space target exceeds joint limit at %d" % index)
        for alpha in np.linspace(0.0, 1.0, 21):
            joints = start_joints + (target_joints - start_joints) * alpha
            pose = np.array(arm.kinematic.forward_kinematic_v2(joints), dtype=float)
            if pose[2] < min_z:
                raise RuntimeError("path drops below %.1fmm at %.2f" % (min_z, alpha))

        def monitor_free_space():
            joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
            try:
                sample((0, 1, 2, 3, 4))
                pose = np.array(arm.kinematic.forward_kinematic_v2(joints), dtype=float)
                if pose[2] < min_z:
                    raise RuntimeError("motion crossed z=%.1fmm" % min_z)
            except Exception:
                arm.set_joint_angle_list(joints.tolist(), interval=120, is_wait=False)
                raise

        arm.set_joint_angle_list(target_joints.tolist(), interval=interval,
                                 is_wait=True, callback=monitor_free_space)
        sample((0, 1, 2, 3, 4), force=True)
        time.sleep(1.0)
        end_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        requested = float(np.max(np.abs(target_joints - start_joints)))
        tracked = float(np.max(np.abs(end_joints - target_joints))) < 0.12
        current_ok = True if requested < 0.01 else bool(tracked)
    elif cmd == "recovery_step":
        deltas = np.array([float(v) for v in sys.argv[2:5]], dtype=float)
        if np.max(np.abs(deltas)) > 0.12 or np.linalg.norm(deltas) <= 0.0:
            raise RuntimeError("recovery delta outside reviewed bounds: %s" % deltas.tolist())
        raw_degrees = np.abs(deltas * np.array(JOINT2SERVO_K[1:4]))
        if np.min(raw_degrees) < 3.2:
            raise RuntimeError("recovery command inside servo deadband: raw=%s" % raw_degrees.tolist())
        start_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        start_pose = np.array(arm.kinematic.forward_kinematic_v2(start_joints), dtype=float)
        if not 25.0 <= start_pose[2] < 70.0:
            raise RuntimeError("recovery start z outside [25,70): %.3f" % start_pose[2])
        target_joints = start_joints.copy()
        target_joints[1:4] += deltas
        for index in (1, 2, 3):
            if not arm.theta_lowerb[index] <= target_joints[index] <= arm.theta_upperb[index]:
                raise RuntimeError("recovery target exceeds joint limit at %d" % index)
        for alpha in np.linspace(0.0, 1.0, 21):
            joints = start_joints.copy()
            joints[1:4] += deltas * alpha
            pose = np.array(arm.kinematic.forward_kinematic_v2(joints), dtype=float)
            actual = pose[:3] - start_pose[:3]
            if actual[2] < -0.5 or np.linalg.norm(actual) > 6.0:
                raise RuntimeError("recovery path violates Cartesian gate at %.2f" % alpha)
            if np.linalg.norm(actual[:2]) > 3.0:
                raise RuntimeError("recovery path lateral drift at %.2f" % alpha)
        target_pose = np.array(arm.kinematic.forward_kinematic_v2(target_joints), dtype=float)
        expected = target_pose[:3] - start_pose[:3]
        if expected[2] < 2.0 or abs(target_pose[3] - start_pose[3]) > 0.15:
            raise RuntimeError("recovery endpoint is not a small stable lift: %s" % expected.tolist())
        arm.trajectory_callback_period = 1

        def monitor_recovery():
            sample((1, 2, 3))
            joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
            pose = np.array(arm.kinematic.forward_kinematic_v2(joints), dtype=float)
            actual = pose[:3] - start_pose[:3]
            if actual[2] < -0.5:
                raise RuntimeError("recovery reversed: %s" % actual.tolist())
            if np.linalg.norm(actual) > 6.0:
                raise RuntimeError("recovery exceeded 6mm: %s" % actual.tolist())
            if np.linalg.norm(actual[:2]) > 3.0:
                raise RuntimeError("recovery lateral drift: %s" % actual.tolist())

        arm.set_joint_angle_list_soft(target_joints.tolist(), T=1.5,
                                      callback=monitor_recovery)
        sample((1, 2, 3), force=True)
        time.sleep(1.0)
        end_joints = np.array(arm.get_joint_angle_list()[:5], dtype=float)
        moved = float(np.max(np.abs(end_joints - start_joints)))
        current_ok = bool(max(motion["max"].values(), default=0.0) > 0.2 and moved > 0.005)
    elif cmd == "grip":
        arm.set_gripper_angle(angle=float(sys.argv[2]), T=1.0,
                              callback=lambda: sample((5,)))
        sample((5,), force=True)
        time.sleep(0.4)
    elif cmd == "home":
        arm.home()
        time.sleep(0.4)
    emit_state(current_ok)


main()
'''


class CodeRunner(Protocol):
    def run(self, code: str, args: list) -> str: ...


class SSHCodeRunner:
    """paramiko 上传任务码到 Nano 执行（11-14 当日全链路验证的模式）。"""

    def __init__(self, host: str, key_path: str, user: str = "jetson", timeout: int = 90):
        self.host, self.key_path, self.user, self.timeout = host, key_path, user, timeout

    def run(self, code: str, args: list) -> str:
        import paramiko  # 延迟导入：WSL 侧测试不需要

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        key = paramiko.Ed25519Key.from_private_key_file(self.key_path)
        client.connect(self.host, username=self.user, pkey=key, timeout=8,
                       look_for_keys=False, allow_agent=False)
        try:
            sftp = client.open_sftp()
            with sftp.open("/tmp/saa_job.py", "w") as f:
                f.write(code)
            sftp.close()
            _, stdout, _ = client.exec_command(
                "cd /home/jetson/nvdli-data && timeout %d python3 -u /tmp/saa_job.py %s 2>&1"
                % (self.timeout, " ".join(args)), timeout=self.timeout + 10)
            return stdout.read().decode("utf-8", "replace")
        finally:
            client.close()


class LocalCodeRunner:
    """本地子进程执行（harness 迁上 Nano 后使用）。"""

    def __init__(self, workdir: str = "/home/jetson/nvdli-data"):
        self.workdir = workdir

    def run(self, code: str, args: list) -> str:
        import subprocess

        r = subprocess.run(["python3", "-u", "-", *args], input=code, cwd=self.workdir,
                           capture_output=True, text=True, timeout=90)
        return r.stdout + r.stderr


def parse_state(out: str) -> ArmState:
    """解析新旧 STATE 格式；真机新格式包含完整位姿与运动电流峰值。"""
    for line in out.splitlines():
        if not line.startswith("STATE "):
            continue
        try:
            jpart, rest = line.split("STATE ", 1)[1].split(" CUR ", 1)
            parts = re.split(r" (POSE|Z|MAXI|CURRENT_OK) ", rest)
            cpart = parts[0]
            fields = dict(zip(parts[1::2], parts[2::2]))
            joints = [float(v) for v in ast.literal_eval(jpart)]
            currents = {int(k): float(v) for k, v in ast.literal_eval(cpart).items()}
            pose = None
            if "POSE" in fields:
                pose = [float(v) for v in ast.literal_eval(fields["POSE"])]
            z = pose[2] if pose is not None else (
                float(fields["Z"]) if fields.get("Z") not in ("", "None", None) else None)
            peaks = ({int(k): float(v) for k, v in ast.literal_eval(fields["MAXI"]).items()}
                     if "MAXI" in fields else {})
            current_ok = (fields["CURRENT_OK"] == "True"
                          if fields.get("CURRENT_OK") in ("True", "False") else None)
            return ArmState(joints=joints, currents=currents, tool_z_mm=z,
                            tool_pose=pose, max_currents=peaks,
                            motion_current_ok=current_ok)
        except (ValueError, IndexError, SyntaxError, TypeError):
            continue
    raise UnsafeStateError("STATE 解析失败（臂无响应或驱动异常）: %r" % out[-240:])


def validate_cartesian_step(start_pose: list, end_pose: list, requested_delta: list) -> None:
    """笛卡尔微步硬门：方向、步长、正交漂移与 z 锁任一不过都熔断。"""
    if len(start_pose) < 3 or len(end_pose) < 3 or len(requested_delta) != 3:
        raise UnsafeStateError("笛卡尔校验需要完整 xyz 起点、终点和三维请求")
    request_norm = math.sqrt(sum(float(v) ** 2 for v in requested_delta))
    if request_norm <= 0.0 or request_norm > CARTESIAN_MAX_STEP_MM:
        raise UnsafeStateError("请求位移 %.3fmm 超过 (0, %.1f]mm 微步范围" % (
            request_norm, CARTESIAN_MAX_STEP_MM))
    actual = [float(end_pose[i]) - float(start_pose[i]) for i in range(3)]
    distance = math.sqrt(sum(v ** 2 for v in actual))
    if distance > CARTESIAN_MAX_STEP_MM:
        raise UnsafeStateError("实际位移 %.3fmm 超过 6mm 硬顶：%s" % (distance, actual))
    if requested_delta[2] > 0 and actual[2] <= 0:
        raise UnsafeStateError("抬升方向反转：dz=%.3fmm" % actual[2])
    if requested_delta[2] < 0 and actual[2] >= 0:
        raise UnsafeStateError("下降方向反转：dz=%.3fmm" % actual[2])
    if requested_delta[2] == 0 and abs(actual[2]) > CARTESIAN_Z_LOCK_TOL_MM:
        raise UnsafeStateError("侧移 z 锁失效：dz=%.3fmm" % actual[2])
    along = sum(actual[i] * float(requested_delta[i]) for i in range(3)) / request_norm
    min_progress = max(1.0, request_norm * 0.4)
    if along < min_progress:
        raise UnsafeStateError("实际运动未沿请求方向：投影 %.3fmm < %.3fmm，delta=%s" % (
            along, min_progress, actual))
    orthogonal = math.sqrt(max(0.0, distance ** 2 - along ** 2))
    if orthogonal > CARTESIAN_Z_LOCK_TOL_MM:
        raise UnsafeStateError("正交漂移 %.3fmm 超过 3mm：%s" % (orthogonal, actual))
    if end_pose[2] < STARTUP_MIN_TOOL_Z_MM:
        raise UnsafeStateError("运动后工具高度 %.3fmm 进入趴伏危险区" % end_pose[2])


class CreateBlockExecutor(Executor):
    """安全层内置：双反馈门 / 电流熔断 / 启动自检 / 限位校验。"""

    def __init__(self, runner: CodeRunner, velocity: int = 50):
        self.runner = runner
        self.velocity = velocity
        self._code = ARM_CODE.replace("__VELOCITY__", str(velocity))
        self._startup_ok = False

    # -- 基础指令 --
    def _exec(self, args: list) -> ArmState:
        return parse_state(self.runner.run(self._code, args))

    def read_state(self) -> ArmState:
        return self._exec([])

    def set_joint(self, name: str, rad: float, interval_ms: int = 1000) -> ArmState:
        if name not in JOINT_LIMITS:
            raise ValueError("未知关节 %s" % name)
        lo, hi = JOINT_LIMITS[name]
        if not (lo - 1e-6 <= rad <= hi + 1e-6):
            raise UnsafeStateError("关节 %s 目标 %.3f 超限位 [%.3f, %.3f]" % (name, rad, lo, hi))
        idx = JOINT_ORDER.index(name)
        state = self._exec(["set", name, "%.4f" % rad, str(interval_ms)])
        # 双反馈门：角度跟踪
        err = abs(state.joints[idx] - rad)
        if err >= ANGLE_TRACK_TOL:
            raise UnsafeStateError(
                "关节 %s 跟踪失败: 命令 %.3f 回读 %.3f（差 %.3f）——疑似卡阻/驱动保护"
                % (name, rad, state.joints[idx], err))
        if state.motion_current_ok is False:
            raise UnsafeStateError(
                "关节 %s 运动电流门未过：峰值 %s —— 禁止继续发动作" % (
                    name, state.max_currents))
        return state

    def move_tool_delta(self, dx: float, dy: float, dz: float) -> ArmState:
        """高位受控笛卡尔微步；近桌低位只许另走关节空间恢复流程。"""
        delta = [float(dx), float(dy), float(dz)]
        request_norm = math.sqrt(sum(v ** 2 for v in delta))
        if request_norm <= 0.0 or request_norm > CARTESIAN_MAX_STEP_MM:
            raise UnsafeStateError("笛卡尔请求必须是非零且不超过 6mm 的微步")
        start = self.read_state()
        if start.tool_pose is None:
            raise UnsafeStateError("缺少完整起点位姿，拒绝笛卡尔运动")
        if start.tool_pose[2] < CARTESIAN_MIN_TOOL_Z_MM:
            raise UnsafeStateError(
                "工具 z=%.1fmm 低于 %.0fmm：低位 IK 对舵机死区误差过敏，"
                "禁止笛卡尔微步；另行人审关节空间恢复 TaskSpec" % (
                    start.tool_pose[2], CARTESIAN_MIN_TOOL_Z_MM))
        state = self._exec(["tool_step", "%.4f" % dx, "%.4f" % dy, "%.4f" % dz])
        if state.motion_current_ok is False:
            raise UnsafeStateError("笛卡尔运动电流门未过：峰值 %s" % state.max_currents)
        if state.tool_pose is None:
            raise UnsafeStateError("运动后缺少完整位姿，拒绝判定成功")
        validate_cartesian_step(start.tool_pose, state.tool_pose, delta)
        return state

    def move_contact_delta(self, dx: float, dy: float, dz: float,
                           min_clearance_mm: float = 35.0) -> ArmState:
        """接触区 6mm 内微步；独立 clearance，不开放旧低位自动恢复。"""
        if not CONTACT_APPROACH_AVAILABLE:
            raise UnsafeStateError("接触微步在爪口几何标定前默认禁用，禁止重复盲试")
        delta = [float(dx), float(dy), float(dz)]
        request_norm = math.sqrt(sum(v ** 2 for v in delta))
        if request_norm <= 0.0 or request_norm > CARTESIAN_MAX_STEP_MM:
            raise UnsafeStateError("接触微步必须非零且不超过 6mm")
        if not CONTACT_MIN_TOOL_Z_MM + 5.0 <= min_clearance_mm <= CARTESIAN_MIN_TOOL_Z_MM:
            raise UnsafeStateError("接触微步 clearance 必须在 [35,70]mm")
        start = self.read_state()
        if start.tool_pose is None:
            raise UnsafeStateError("接触微步缺少完整起点")
        if start.tool_pose[2] + delta[2] < min_clearance_mm:
            raise UnsafeStateError("接触微步目标低于任务 clearance")
        state = self._exec(["tool_step", "%.4f" % dx, "%.4f" % dy, "%.4f" % dz,
                            "%.1f" % min_clearance_mm])
        if state.motion_current_ok is False:
            raise UnsafeStateError("接触微步没有确认到实际运动：%s" % state.max_currents)
        if state.tool_pose is None:
            raise UnsafeStateError("接触微步后缺少完整位姿")
        validate_cartesian_step(start.tool_pose, state.tool_pose, delta)
        return state

    def move_free_space(self, target_joints: list, interval_ms: int = 3000,
                        start_state: ArmState | None = None,
                        min_clearance_mm: float = FREESPACE_MIN_TOOL_Z_MM) -> ArmState:
        """高位自动转场：Nano 预检全路径并监护 z，主机复核电流和关节跟踪。"""
        if len(target_joints) != 5:
            raise UnsafeStateError("自由空间转场需要五个关节目标")
        if not OBSERVATION_MIN_TOOL_Z_MM <= min_clearance_mm <= FREESPACE_MIN_TOOL_Z_MM:
            raise UnsafeStateError("观察/自由空间 clearance 必须在 [90,120]mm")
        for idx, name in enumerate(JOINT_ORDER[:5]):
            lo, hi = JOINT_LIMITS[name]
            if not lo <= float(target_joints[idx]) <= hi:
                raise UnsafeStateError("自由空间目标 %s 超出限位" % name)
        start = start_state or self.read_state()
        if start.tool_pose is None or start.tool_pose[2] < min_clearance_mm:
            raise UnsafeStateError("转场起点低于任务 clearance：%s" % start.tool_pose)
        args = ["free_space_move", *["%.5f" % float(v) for v in target_joints],
                str(int(interval_ms)), "%.1f" % min_clearance_mm]
        state = self._exec(args)
        if state.motion_current_ok is False:
            raise UnsafeStateError("自由空间运动反馈门未过（低载电流可接受，需角度/FK跟踪）：峰值 %s"
                                   % state.max_currents)
        if state.tool_pose is None or state.tool_pose[2] < min_clearance_mm:
            raise UnsafeStateError("运动后高度门未过（clearance=%.1fmm）：%s" % (
                min_clearance_mm, state.tool_pose))
        errors = [abs(state.joints[i] - float(target_joints[i])) for i in range(5)]
        if max(errors) >= 0.12:
            raise UnsafeStateError("自由空间关节跟踪失败：%s" % errors)
        return state

    def move_contact_approach(self, target_joints: list, min_clearance_mm: float = 35.0,
                              interval_ms: int = 2200) -> ArmState:
        """接触接近专用入口：低速、可取消、逐帧复核；不与自由空间规则混用。"""
        if not CONTACT_APPROACH_AVAILABLE:
            raise UnsafeStateError("接触接近在爪口几何标定前默认禁用，禁止重复盲试")
        if len(target_joints) != 5:
            raise UnsafeStateError("接触接近需要五个关节目标")
        if not CONTACT_MIN_TOOL_Z_MM <= min_clearance_mm <= OBSERVATION_MIN_TOOL_Z_MM:
            raise UnsafeStateError("接触接近 clearance 必须在 [30,90]mm")
        for idx, name in enumerate(JOINT_ORDER[:5]):
            lo, hi = JOINT_LIMITS[name]
            if not lo <= float(target_joints[idx]) <= hi:
                raise UnsafeStateError("接触接近目标 %s 超出限位" % name)
        start = self.read_state()
        if start.tool_pose is None or start.tool_pose[2] < min_clearance_mm:
            raise UnsafeStateError("接触接近起点低于任务 clearance")
        args = ["free_space_move", *["%.5f" % float(v) for v in target_joints],
                str(int(interval_ms)), "%.1f" % min_clearance_mm]
        state = self._exec(args)
        if state.motion_current_ok is False:
            raise UnsafeStateError("接触接近运动反馈门未过：峰值 %s" % state.max_currents)
        if state.tool_pose is None or state.tool_pose[2] < min_clearance_mm:
            raise UnsafeStateError("接触接近后高度门未过：%s" % state.tool_pose)
        errors = [abs(state.joints[i] - float(target_joints[i])) for i in range(5)]
        if max(errors) >= 0.12:
            raise UnsafeStateError("接触接近关节跟踪失败：%s" % errors)
        return state

    def recover_low_pose(self, joint2_delta: float, joint3_delta: float,
                         joint4_delta: float) -> ArmState:
        """近桌协调关节恢复：真机验收否决后默认禁用，只保留离线看门狗覆盖。"""
        if not LOW_POSE_RECOVERY_AVAILABLE:
            raise UnsafeStateError(
                "低位协调恢复已被 2026-11-15 真机验收否决并默认禁用；"
                "请断电重启并人工安置到安全姿态")
        deltas = [float(joint2_delta), float(joint3_delta), float(joint4_delta)]
        if max(abs(v) for v in deltas) > 0.12 or not any(deltas):
            raise UnsafeStateError("恢复关节增量必须在 ±0.12rad 内且非零")
        start = self.read_state()
        if start.tool_pose is None or len(start.joints) != 6:
            raise UnsafeStateError("恢复前缺少完整关节与位姿")
        if not 25.0 <= start.tool_pose[2] < CARTESIAN_MIN_TOOL_Z_MM:
            raise UnsafeStateError("恢复起点 z 必须在 [25,70)mm：%s" % start.tool_pose[2])
        state = self._exec(["recovery_step", *["%.4f" % v for v in deltas]])
        if state.motion_current_ok is False:
            raise UnsafeStateError("恢复运动电流门未过：峰值 %s" % state.max_currents)
        if state.tool_pose is None:
            raise UnsafeStateError("恢复后缺少完整位姿")
        target = list(start.joints[:5])
        for offset, delta in enumerate(deltas, 1):
            target[offset] += delta
        errors = [abs(state.joints[i] - target[i]) for i in range(1, 4)]
        if max(errors) >= RECOVERY_ANGLE_TRACK_TOL:
            raise UnsafeStateError("恢复关节跟踪未达 0.065rad 门限：%s" % errors)
        actual = [state.tool_pose[i] - start.tool_pose[i] for i in range(3)]
        distance = math.sqrt(sum(v ** 2 for v in actual))
        horizontal = math.sqrt(sum(v ** 2 for v in actual[:2]))
        if actual[2] < 1.0:
            raise UnsafeStateError("恢复后未有效抬升：delta=%s" % actual)
        if distance > CARTESIAN_MAX_STEP_MM:
            raise UnsafeStateError("恢复实际位移超过 6mm：delta=%s" % actual)
        if horizontal > CARTESIAN_Z_LOCK_TOL_MM:
            raise UnsafeStateError("恢复水平漂移超过 3mm：delta=%s" % actual)
        return state

    def check_currents(self, state: ArmState) -> None:
        loaded = [state.currents.get(s, 0.0) for s in (2, 3)]  # 肘/腕承载关节
        if any(c > CURRENT_SPIKE_A for c in loaded):
            raise UnsafeStateError("电流尖峰 %s —— 立即回退" % state.currents)
        # 注：带载关节 0A 在两种状态出现（驱动保护/完全卸载），由角度跟踪门兜底

    def set_gripper(self, rad: float) -> ArmState:
        return self._exec(["grip", "%.4f" % rad])

    def home(self) -> ArmState:
        return self._exec(["home"])

    def prepare(self) -> ArmState:
        """自动开工准备：高位越界关节由 harness 合法化，无需操作者理解关节角。"""
        state = self.read_state()
        if len(state.joints) != 6 or state.tool_pose is None:
            raise UnsafeStateError("开工准备失败：关节或位姿读数不全")
        target = [min(max(state.joints[i], JOINT_LIMITS[name][0]), JOINT_LIMITS[name][1])
                  for i, name in enumerate(JOINT_ORDER[:5])]
        outside = [abs(target[i] - state.joints[i]) > JOINT_LIMIT_READ_TOL_RAD
                   for i in range(5)]
        if any(outside):
            if state.tool_pose[2] < FREESPACE_MIN_TOOL_Z_MM:
                raise UnsafeStateError("关节越界且工具低于 120mm，拒绝自动合法化")
            state = self.move_free_space(target, interval_ms=3000, start_state=state)
        return self.startup_check(state)

    # -- 启动自检（docs/09 安全层：z<30mm 判趴伏拒任务）--
    def startup_check(self, state: ArmState | None = None) -> ArmState:
        state = state or self.read_state()
        if len(state.joints) != 6:
            raise UnsafeStateError("启动自检失败：关节读数不全 %s" % state.joints)
        for idx, name in enumerate(JOINT_ORDER[:5]):
            lo, hi = JOINT_LIMITS[name]
            value = state.joints[idx]
            if not lo - JOINT_LIMIT_READ_TOL_RAD <= value <= hi + JOINT_LIMIT_READ_TOL_RAD:
                raise UnsafeStateError(
                    "启动自检失败：%s=%.3frad 超出限位 [%.3f, %.3f]，"
                    "禁止靠运动命令自动越界回正；请断电人工恢复" % (name, value, lo, hi))
        if state.tool_z_mm is not None and state.tool_z_mm < STARTUP_MIN_TOOL_Z_MM:
            raise UnsafeStateError(
                "启动自检失败：工具 z=%.1fmm 低于 %.0fmm —— 趴伏构型，"
                "先执行关节反序解卡（docs/08 §10.1），禁用 home()"
                % (state.tool_z_mm, STARTUP_MIN_TOOL_Z_MM))
        self._startup_ok = True
        return state

    # -- Executor 接口（接五层骨架的通用原语）--
    def execute(self, primitive: Primitive) -> dict:
        if not self._startup_ok:
            self.prepare()
        from ..primitives.library import MoveTo, SetGripper  # 局部导入避环
        if isinstance(primitive, MoveTo):
            state = self.home() if primitive.pose_name.upper() == "HOME" else self.read_state()
        elif isinstance(primitive, SetGripper):
            state = self.set_gripper(1.0 if primitive.open else 0.0)
        else:
            raise NotImplementedError("CreateBlock 原语走 primitives.createblock 编排函数")
        return {"joints": state.joints, "currents": state.currents,
                "held": state.held}

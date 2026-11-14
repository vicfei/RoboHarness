"""Closed-loop task executor: observe -> reason -> act -> verify, every step.

2026-11-15 定稿纪律：动作前后必须过三问——我在哪(FK)/它在哪(感知)/动对没(验证)。
任何一步前提不满足即停，绝不盲走。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .executor.createblock import ArmState, CreateBlockExecutor


@dataclass
class WorldState:
    """世界状态三元组：爪在哪 / 目标在哪 / 差多少。"""
    ts: float
    joints: list
    gripper_xy_mm: tuple | None = None   # FK 推算的爪口位置
    tool_z_mm: float | None = None
    target_px: tuple | None = None       # 目标在画面中的像素
    target_visible: bool = False
    note: str = ""


class ClosedLoopTask:
    """一次任务的闭环执行器。每个 act() 前后自动 observe+verify。"""

    def __init__(self, executor: CreateBlockExecutor, frame_src, locator,
                 name: str = "task"):
        self.ex = executor
        self.fs = frame_src
        self.locator = locator          # callable(frame)->(cx,cy)|None
        self.name = name
        self.history: list = []
        self.evidence: list = []

    # ---- 观察 ----
    def observe(self, note="") -> WorldState:
        st: ArmState = self.ex.read_state()
        frame = self.fs.read_warmed(1)
        tgt = self.locator(frame) if frame is not None else None
        ws = WorldState(ts=time.time(), joints=st.joints,
                        tool_z_mm=st.tool_z_mm,
                        target_px=tgt, target_visible=tgt is not None,
                        note=note)
        self.history.append(ws)
        self.evidence.append({"kind": "observe", "state": str(ws)})
        return ws

    # ---- 推理：目标是否在爪口作用区(画面下半中央盒) ----
    @staticmethod
    def target_under_mouth(ws: WorldState, box=(260, 280, 420, 400)) -> bool:
        if not ws.target_visible:
            return False
        x, y = ws.target_px
        return box[0] <= x <= box[2] and box[1] <= y <= box[3]

    # ---- 前提校验后动作 ----
    def act(self, joint: str, rad: float, interval_ms: int = 1000,
            require_target: bool = False) -> WorldState:
        pre = self.observe(note="pre-%s->%.2f" % (joint, rad))
        if require_target and not self.target_under_mouth(pre):
            self.evidence.append({"kind": "ABORT",
                                  "why": "target not under mouth", "state": str(pre)})
            raise RuntimeError("前提不满足: 目标不在爪口区 %s" % (pre,))
        st = self.ex.set_joint(joint, rad, interval_ms=interval_ms)
        self.ex.check_currents(st)
        post = self.observe(note="post-%s" % joint)
        self.evidence.append({"kind": "act", "joint": joint, "to": rad,
                              "achieved": st.joints, "post_target": post.target_px})
        return post

    def gripper(self, rad: float) -> ArmState:
        pre = self.observe(note="pre-grip%.2f" % rad)
        st = self.ex.set_gripper(rad)
        self.evidence.append({"kind": "gripper", "to": rad,
                              "held": st.held, "grip": st.grip_angle})
        return st

    def report(self) -> str:
        ws = self.history[-1] if self.history else None
        return "%s | steps=%d | last: %s" % (self.name, len(self.history), ws)

"""Executor 接口：mock 与真机 TCP 桥同接口，其余层无感切换。"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..types import Primitive


class UnsafeStateError(RuntimeError):
    """跨后端安全熔断：上层必须停任务并携带证据叫人/换策略。"""


class Executor(ABC):
    @abstractmethod
    def execute(self, primitive: Primitive) -> dict:
        """执行单条原语 → 回执（进证据流）。"""

    def calibrate(self) -> None:  # 真机期：位姿库标定/复核入口
        pass


class TCPBridgeExecutor(Executor):
    """真机实现（05 号资产）：TCP 桥 → 度数级舵机控制。骨架期留桩。"""

    def execute(self, primitive: Primitive) -> dict:
        raise NotImplementedError("臂重装后接 SO-101 TCP 桥（照 SO101_QUICKSTART）")

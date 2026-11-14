"""执行纪律层：TaskSpec / 目标验证器 / 宏黑名单。

2026-11-15 事故整改（推飞白插头+横扫打落黑插头后定稿）：
1. 禁用一切不透明宏（厂商 grab_cubic 等）——动作只允许受控微步；
2. 任务必须先声明 TaskSpec（每步预期+校验+失败处置），人审后才执行；
3. Verify 只认目标证据（前后帧区域比对），单一启发式禁止宣告成功。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

# 黑名单：这些"一站式宏"内部动作不受观察/电流监护覆盖，曾造成推飞/打落事故
MACRO_BAN = {"grab_cubic", "home_from_collapsed", "set_damping_store"}


@dataclass
class StepSpec:
    name: str
    action: str                      # 动作描述
    expect: str                      # 预期结果（可观察表述）
    check: str                       # 校验方法
    on_fail: str = "abort"           # abort / retry1 / replan


@dataclass
class TaskSpec:
    goal: str                        # 最终目标（可验证表述）
    steps: list = field(default_factory=list)
    confidence_rule: str = "成功宣告需目标证据；证据不足='未验证'"

    def table(self) -> str:
        rows = ["# %s" % self.goal, "| 步 | 动作 | 预期 | 校验 | 失败处置 |",
                "|---|---|---|---|---|"]
        for i, s in enumerate(self.steps, 1):
            rows.append("| %d | %s | %s | %s | %s |" % (i, s.action, s.expect, s.check, s.on_fail))
        return "\n".join(rows)


class GoalVerifier:
    """目标状态验证：前后帧区域比对（区域=目标ROI的像素矩形）。"""

    @staticmethod
    def blob_in_roi(img, roi, hsv_lo, hsv_hi, min_area=400) -> bool:
        x, y, w, h = roi
        crop = img[y:y + h, x:x + w]
        m = cv2.inRange(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV), hsv_lo, hsv_hi)
        return int(m.sum() / 255) >= min_area

    @classmethod
    def moved(cls, before, after, from_roi, to_roi, hsv_lo, hsv_hi) -> dict:
        """目标从 from_roi 消失且在 to_roi 出现 => 目标达成（带证据）。"""
        was = cls.blob_in_roi(before, from_roi, hsv_lo, hsv_hi)
        gone = not cls.blob_in_roi(after, from_roi, hsv_lo, hsv_hi)
        now = cls.blob_in_roi(after, to_roi, hsv_lo, hsv_hi)
        return {"achieved": bool(gone and now), "was_there": was,
                "gone_from_origin": gone, "appears_in_target": now,
                "confidence": "HIGH" if (gone and now) else
                              ("LOW" if now or gone else "NONE")}

"""Cloud VLM planner with provider fallback chain (M2).

设计（docs/09）：LLM 低频出原语序列（文本计划），确定性代码执行。
三家降级链：deepseek -> glm -> kimi -> ScriptedFallback。
密钥一律环境变量注入，绝不落仓。
"""

from __future__ import annotations

import json
import os
import urllib.request

PRIMITIVES_SCHEMA = ["pick", "place", "scan", "home", "push"]


class PlanStep(dict):
    """单步原语: {"op": "pick", "target": "white_plug"} 等。"""


def validate_steps(steps) -> list:
    """计划 schema 校验：必须是列表且每步 op 合法。"""
    if not isinstance(steps, list):
        raise ValueError("plan not a list")
    for s in steps:
        if not isinstance(s, dict) or s.get("op") not in PRIMITIVES_SCHEMA:
            raise ValueError("bad step %r" % (s,))
    return steps


class CloudPlanner:
    """VLM 规划器：任务文本(+场景摘要) -> PlanStep 列表。"""

    PROMPT = (
        "You are the planner of a robot arm harness. Available primitives: "
        "pick(target), place(container), scan(target), home(), push(target, to). "
        "Targets/containers are short names from the scene summary. "
        "Reply with ONLY a JSON array of steps, e.g. "
        '[{"op":"pick","target":"white_plug"},{"op":"place","container":"plate"}] '
        "No prose."
    )

    def __init__(self, providers=None, timeout=45):
        self.providers = providers or self._default_providers()
        self.timeout = timeout

    @staticmethod
    def _default_providers():
        defs = []
        if os.environ.get("DEEPSEEK_API_KEY"):
            defs.append(("deepseek",
                         "https://api.deepseek.com/chat/completions",
                         os.environ["DEEPSEEK_API_KEY"], "deepseek-chat"))
        if os.environ.get("GLM_API_KEY"):
            defs.append(("glm",
                         "https://open.bigmodel.cn/api/paas/v4/chat/completions",
                         os.environ["GLM_API_KEY"], "glm-4v-flash"))
        if os.environ.get("KIMI_API_KEY"):
            defs.append(("kimi",
                         "https://api.moonshot.cn/v1/chat/completions",
                         os.environ["KIMI_API_KEY"], "moonshot-v1-8k-vision"))
        return defs

    def plan(self, task: str, scene_summary: str = "") -> list:
        """返回 PlanStep 列表；全部 provider 失败抛 RuntimeError。"""
        user_msg = "Task: %s\nScene: %s" % (task, scene_summary or "(empty)")
        errors = []
        for name, url, key, model in self.providers:
            try:
                steps = self._call(url, key, model, user_msg)
                print("[planner] plan by %s: %s" % (name, steps))
                return steps
            except Exception as e:  # noqa: BLE001 — 降级链核心
                errors.append("%s: %s" % (name, e))
                print("[planner] %s failed: %s" % (name, e))
        raise RuntimeError("all providers failed: %s" % errors)

    def _call(self, url, key, model, user_msg):
        body = json.dumps({
            "model": model,
            "messages": [
                {"role": "system", "content": self.PROMPT},
                {"role": "user", "content": user_msg},
            ],
            "max_tokens": 400, "temperature": 0.0,
        }).encode()
        req = urllib.request.Request(url, data=body, headers={
            "Authorization": "Bearer " + key,
            "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            data = json.load(r)
        text = data["choices"][0]["message"]["content"].strip()
        # 兼容 ```json 包裹
        if text.startswith("```"):
            text = text.split("```")[1].lstrip("json").strip()
        return validate_steps(json.loads(text))

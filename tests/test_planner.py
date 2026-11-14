"""M2 planner 单测：schema 校验与降级链（离线，不触网）。"""
import unittest
from unittest import mock

from roboharness.planner import CloudPlanner


class TestPlanner(unittest.TestCase):
    def test_no_keys_no_providers(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            p = CloudPlanner()
            self.assertEqual(p.providers, [])

    def test_schema_rejects_unknown_op(self):
        p = CloudPlanner(providers=[])
        with mock.patch.object(p, "_call", return_value=[{"op": "fly"}]):
            with mock.patch.object(p, "_call", return_value=[{"op": "fly"}]):
                with self.assertRaises(ValueError):
                    p.plan("x")

    def test_fallback_chain(self):
        calls = []

        def boom(url, key, model, msg):
            calls.append(model)
            if len(calls) < 2:
                raise TimeoutError("simulated")
            return [{"op": "home"}]

        p = CloudPlanner(providers=[("a", "u1", "k1", "m1"), ("b", "u2", "k2", "m2")])
        with mock.patch.object(p, "_call", side_effect=boom):
            steps = p.plan("go home")
        self.assertEqual(steps, [{"op": "home"}])
        self.assertEqual(calls, ["m1", "m2"])


if __name__ == "__main__":
    unittest.main()

"""三角定位单测：两个已知位姿的射线交点应还原目标真值。"""
import unittest
import numpy as np
from roboharness.locate import triangulate, tool_to_cam, pixel_ray

class TestTriangulate(unittest.TestCase):
    def _T(self, x, y, z):
        from roboharness.locate import rpy_to_R
        T = np.eye(4); T[:3, :3] = rpy_to_R(np.array([0, np.pi/2, 0])); T[:3, 3] = [x, y, z]
        return T

    def test_two_views_recover_point(self):
        # 相机在 (0,0,200) 与 (100,0,200) 都看向 (50,0,50) -> 反算应还原
        target = np.array([50.0, 0.0, 50.0])
        sights = []
        for cx in (0.0, 100.0):
            Tc = self._T(cx, 0, 200)
            # 由真值反推像素: 光心->目标向量转回相机系
            v = target - Tc[:3, 3]
            # 相机系 rpy(0,pi/2,0): 直接构造逆旋转
            from roboharness.locate import rpy_to_R, FX, FY, CX, CY
            R = rpy_to_R(np.array([0, np.pi/2, 0]))
            d = R.T @ v
            px = (d[0] / d[2] * FX + CX, d[1] / d[2] * FY + CY)
            sights.append((px, Tc))
        p, resid = triangulate(sights)
        self.assertTrue(np.allclose(p, target, atol=1.0), p)
        self.assertLess(resid, 1.0)

if __name__ == "__main__":
    unittest.main()

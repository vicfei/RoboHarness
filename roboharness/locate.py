"""目标 3D 三角定位：多姿态观测 -> 世界坐标(mm)。

原理：每次观测 = 相机位姿(FK+URDF 相机安装变换) + 一条像素射线；
两条以上射线最小二乘求交 -> 目标在基座系的 3D 位置。
之后夹取不再依赖"像素锚点"，直接把爪口送往坐标点。
"""

from __future__ import annotations

import numpy as np

# URDF: camera 固定于 link4, 安装变换 (位置 m, rpy)
CAM_POS = np.array([0.06715, -0.053, 0.0125])
CAM_RPY = np.array([0.0, np.pi / 2, 0.0])
FOV_DEG = 60.0
IMG_W, IMG_H = 640, 480
FX = IMG_W / (2 * np.tan(np.radians(FOV_DEG) / 2))  # ≈554
FY = FX
CX, CY = IMG_W / 2, IMG_H / 2


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy_, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def tool_to_cam(T_tool_4x4_mm: np.ndarray) -> np.ndarray:
    """工具系 4x4(mm) -> 相机系 4x4(mm)。"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(CAM_RPY)
    T[:3, 3] = CAM_POS * 1000.0
    return T_tool_4x4_mm @ T


def pixel_ray(pixel_xy, T_cam_4x4_mm):
    """像素 -> 基座系射线 (origin=相机光心, 单位方向)。"""
    x, y = pixel_xy
    d_cam = np.array([(x - CX) / FX, (y - CY) / FY, 1.0])
    d_cam /= np.linalg.norm(d_cam)
    R = T_cam_4x4_mm[:3, :3]
    o = T_cam_4x4_mm[:3, 3]
    return o, R @ d_cam


def triangulate(sightings) -> tuple:
    """sightings: [(pixel_xy, T_cam_4x4_mm)] -> (xyz_mm, 残差mm)。

    最小二乘: 求 p 使 sum_i |(p-o_i) x d_i|^2 最小 (点到射线距离)。
    """
    A, b = [], []
    for px, T in sightings:
        o, d = pixel_ray(px, T)
        M = np.eye(3) - np.outer(d, d)
        A.append(M)
        b.append(M @ o)
    A = np.vstack(A)
    b = np.concatenate(b)
    p, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = max(np.linalg.norm((np.eye(3) - np.outer(pixel_ray(s[0], s[1])[1],
                pixel_ray(s[0], s[1])[1])) @ (p - pixel_ray(s[0], s[1])[0]))
                for s in sightings)
    return p, float(resid)

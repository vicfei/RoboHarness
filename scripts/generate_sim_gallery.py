"""Generate honest PyBullet homepage images for RoboHarness.

Reads the local CreateBlock simulation twin (original vendor STL/URDF) and writes:
1. a deterministic six-step rehearsal sequence, and
2. a nine-scene domain-randomization preview.

These are environment previews, **not** results from a trained policy. The
generator intentionally labels them that way so the README cannot overclaim M1.7.

Usage:
    CREATEBLOCK_SIM_DIR=~/services/robotics/createblock-arm/sim \
      python scripts/generate_sim_gallery.py
"""

from __future__ import annotations

import argparse
import math
import tempfile
from pathlib import Path
import sys

import cv2
import numpy as np
import pybullet


def parse_args() -> argparse.Namespace:
    default_sim = Path.home() / "services/robotics/createblock-arm/sim"
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-dir", type=Path, default=default_sim)
    parser.add_argument(
        "--out-dir", type=Path,
        default=Path(__file__).resolve().parents[1] / "docs/assets/sim")
    return parser.parse_args()


def load_sim(sim_dir: Path):
    sys.path.insert(0, str(sim_dir))
    from arm5dof_pybullet import Arm5DoFPyBullet
    return Arm5DoFPyBullet(gui=False)


def checker_texture(path: Path) -> int:
    """A warm checkerboard makes the physics plane read as a workspace."""
    size, cell = 1024, 32
    image = np.zeros((size, size, 3), dtype=np.uint8)
    light = np.array([176, 168, 156], dtype=np.uint8)
    dark = np.array([140, 132, 122], dtype=np.uint8)
    for y in range(0, size, cell):
        for x in range(0, size, cell):
            color = light if ((x // cell + y // cell) % 2 == 0) else dark
            image[y:y + cell, x:x + cell] = color
    # Subtle border makes repeated texture tiles visible without visual noise.
    cv2.rectangle(image, (0, 0), (size - 1, size - 1), (112, 106, 98), 2)
    cv2.imwrite(str(path), image)
    return pybullet.loadTexture(str(path))


def camera_matrix(yaw: float = 52.0, pitch: float = -30.0,
                  distance: float = 0.66, fov: float = 46.0):
    # pitch <= -30 with fov 46 keeps the top-of-frame ray pointing down at
    # the floor, so no background/horizon band can appear inside a panel.
    view = pybullet.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.15, 0.0, 0.12], distance=distance,
        yaw=yaw, pitch=pitch, roll=0, upAxisIndex=2)
    proj = pybullet.computeProjectionMatrixFOV(
        fov=fov, aspect=4 / 3, nearVal=0.02, farVal=2.0)
    return view, proj


# Rows 0-107 of the 640x480 render are empty white sky; crop them so panels
# are filled by the workspace instead of a blank strip.
SKY_ROWS = 108


def render(view, proj, width: int = 640, height: int = 480) -> np.ndarray:
    _, _, rgba, _, _ = pybullet.getCameraImage(
        width, height, view, proj, renderer=pybullet.ER_TINY_RENDERER)
    rgb = np.reshape(rgba, (height, width, 4))[:, :, :3]
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return image[SKY_ROWS:, :]


def capture(**camera_kwargs) -> np.ndarray:
    return render(*camera_matrix(**camera_kwargs))


def project_point(point, view, proj, width: int, height: int):
    homogeneous = np.array([*point, 1.0], dtype=np.float64)
    # PyBullet hands out column-major OpenGL matrices; .T restores row-major.
    clip = (np.asarray(proj, dtype=np.float64).reshape(4, 4).T
            @ np.asarray(view, dtype=np.float64).reshape(4, 4).T
            @ homogeneous)
    ndc = clip[:2] / clip[3]
    x = (ndc[0] * 0.5 + 0.5) * width
    y = (1.0 - (ndc[1] * 0.5 + 0.5)) * height
    return x, y


def draw_target_bracket(image: np.ndarray, body: int, view, proj,
                        color=(64, 235, 255)) -> None:
    """Corner brackets around the randomized target's projected AABB."""
    aabb_min, aabb_max = pybullet.getAABB(body)
    height, width = image.shape[:2]
    corners = [
        (x, y, z)
        for x in (aabb_min[0], aabb_max[0])
        for y in (aabb_min[1], aabb_max[1])
        for z in (aabb_min[2], aabb_max[2])]
    points = [project_point(c, view, proj, width, height) for c in corners]
    x0 = int(min(p[0] for p in points))
    x1 = int(max(p[0] for p in points))
    y0 = int(min(p[1] for p in points))
    y1 = int(max(p[1] for p in points))
    arm_len = max(8, int(min(x1 - x0, y1 - y0) * 0.28))
    for corner_x, corner_y, dx, dy in (
            (x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(image, (corner_x, corner_y),
                 (corner_x + dx * arm_len, corner_y), color, 2, cv2.LINE_AA)
        cv2.line(image, (corner_x, corner_y),
                 (corner_x, corner_y + dy * arm_len), color, 2, cv2.LINE_AA)


def add_box(position, half_extents, color, mass: float = 0.0) -> int:
    shape = pybullet.createCollisionShape(
        pybullet.GEOM_BOX, halfExtents=half_extents)
    visual = pybullet.createVisualShape(
        pybullet.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    return pybullet.createMultiBody(
        baseMass=mass, baseCollisionShapeIndex=shape,
        baseVisualShapeIndex=visual, basePosition=position)


def make_plate() -> int:
    # Pink placement target from the real plug-to-plate task.
    plate = add_box([0.225, -0.105, 0.007], [0.045, 0.045, 0.007],
                    [0.94, 0.42, 0.62, 1.0])
    pybullet.changeVisualShape(plate, -1, rgbaColor=[0.94, 0.42, 0.62, 1.0])
    return plate


def panel(image: np.ndarray, title: str, subtitle: str,
          width: int = 512, height: int = 298, bar: int = 47) -> np.ndarray:
    scaled = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((height + bar, width, 3), dtype=np.uint8)
    canvas[:bar] = (18, 24, 39)
    canvas[bar:] = scaled
    cv2.rectangle(canvas, (0, 0), (width - 1, height + bar - 1),
                  (36, 194, 236), 2)
    cv2.putText(canvas, title, (14, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, subtitle, (14, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (165, 195, 222), 1, cv2.LINE_AA)
    return canvas


def header(canvas: np.ndarray, title: str, subtitle: str) -> np.ndarray:
    top = np.zeros((86, canvas.shape[1], 3), dtype=np.uint8)
    top[:] = (18, 24, 39)
    cv2.putText(top, title, (28, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.90,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(top, subtitle, (28, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (145, 178, 215), 1, cv2.LINE_AA)
    cv2.rectangle(top, (18, 18), (canvas.shape[1] - 18, 70),
                  (36, 194, 236), 1)
    return np.vstack([top, canvas])


def footer(canvas: np.ndarray, text: str) -> np.ndarray:
    bottom = np.zeros((50, canvas.shape[1], 3), dtype=np.uint8)
    bottom[:] = (18, 24, 39)
    cv2.putText(bottom, text, (28, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                (226, 232, 240), 1, cv2.LINE_AA)
    return np.vstack([canvas, bottom])


def montage(panels: list[np.ndarray], columns: int,
            margin: int = 18, background=(10, 14, 23)) -> np.ndarray:
    rows = (len(panels) + columns - 1) // columns
    panel_h, panel_w = panels[0].shape[:2]
    width = columns * panel_w + (columns + 1) * margin
    height = rows * panel_h + (rows + 1) * margin
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:] = background
    for index, item in enumerate(panels):
        row, col = divmod(index, columns)
        x = margin + col * (panel_w + margin)
        y = margin + row * (panel_h + margin)
        canvas[y:y + item.shape[0], x:x + item.shape[1]] = item
    return canvas


def pose_text(arm) -> str:
    x, y, z, _, _ = arm.get_tool_pose()
    return f"tool {x:.0f},{y:.0f},{z:.0f} mm"


def make_sequence(arm, cube: int) -> np.ndarray:
    steps = []

    def snap(title: str, detail: str) -> None:
        steps.append(panel(capture(), title, detail))

    arm.home(); arm.wait()
    snap("1 | Home", pose_text(arm))

    arm.set_tool_pose([160, 0, 112], T=1.1); arm.wait()
    snap("2 | Approach", pose_text(arm))

    arm.gripper_open(angle=np.pi / 3, T=0.6); arm.wait()
    snap("3 | Gripper open", f"{pose_text(arm)} | jaw 60 deg")

    arm.set_tool_pose([160, 0, 68], T=0.8); arm.wait()
    snap("4 | Descend", pose_text(arm))

    arm.gripper_close(T=0.6); arm.wait()
    snap("5 | Close / contact gate", f"{pose_text(arm)} | mimic constrained")

    arm.set_tool_pose([160, 0, 112], T=0.8); arm.wait()
    snap("6 | Retract", f"{pose_text(arm)} | rehearsal only")

    canvas = montage(steps, columns=3)
    canvas = header(
        canvas,
        "CreateBlock PyBullet twin - deterministic rehearsal",
        "Original vendor STL/URDF | six-step primitive sequence | HEADLESS WSL")
    return footer(
        canvas,
        "Simulation preview - deterministic primitives | not a trained RL result, not a real-grasp success claim")


def make_randomization(arm) -> np.ndarray:
    arm.home(); arm.wait()
    rng = np.random.default_rng(20261115)
    palette = [
        (0.96, 0.94, 0.90), (0.94, 0.42, 0.62), (0.25, 0.68, 0.95),
        (0.98, 0.72, 0.22), (0.45, 0.82, 0.45), (0.78, 0.55, 0.96),
    ]
    # Deterministic color sweep so the preview grid shows the actual
    # randomization range instead of clustering on one palette draw.
    target_colors = [
        (0.97, 0.96, 0.90),  # off-white, the real plug color
        (0.94, 0.42, 0.62),  # pink, the real plate color
        (0.25, 0.68, 0.95),  # blue
        (0.98, 0.72, 0.22),  # orange
        (0.45, 0.82, 0.45),  # green
        (0.78, 0.55, 0.96),  # purple
        (0.93, 0.83, 0.30),  # yellow
        (0.30, 0.85, 0.82),  # teal
        (0.55, 0.58, 0.62),  # gray
    ]
    panels = []
    objects = []
    for seed in range(9):
        for body in objects:
            pybullet.removeBody(body)
        objects.clear()
        cube_color = target_colors[seed]
        cube_x = float(rng.uniform(0.12, 0.21))
        cube_y = float(rng.uniform(-0.075, 0.075))
        cube_size = float(rng.uniform(0.018, 0.027))
        cube = add_box([cube_x, cube_y, cube_size], [cube_size] * 3,
                       (*cube_color, 1.0), mass=0.05)
        pybullet.changeVisualShape(cube, -1, rgbaColor=[*cube_color, 1.0])
        objects.append(cube)
        # Distractors emphasize that this is a randomized environment, not a
        # hand-positioned demo scene; keep them clear of the target so the
        # bracket never lands on an ambiguous cluster.
        for _ in range(2):
            for _attempt in range(20):
                dx = float(rng.uniform(0.08, 0.25))
                dy = float(rng.uniform(-0.14, 0.14))
                if math.hypot(dx - cube_x, dy - cube_y) > 0.06:
                    break
            ds = float(rng.uniform(0.012, 0.021))
            color = palette[int(rng.integers(len(palette)))]
            body = add_box([dx, dy, ds], [ds] * 3, (*color, 1.0), mass=0.04)
            pybullet.changeVisualShape(body, -1, rgbaColor=[*color, 1.0])
            objects.append(body)
        for _ in range(12):
            pybullet.stepSimulation()
        yaw = float(rng.uniform(46, 58))
        pitch = float(rng.uniform(-36, -30))
        distance = float(rng.uniform(0.62, 0.72))
        view, proj = camera_matrix(yaw=yaw, pitch=pitch, distance=distance)
        frame = render(view, proj)
        draw_target_bracket(frame, cube, view, proj)
        label = f"seed {101 + seed}"
        detail = (f"target {cube_x * 1000:.0f},{cube_y * 1000:.0f} mm | "
                  f"size {cube_size * 1000:.0f} mm | cam yaw {yaw:.0f} deg")
        panels.append(panel(frame, label, detail, width=400, height=232, bar=42))
    canvas = montage(panels, columns=3, margin=14)
    canvas = header(
        canvas,
        "Domain-randomized scenes (object-level)",
        "Target pose, size, color, distractors and camera vary | yellow bracket = randomized target")
    return footer(
        canvas,
        "M1.7 environment preview - object-level randomization planned; no policy has been trained yet")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    arm = load_sim(args.sim_dir)

    with tempfile.TemporaryDirectory() as temp:
        texture = checker_texture(Path(temp) / "workspace_checker.png")
        # The vendor plane.urdf is small enough that its far edge shows as a
        # white horizon band inside every panel; replace it with a floor that
        # extends beyond the camera frame.
        pybullet.removeBody(arm.plane_id)
        floor = add_box([0, 0, -0.005], [5.0, 5.0, 0.005], (1, 1, 1, 1))
        pybullet.changeVisualShape(
            floor, -1, textureUniqueId=texture, rgbaColor=[1, 1, 1, 1])

    plate = make_plate()
    cube = add_box([0.160, 0.0, 0.024], [0.024] * 3,
                   (0.97, 0.96, 0.92, 1.0), mass=0.05)
    pybullet.changeVisualShape(cube, -1, rgbaColor=[0.97, 0.96, 0.92, 1.0])

    sequence = make_sequence(arm, cube)
    randomization = make_randomization(arm)
    cv2.imwrite(str(args.out_dir / "sim_sequence.jpg"), sequence,
                [cv2.IMWRITE_JPEG_QUALITY, 92])
    cv2.imwrite(str(args.out_dir / "sim_domain_randomization.jpg"), randomization,
                [cv2.IMWRITE_JPEG_QUALITY, 92])

    pybullet.removeBody(cube)
    pybullet.removeBody(plate)
    pybullet.removeBody(floor)
    arm.disconnect()
    print("WROTE", args.out_dir / "sim_sequence.jpg", sequence.shape)
    print("WROTE", args.out_dir / "sim_domain_randomization.jpg", randomization.shape)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Generate the RoboHarness homepage hero banner.

Composition: a high-resolution PyBullet twin render (left) + copy and two real
wrist-cam evidence frames from the first fully-automatic grasp (2026-11-14).

Honesty rules baked in:
- the real frames are the actual task camera (no staging, no stock photos);
- status pills match the README claims (real-hw validated / 64/64 / zero
  training); the grasp thumbnail caption says exactly what happened.

Usage:
    CREATEBLOCK_SIM_DIR=~/services/robotics/createblock-arm/sim \
      python scripts/generate_hero_banner.py
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pybullet
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_sim_gallery import (  # noqa: E402
    add_box, camera_matrix, checker_texture, load_sim, make_plate,
    project_point,
)

W, H = 1600, 560
BG = (10, 14, 23)
GRID = (14, 19, 32)
CYAN = (36, 194, 236)
WHITE = (240, 246, 252)
MUTED = (150, 165, 188)
BODY = (200, 210, 226)
GREEN = (96, 214, 140)
AMBER = (242, 186, 84)

FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")
F_KICK = ImageFont.truetype(str(FONT_DIR / "DejaVuSans-Bold.ttf"), 17)
F_HEAD = ImageFont.truetype(str(FONT_DIR / "DejaVuSans-Bold.ttf"), 37)
F_SUB = ImageFont.truetype(str(FONT_DIR / "DejaVuSans.ttf"), 18)
F_PILL = ImageFont.truetype(str(FONT_DIR / "DejaVuSans-Bold.ttf"), 14)
F_TAG = ImageFont.truetype(str(FONT_DIR / "DejaVuSans-Bold.ttf"), 15)
F_CAP = ImageFont.truetype(str(FONT_DIR / "DejaVuSans.ttf"), 14)
F_CHIP = ImageFont.truetype(str(FONT_DIR / "DejaVuSans.ttf"), 14)
F_HUD = ImageFont.truetype(str(FONT_DIR / "DejaVuSansMono.ttf"), 15)


def parse_args() -> argparse.Namespace:
    default_sim = Path.home() / "services/robotics/createblock-arm/sim"
    default_caps = Path("/mnt/c/Users/chenfei/Desktop/arm_captures")
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-dir", type=Path, default=default_sim)
    parser.add_argument("--captures-dir", type=Path, default=default_caps)
    parser.add_argument("--out", type=Path,
                        default=Path(__file__).resolve().parents[1]
                        / "docs/assets/hero/hero_banner.jpg")
    return parser.parse_args()


def render_twin(sim_dir: Path) -> np.ndarray:
    """2x-supersampled render of the arm descending onto the white plug."""
    arm = load_sim(sim_dir)
    with tempfile.TemporaryDirectory() as temp:
        texture = checker_texture(Path(temp) / "checker.png")
        pybullet.removeBody(arm.plane_id)
        floor = add_box([0, 0, -0.005], [5.0, 5.0, 0.005], (1, 1, 1, 1))
        pybullet.changeVisualShape(
            floor, -1, textureUniqueId=texture, rgbaColor=[1, 1, 1, 1])
    make_plate()
    plug = add_box([0.160, 0.0, 0.024], [0.024] * 3,
                   (0.97, 0.96, 0.92, 1.0), mass=0.05)
    pybullet.changeVisualShape(plug, -1, rgbaColor=[0.97, 0.96, 0.92, 1.0])

    arm.home(); arm.wait()
    arm.set_tool_pose([160, 0, 108], T=1.0); arm.wait()
    arm.gripper_open(angle=np.pi / 3, T=0.6); arm.wait()
    arm.set_tool_pose([160, 0, 68], T=0.8); arm.wait()

    w, h = 1760, 960
    view = pybullet.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.155, 0.0, 0.10], distance=0.58,
        yaw=52, pitch=-30, roll=0, upAxisIndex=2)
    proj = pybullet.computeProjectionMatrixFOV(
        fov=46, aspect=w / h, nearVal=0.02, farVal=2.0)
    _, _, rgba, _, _ = pybullet.getCameraImage(
        w, h, view, proj, renderer=pybullet.ER_TINY_RENDERER)
    frame = cv2.cvtColor(
        np.reshape(rgba, (h, w, 4))[:, :, :3], cv2.COLOR_RGB2BGR)

    # HUD: corner brackets on the plug target.
    aabb_min, aabb_max = pybullet.getAABB(plug)
    corners = [(x, y, z)
               for x in (aabb_min[0], aabb_max[0])
               for y in (aabb_min[1], aabb_max[1])
               for z in (aabb_min[2], aabb_max[2])]
    pts = [project_point(c, view, proj, w, h) for c in corners]
    x0, x1 = int(min(p[0] for p in pts)), int(max(p[0] for p in pts))
    y0, y1 = int(min(p[1] for p in pts)), int(max(p[1] for p in pts))
    arm_len = max(14, int(min(x1 - x0, y1 - y0) * 0.30))
    for cx, cy, dx, dy in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                           (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(frame, (cx, cy), (cx + dx * arm_len, cy), (36, 194, 236), 3)
        cv2.line(frame, (cx, cy), (cx, cy + dy * arm_len), (36, 194, 236), 3)
    # HUD status line
    cv2.putText(frame, "scan_for OK  center_on OK", (36, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (150, 224, 176), 3, cv2.LINE_AA)
    cv2.putText(frame, "grasp: contact-gated  z=68mm", (36, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (94, 196, 252), 3, cv2.LINE_AA)

    pybullet.removeBody(plug)
    pybullet.removeBody(floor)
    arm.disconnect()
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def paste_thumb(canvas: Image.Image, path: Path, x: int, y: int,
                w: int, h: int, caption: str, d: ImageDraw.ImageDraw) -> None:
    im = Image.open(path).convert("RGB")
    # center-crop to the target aspect, then resize
    tw, th = im.size
    target = w / h
    if tw / th > target:
        nw = int(th * target)
        im = im.crop(((tw - nw) // 2, 0, (tw + nw) // 2, th))
    else:
        nh = int(tw / target)
        im = im.crop((0, (th - nh) // 2, tw, (th + nh) // 2))
    im = im.resize((w, h), Image.LANCZOS)
    # brighten and gamma-lift — the originals are night shots
    arr = np.asarray(im).astype(np.float32) / 255.0
    arr = np.clip(arr * 1.45 + 0.05, 0, 1) ** 0.9
    im = Image.fromarray((arr * 255).astype(np.uint8))
    canvas.paste(im, (x, y))
    d.rectangle([x - 1, y - 1, x + w, y + h], outline=GREEN, width=2)
    # caption strip inside the bottom
    d.rectangle([x, y + h - 26, x + w, y + h], fill=(8, 12, 20))
    d.text((x + 10, y + h - 21), caption, font=F_CAP, fill=BODY)


def pill(d: ImageDraw.ImageDraw, x: int, y: int, text: str, color) -> int:
    w = int(d.textlength(text, font=F_PILL)) + 24
    d.rounded_rectangle([x, y, x + w, y + 30], radius=15,
                        fill=(16, 21, 35), outline=color, width=2)
    d.text((x + 12, y + 6), text, font=F_PILL, fill=color)
    return w


def main() -> int:
    args = parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)

    twin = render_twin(args.sim_dir)
    twin_img = Image.fromarray(twin).resize((880, 480), Image.LANCZOS)

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    # subtle background grid
    for gx in range(40, W, 80):
        d.line([(gx, 20), (gx, H - 20)], fill=GRID, width=1)
    for gy in range(20, H, 80):
        d.line([(20, gy), (W - 20, gy)], fill=GRID, width=1)

    # left: simulation panel
    px, py, pw, ph = 40, 40, 880, 480
    img.paste(twin_img, (px, py))
    d.rectangle([px - 2, py - 2, px + pw + 1, py + ph + 1],
                outline=CYAN, width=2)
    chip = "PyBullet twin | vendor STL/URDF | HEADLESS rehearsal"
    cw = int(d.textlength(chip, font=F_CHIP)) + 24
    d.rounded_rectangle([px + 14, py + ph - 44, px + 14 + cw, py + ph - 12],
                        radius=12, fill=(8, 12, 20), outline=(48, 66, 96))
    d.text((px + 26, py + ph - 37), chip, font=F_CHIP, fill=MUTED)

    # divider
    d.line([(940, 40), (940, 520)], fill=(24, 34, 54), width=1)

    # right: copy column
    cx = 972
    kicker = "INCIDENT-HARDENED ROBOT HARNESS"
    kx = cx
    for ch in kicker:
        d.text((kx, 52), ch, font=F_KICK, fill=CYAN)
        kx += d.textlength(ch, font=F_KICK) + 3.2
    d.text((cx, 92), "LLM plans. Primitives act.", font=F_HEAD, fill=WHITE)
    d.text((cx, 140), "Safety owns every motion.", font=F_HEAD, fill=CYAN)
    d.text((cx, 196),
           "$200 arm | Jetson Nano 4GB | zero-training core",
           font=F_SUB, fill=MUTED)

    px2 = cx
    px2 += pill(d, px2, 236, "real-hw validated", GREEN) + 12
    px2 += pill(d, px2, 236, "64/64 offline tests", (150, 160, 175)) + 12
    pill(d, px2, 236, "zero training", CYAN)

    d.text((cx, 296), "REAL HARDWARE - first automatic grasp, 2026-11-14",
           font=F_TAG, fill=GREEN)
    caps = args.captures_dir
    paste_thumb(img, caps / "grasp_closed.jpg", cx, 324, 288, 196,
                "close: plug in jaws", d)
    paste_thumb(img, caps / "grasp_lifted.jpg", cx + 300, 324, 288, 196,
                "lift: verified off-table", d)
    # sim -> real narrative link across the divider
    link_y = 422
    lx = 946
    while lx < cx - 8:
        d.line([(lx, link_y), (min(lx + 8, cx - 10), link_y)],
               fill=(96, 214, 140), width=2)
        lx += 16
    d.polygon([(cx - 4, link_y), (cx - 14, link_y - 5), (cx - 14, link_y + 5)],
              fill=(96, 214, 140))

    img.save(str(args.out), quality=92)
    print("WROTE", args.out, img.size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

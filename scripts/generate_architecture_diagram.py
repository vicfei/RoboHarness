"""Generate the RoboHarness architecture diagram for the README.

Pure-PIL drawing (no simulation dependency). Every box, arrow, and status tag
maps to real code in this repository; status colors are the honesty contract:

- green  = validated on real hardware
- amber  = implemented but locked pending calibration
- gray solid  = offline-tested
- gray dashed = planned, not built yet
- red dashed  = safety fuse path (halts the task)

Usage:
    python scripts/generate_architecture_diagram.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1680, 1110
BG = (10, 14, 23)
BOX = (16, 21, 35)
BOX2 = (22, 28, 45)
EDGE = (36, 194, 236)
ORANGE = (255, 158, 88)
WHITE = (240, 246, 252)
BODY = (198, 208, 224)
MUTED = (140, 155, 178)
GREEN = (96, 214, 140)
AMBER = (242, 186, 84)
GRAY = (150, 160, 175)
RED = (235, 110, 110)

FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(str(FONT_DIR / name), size)


F_TITLE = font(34, True)
F_SUB = font(19)
F_BOX = font(22, True)
F_BOXSUB = font(15)
F_ITEM = font(17, True)
F_BODY = font(16)
F_SMALL = font(14)
F_PILL = font(13, True)
F_LEGEND = font(16)


def rrect(d: ImageDraw.ImageDraw, xy, r=14, fill=BOX, outline=EDGE, width=2):
    d.rounded_rectangle(xy, radius=r, fill=fill, outline=outline, width=width)


def dashed_rect(d: ImageDraw.ImageDraw, xy, r=14, color=GRAY, width=2):
    x0, y0, x1, y1 = xy
    step, gap = 12, 7
    x = x0
    while x < x1:
        xe = min(x + step, x1)
        d.line([(x, y0), (xe, y0)], fill=color, width=width)
        d.line([(x, y1), (xe, y1)], fill=color, width=width)
        x += step + gap
    y = y0
    while y < y1:
        ye = min(y + step, y1)
        d.line([(x0, y), (x0, ye)], fill=color, width=width)
        d.line([(x1, y), (x1, ye)], fill=color, width=width)
        y += step + gap


def dashed_line(d: ImageDraw.ImageDraw, p1, p2, color=GRAY, width=3):
    x0, y0 = p1
    x1, y1 = p2
    length = max(abs(x1 - x0), abs(y1 - y0))
    if length == 0:
        return
    n = int(length // 16) + 1
    for i in range(0, n, 2):
        a = (x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n)
        b = (x0 + (x1 - x0) * min(i + 1, n) / n, y0 + (y1 - y0) * min(i + 1, n) / n)
        d.line([a, b], fill=color, width=width)


def arrow(d: ImageDraw.ImageDraw, p1, p2, color=EDGE, width=3, dashed=False):
    if dashed:
        dashed_line(d, p1, p2, color, width)
    else:
        d.line([p1, p2], fill=color, width=width)
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n
    px, py = -uy, ux
    head = 13
    d.polygon([
        p2,
        (p2[0] - head * ux + 6 * px, p2[1] - head * uy + 6 * py),
        (p2[0] - head * ux - 6 * px, p2[1] - head * uy - 6 * py),
    ], fill=color)


def elbow(d: ImageDraw.ImageDraw, points, color=EDGE, width=3, dashed=False):
    for i in range(len(points) - 2):
        if dashed:
            dashed_line(d, points[i], points[i + 1], color, width)
        else:
            d.line([points[i], points[i + 1]], fill=color, width=width)
    arrow(d, points[-2], points[-1], color, width, dashed)


def pill(d: ImageDraw.ImageDraw, x, y, text: str, color, dashed_=False):
    w = int(d.textlength(text, font=F_PILL)) + 22
    d.rounded_rectangle([x, y, x + w, y + 26], radius=13,
                        fill=BOX, outline=color, width=2 if not dashed_ else 1)
    if dashed_:
        dash_x = x + 4
        while dash_x < x + w - 4:
            d.line([(dash_x, y + 1), (min(dash_x + 8, x + w - 4), y + 1)],
                   fill=color, width=2)
            dash_x += 14
    d.text((x + 11, y + 5), text, font=F_PILL, fill=color)


def lines(d, x, y, items, gap=26):
    for text, f, c in items:
        d.text((x, y), text, font=f, fill=c)
        y += gap
    return y


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path,
                        default=Path(__file__).resolve().parents[1]
                        / "docs/assets/arch/architecture.jpg")
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    # ---------- header ----------
    d.text((28, 34), "RoboHarness architecture", font=F_TITLE, fill=WHITE)
    d.text((28, 82),
           "LLM plans, deterministic primitives act, the safety executor owns every motion  |  zero-training core, evidence-only success claims",
           font=F_SUB, fill=MUTED)

    # ---------- main row (y 138..538) ----------
    # A: cloud brain
    rrect(d, [28, 138, 316, 538])
    d.text((46, 156), "CLOUD BRAIN", font=F_BOX, fill=WHITE)
    d.text((46, 188), "LLM planner - off-the-shelf API", font=F_BOXSUB, fill=MUTED)
    lines(d, 46, 226, [
        ("DeepSeek / GLM / Kimi", F_BODY, BODY),
        ("1-3 calls per task", F_BODY, BODY),
        ("task -> primitive sequence", F_BODY, BODY),
        ("no fine-tuning, no GPU", F_BODY, BODY),
    ])
    pill(d, 46, 480, "planned M2", GRAY, dashed_=True)

    arrow(d, (316, 338), (352, 338))

    # B: discipline gate
    rrect(d, [352, 138, 640, 538])
    d.text((370, 156), "DISCIPLINE GATE", font=F_BOX, fill=WHITE)
    d.text((370, 188), "discipline.py - post-incident rules", font=F_BOXSUB, fill=MUTED)
    y = 226
    for head, sub in [
        ("TaskSpec", "per-step expect / check / on-fail"),
        ("GoalVerifier", "evidence or 'unverified'"),
        ("MACRO_BAN", "one-shot vendor macros banned"),
        ("human review", "plan approved before motion"),
    ]:
        d.text((370, y), head, font=F_ITEM, fill=WHITE)
        d.text((386, y + 24), sub, font=F_BODY, fill=BODY)
        y += 58
    pill(d, 370, 480, "offline-tested", GRAY)

    arrow(d, (640, 338), (676, 338))

    # C: local harness (the star)
    rrect(d, [676, 138, 1180, 538], fill=(14, 19, 32))
    d.text((696, 156), "LOCAL HARNESS", font=F_BOX, fill=WHITE)
    d.text((696, 188), "closed loop: observe -> reason -> act -> verify, every step",
           font=F_BOXSUB, fill=MUTED)
    chips = [
        (690, 214, "ClosedLoopTask", ["loop.py", "precondition abort", "evidence history"]),
        (928, 214, "Primitives", ["zero-training by design", "scan_for + center_on [hw]", "release [hw] | grasp [lock]"]),
        (690, 330, "Perception", ["camera by device name", "HSV three-state:", "FOUND/UNCERTAIN/NONE"]),
        (928, 330, "Locate", ["multi-pose pixel rays", "triangulation -> mm", "kills per-pose anchors"]),
    ]
    for cx, cy, title, body in chips:
        rrect(d, [cx, cy, cx + 232, cy + 96], r=10, fill=BOX2,
              outline=(48, 66, 96), width=1)
        d.text((cx + 14, cy + 10), title, font=F_ITEM, fill=WHITE)
        yy = cy + 38
        for b in body:
            d.text((cx + 14, yy), b, font=F_SMALL, fill=BODY)
            yy += 19
    rrect(d, [690, 442, 1166, 486], r=8, fill=BOX2, outline=(48, 66, 96), width=1)
    d.text((704, 452), "evidence log: each step observed + verified",
           font=F_BODY, fill=BODY)
    d.text((690, 500), "LLM plan enters as a TaskSpec-approved primitive sequence",
           font=F_SMALL, fill=MUTED)

    arrow(d, (1180, 338), (1216, 338))

    # D: safety executor
    rrect(d, [1216, 138, 1652, 538], outline=ORANGE)
    d.text((1234, 156), "SAFETY EXECUTOR", font=F_BOX, fill=WHITE)
    d.text((1234, 188), "reflex layer - never LLM  |  executor/", font=F_BOXSUB, fill=MUTED)
    gates = [
        "current spike fuse 1.2 A -> auto-retract",
        "contextual feedback: current + motion",
        "z-clearance tiers 30 / 70 / 90 / 120 mm",
        "Cartesian checks: 6 mm/step, z-lock",
        "contact + low-pose recovery [locked]",
        "adaptive prepare() above 120 mm",
    ]
    y = 226
    for g in gates:
        d.text((1234, y), g, font=F_BODY, fill=BODY)
        y += 27
    pill(d, 1234, 420, "real-hw validated", GREEN)
    rrect(d, [1234, 462, 1634, 506], r=8, fill=(38, 18, 18), outline=RED, width=1)
    d.text((1248, 472), "UnsafeStateError: halt task, keep evidence",
           font=F_BODY, fill=RED)

    # fuse path D -> C (above the main row)
    elbow(d, [(1434, 138), (1434, 118), (940, 118), (940, 134)],
          color=RED, width=2, dashed=True)
    d.text((700, 104), "UnsafeStateError -> halt + evidence", font=F_SMALL, fill=RED)

    # ---------- bottom row (y 598..958) ----------
    # F: simulation twin
    rrect(d, [28, 598, 520, 958])
    d.text((46, 616), "SIMULATION TWIN", font=F_BOX, fill=WHITE)
    d.text((46, 648), "PyBullet, original vendor STL/URDF", font=F_BOXSUB, fill=MUTED)
    lines(d, 46, 684, [
        ("HEADLESS on dev box + Jetson", F_BODY, BODY),
        ("deterministic rehearsal of primitives", F_BODY, BODY),
        ("7-frame regression: home err 2.4 mm", F_BODY, BODY),
    ])
    dashed_rect(d, [46, 778, 502, 926])
    d.text((62, 792), "M1.7 residual-RL plugin", font=F_ITEM, fill=WHITE)
    lines(d, 62, 822, [
        ("bounded joint/Cartesian residuals", F_BODY, BODY),
        ("executor keeps actuation authority", F_BODY, BODY),
        ("disable plugin -> deterministic", F_BODY, BODY),
        ("primitive stays the default", F_BODY, BODY),
    ], gap=22)

    # E: backends
    rrect(d, [560, 598, 1100, 958])
    d.text((578, 616), "BACKENDS", font=F_BOX, fill=WHITE)
    d.text((578, 648), "capability matrix  |  executor/backends.py", font=F_BOXSUB, fill=MUTED)
    rrect(d, [578, 680, 1082, 800], r=10, fill=BOX2, outline=(48, 66, 96), width=1)
    d.text((594, 692), "CreateBlock 5-DOF  [active]", font=F_ITEM, fill=WHITE)
    d.text((594, 720), "CLB-S325 bus servos via Jetson Nano", font=F_BODY, fill=BODY)
    d.text((594, 744), "state + current telemetry", font=F_BODY, fill=BODY)
    pill(d, (1082 - int(d.textlength("gated, real-hw validated", font=F_PILL)) - 33),
         766, "gated, real-hw validated", GREEN)
    rrect(d, [578, 820, 1082, 940], r=10, fill=BOX2, outline=(48, 66, 96), width=1)
    d.text((594, 832), "SO-ARM100/101  [adapter]", font=F_ITEM, fill=WHITE)
    d.text((594, 860), "Feetech STS3215, LeRobot semantics", font=F_BODY, fill=BODY)
    d.text((594, 884), "state reads only", font=F_BODY, fill=BODY)
    pill(d, 594, 906, "motion locked pending calibration", AMBER)

    # G: runtimes
    rrect(d, [1140, 598, 1652, 958])
    d.text((1158, 616), "RUNTIMES", font=F_BOX, fill=WHITE)
    d.text((1158, 648), "swap executors, not code", font=F_BOXSUB, fill=MUTED)
    lines(d, 1158, 684, [
        ("SSHCodeRunner - laptop -> relay -> Nano", F_BODY, BODY),
        ("LocalCodeRunner - resident on Jetson", F_BODY, BODY),
        ("one safety contract per backend", F_BODY, BODY),
    ])
    pill(d, 1158, 796, "real-hw validated", GREEN)
    pill(d, 1158, 832, "offline-tested", GRAY)

    # ---------- gap-band arrows (y 538..598, labels on their own rows) ----------
    # D -> E: gated commands (orange elbow)
    elbow(d, [(1250, 538), (1250, 578), (1040, 578), (1040, 596)], color=ORANGE)
    d.text((1056, 556), "gated joint commands + current watchdog",
           font=F_SMALL, fill=(255, 178, 128))

    # E -> C: camera feedback (green, vertical)
    arrow(d, (880, 598), (880, 540), color=GREEN)
    d.text((690, 545), "640x480 wrist-cam frames", font=F_SMALL, fill=GREEN)

    # F -> C: rehearsal (dashed, planned)
    elbow(d, [(480, 598), (480, 578), (720, 578), (720, 540)],
          color=GRAY, width=2, dashed=True)
    d.text((495, 582), "rehearse before hardware (M3 planned)",
           font=F_SMALL, fill=GRAY)

    # ---------- legend ----------
    lx = 28
    for color, text, dash in [
        (GREEN, "validated on real hardware", False),
        (AMBER, "locked pending calibration", False),
        (GRAY, "offline-tested", False),
        (GRAY, "planned", True),
        (RED, "safety fuse path", True),
    ]:
        if dash:
            for i in range(0, 34, 10):
                d.line([(lx + i, 1004), (lx + min(i + 6, 34), 1004)],
                       fill=color, width=3)
        else:
            d.line([(lx, 1004), (lx + 34, 1004)], fill=color, width=3)
        d.text((lx + 44, 994), text, font=F_LEGEND, fill=BODY)
        lx += 44 + int(d.textlength(text, font=F_LEGEND)) + 46

    # ---------- footer ----------
    d.text((28, 1040),
           "Every box maps to code in this repo: discipline.py | loop.py | primitives/ | perception/ | locate.py | executor/",
           font=F_BODY, fill=MUTED)
    d.text((28, 1068),
           "64/64 offline tests | zero-training core | residual RL stays an optional, bounded plugin",
           font=F_BODY, fill=MUTED)

    img.save(str(args.out), quality=92)
    print("WROTE", args.out, img.size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

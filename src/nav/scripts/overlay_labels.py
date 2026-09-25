#!/usr/bin/env python3
"""User-requested capability (2026-09-24, third pass): "wider view which
captures all elements in the room and shows the simulation properly" - the
un-labeled overhead video left it genuinely ambiguous which on-screen blob
was the UGV, the UAV, the start, the goal, or the tunnel structure (this
exact ambiguity caused a real misdiagnosis earlier in this project's own
history - see step06_hybrid_3d_voxel_navigation.md's "Second pass" section).

This script post-processes an ALREADY-RECORDED overhead_view.mp4 (from
record_run.py) and draws text labels directly on each frame, using COLOR
DETECTION rather than camera-pose/world-to-pixel math - deliberately, since
this project has already been bitten multiple times by camera-orientation
assumptions that turned out backwards or mirrored. Every element this
labels is either self-illuminated (the start/goal markers use an
<emissive> material - see worlds/tunnel_test.world) or a distinctly-colored
material (UGV chassis is pure white, UAV rotors are bright orange, the
tunnel structure is a distinct warm brown - see model.sdf/world file
comments), so a simple HSV threshold reliably finds each one without
needing to know the camera's exact pose/rotation convention at all.

Usage (after record_run.py has already produced overhead_view.mp4):

    python3 src/nav/scripts/overlay_labels.py \\
        --in /path/overhead_view.mp4 --out /path/overhead_view_labeled.mp4
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

# (label, BGR draw color, HSV lower bound, HSV upper bound, min contour area,
#  "static" = compute centroid once from the first frame it's seen in and
#  reuse for every subsequent frame - used for the two markers, since the
#  UGV parking directly on top of one would otherwise make per-frame
#  detection lose the label exactly when it matters most).
_TARGETS = [
    dict(label="START", color=(0, 255, 0), lower=(45, 150, 150), upper=(75, 255, 255), min_area=15, static=True),
    dict(label="GOAL", color=(0, 0, 255), lower=(0, 150, 150), upper=(10, 255, 255), min_area=15, static=True),
    dict(label="UGV", color=(255, 255, 255), lower=(0, 0, 200), upper=(179, 40, 255), min_area=20, static=False),
    dict(label="UAV", color=(0, 140, 255), lower=(8, 120, 120), upper=(22, 255, 255), min_area=8, static=False),
]


def _find_centroid(hsv: np.ndarray, lower: tuple, upper: tuple, min_area: float) -> tuple[int, int] | None:
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])


def _draw_label(frame: np.ndarray, text: str, pos: tuple[int, int], color: tuple[int, int, int]) -> None:
    x, y = pos
    cv2.circle(frame, (x, y), 10, color, 2)
    # Black outline behind white-ish text (and vice versa for readability
    # against either a light floor or a dark wall shadow behind the label).
    outline = (0, 0, 0) if sum(color) > 380 else (255, 255, 255)
    org = (x + 14, y - 10)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, outline, 3, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def _draw_legend(frame: np.ndarray) -> None:
    h, w = frame.shape[:2]
    x0, y0 = 8, h - 8 - 20 * len(_TARGETS) - 8
    cv2.rectangle(frame, (x0 - 6, y0 - 6), (x0 + 160, h - 8), (0, 0, 0), -1)
    for i, t in enumerate(_TARGETS):
        y = y0 + 16 + i * 20
        cv2.putText(frame, t["label"], (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, t["color"], 1, cv2.LINE_AA)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--in", dest="in_path", required=True)
    parser.add_argument("--out", dest="out_path", required=True)
    args = parser.parse_args(argv)

    cap = cv2.VideoCapture(args.in_path)
    if not cap.isOpened():
        raise SystemExit(f"could not open {args.in_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*"avc1")
    writer = cv2.VideoWriter(args.out_path, fourcc, fps, (width, height))

    static_pos: dict[str, tuple[int, int]] = {}
    n = 0
    labeled_counts = {t["label"]: 0 for t in _TARGETS}
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for t in _TARGETS:
            label = t["label"]
            if t["static"] and label in static_pos:
                pos = static_pos[label]
            else:
                pos = _find_centroid(hsv, t["lower"], t["upper"], t["min_area"])
                if pos is not None and t["static"]:
                    static_pos[label] = pos
            if pos is not None:
                _draw_label(frame, label, pos, t["color"])
                labeled_counts[label] += 1
        _draw_legend(frame)
        writer.write(frame)
        n += 1

    writer.release()
    cap.release()
    print(f"overlay_labels: processed {n} frames -> {args.out_path}")
    print(f"overlay_labels: frames each label was actually found in: {labeled_counts}")


if __name__ == "__main__":
    main()

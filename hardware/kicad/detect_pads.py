"""Detect TH pads on URTK1.2 photo via HoughCircles; scale from 2.54mm pitch."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

ASSETS = Path(r"C:\Users\USER\.cursor\projects\c-project-gnss\assets")
OUT = ASSETS / "_meas"


def detect(path: Path, tag: str, roi: tuple[int, int, int, int] | None = None) -> None:
    bgr = cv2.imread(str(path))
    if bgr is None:
        raise SystemExit(f"cannot read {path}")
    if roi:
        x0, y0, x1, y1 = roi
        bgr = bgr[y0:y1, x0:x1]
        ox, oy = x0, y0
    else:
        ox = oy = 0
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    # pads are bright copper rings on black
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=12,
        param1=80,
        param2=18,
        minRadius=4,
        maxRadius=14,
    )
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    pts = []
    if circles is not None:
        circles = np.round(circles[0]).astype(int)
        for x, y, r in circles:
            pts.append((x + ox, y + oy, r))
            cv2.circle(vis, (x, y), r, (0, 255, 0), 1)
            cv2.circle(vis, (x, y), 2, (0, 0, 255), -1)
    print(f"{tag}: found {len(pts)} circles")
    out_path = OUT / f"{tag}_circles.jpg"
    cv2.imwrite(str(out_path), vis)

    if len(pts) < 8:
        return

    arr = np.array([(p[0], p[1]) for p in pts], dtype=float)
    # cluster into two vertical columns (or two horizontal rows) by x or y variance
    xs, ys = arr[:, 0], arr[:, 1]
    if xs.std() > ys.std():
        # horizontal rows — sort by y into two clusters
        order = np.argsort(ys)
        mid = len(order) // 2
        # split by median y
        med_y = np.median(ys)
        row_a = arr[ys < med_y]
        row_b = arr[ys >= med_y]
        axis = "x"
    else:
        med_x = np.median(xs)
        row_a = arr[xs < med_x]
        row_b = arr[xs >= med_x]
        axis = "y"

    def pitch_of(row: np.ndarray, ax: str) -> float:
        vals = np.sort(row[:, 0 if ax == "x" else 1])
        if len(vals) < 2:
            return float("nan")
        diffs = np.diff(vals)
        # keep diffs near the median (reject outliers)
        med = np.median(diffs)
        good = diffs[(diffs > med * 0.6) & (diffs < med * 1.4)]
        return float(np.median(good)) if len(good) else float(med)

    pa = pitch_of(row_a, axis)
    pb = pitch_of(row_b, axis)
    pitch_px = np.nanmedian([pa, pb])
    print(f"  rows {len(row_a)}+{len(row_b)}  pitch_px≈{pitch_px:.2f}  ({pa:.2f}/{pb:.2f})")
    if not np.isfinite(pitch_px) or pitch_px < 5:
        return
    mmpp = 2.54 / pitch_px

    # row spacing
    if axis == "x":
        spacing = abs(row_a[:, 1].mean() - row_b[:, 1].mean()) * mmpp
        span_a = (row_a[:, 0].max() - row_a[:, 0].min()) * mmpp
        span_b = (row_b[:, 0].max() - row_b[:, 0].min()) * mmpp
    else:
        spacing = abs(row_a[:, 0].mean() - row_b[:, 0].mean()) * mmpp
        span_a = (row_a[:, 1].max() - row_a[:, 1].min()) * mmpp
        span_b = (row_b[:, 1].max() - row_b[:, 1].min()) * mmpp
    print(f"  row spacing≈{spacing:.2f} mm  span A/B≈{span_a:.2f}/{span_b:.2f} mm")

    # mounting holes: larger circles
    holes = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=40,
        param1=80,
        param2=22,
        minRadius=10,
        maxRadius=28,
    )
    if holes is not None:
        holes = np.round(holes[0]).astype(int)
        print(f"  candidate holes: {len(holes)}")
        for x, y, r in holes:
            cv2.circle(vis, (x, y), r, (255, 255, 0), 2)
            print(f"    hole px=({x+ox},{y+oy}) r={r}")
        if len(holes) >= 2:
            harr = holes[:, :2].astype(float)
            # pairwise distances
            for i in range(len(harr)):
                for j in range(i + 1, len(harr)):
                    d = np.linalg.norm(harr[i] - harr[j]) * mmpp
                    print(f"    hole dist {i}-{j}: {d:.2f} mm")
    cv2.imwrite(str(out_path), vis)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    # Full frames — ROIs tuned to PCB body
    jobs = [
        (
            "ab",
            ASSETS / "ab51691e-be6c-40dd-a024-33a92b3324d2-ae036590-40c7-4c68-95d3-42c6f5e8ea5b.jpg",
            (50, 300, 530, 780),
        ),
        (
            "da",
            ASSETS / "da6c483c-3c50-489d-8b23-1cb18b366001-62b4f619-7a1f-427c-9168-7e257fa71e21.jpg",
            (60, 200, 520, 780),
        ),
        (
            "s38",
            ASSETS / "S__193986569_0-ef556851-39d4-49fe-8af6-9f8ce734370a.jpg",
            (60, 180, 520, 720),
        ),
        (
            "b78",
            ASSETS / "b78d627b-d422-401f-84f4-4032b2d613da-ca4a94c1-e183-4a0a-8e04-c99937269934.jpg",
            (40, 220, 540, 700),
        ),
    ]
    for tag, path, roi in jobs:
        detect(path, tag, roi)

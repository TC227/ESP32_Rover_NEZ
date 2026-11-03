"""Measure URTK1.2 board from photos using pin pitch as scale."""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ASSETS = Path(r"C:\Users\USER\.cursor\projects\c-project-gnss\assets")
OUT = ASSETS / "_meas"
OUT.mkdir(exist_ok=True)

# Manually annotated pad centers (px) from top-down photos.
# ab51691e: USB left, SMA right, headers on top & bottom.
# Measured by inspecting grid overlays / photo.
# Pitch reference = 2.54 mm between adjacent pads in a row.

PHOTOS = {
    # USB left / SMA right — headers top(SPI) and bottom(CTRL)
    "ab51691e": {
        "path": ASSETS / "ab51691e-be6c-40dd-a024-33a92b3324d2-ae036590-40c7-4c68-95d3-42c6f5e8ea5b.jpg",
        # top row L->R (USB->SMA): GND 5V 3V3 TX2 RX2 CS RX/MOSI TX/MISO SCK (+?)
        "top_pads": [
            (95, 195),
            (130, 195),
            (165, 195),
            (200, 195),
            (235, 195),
            (270, 195),
            (305, 195),
            (340, 195),
            (375, 195),
        ],
        # bottom row L->R: GND 3V3 SDA SCL FENCE RTK PPS RST SAFE INT
        "bot_pads": [
            (95, 545),
            (130, 545),
            (165, 545),
            (200, 545),
            (235, 545),
            (270, 545),
            (305, 545),
            (340, 545),
            (375, 545),
            (410, 545),
        ],
        "holes": [(70, 170), (480, 170), (70, 570), (480, 570)],
        "board": (55, 155, 500, 590),  # x0,y0,x1,y1
    },
}


def analyze(name: str, data: dict) -> None:
    im = Image.open(data["path"]).convert("RGB")
    draw = ImageDraw.Draw(im)
    top = np.array(data["top_pads"], dtype=float)
    bot = np.array(data["bot_pads"], dtype=float)

    # average pitch in px from both rows
    pitches = []
    for row in (top, bot):
        for i in range(len(row) - 1):
            pitches.append(np.linalg.norm(row[i + 1] - row[i]))
    pitch_px = float(np.median(pitches))
    mm_per_px = 2.54 / pitch_px
    print(f"\n=== {name} ===")
    print(f"median pitch_px={pitch_px:.2f}  mm/px={mm_per_px:.5f}")

    # row spacing (mean y of top vs bottom)
    row_dy = abs(bot[:, 1].mean() - top[:, 1].mean()) * mm_per_px
    print(f"header row spacing ≈ {row_dy:.2f} mm")

    # board size
    x0, y0, x1, y1 = data["board"]
    bw = (x1 - x0) * mm_per_px
    bh = (y1 - y0) * mm_per_px
    print(f"board outline ≈ {bw:.2f} x {bh:.2f} mm")

    holes = np.array(data["holes"], dtype=float)
    # hole distances
    hx = abs(holes[1, 0] - holes[0, 0]) * mm_per_px
    hy = abs(holes[2, 1] - holes[0, 1]) * mm_per_px
    print(f"hole spacing X≈{hx:.2f} mm  Y≈{hy:.2f} mm")

    # annotate
    draw.rectangle([x0, y0, x1, y1], outline=(255, 0, 0), width=2)
    for p in top:
        draw.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], outline=(0, 255, 0), width=2)
    for p in bot:
        draw.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], outline=(0, 200, 255), width=2)
    for p in holes:
        draw.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], outline=(255, 255, 0), width=2)
    im.save(OUT / f"{name}_annot.jpg", quality=90)
    print(f"wrote {OUT / (name + '_annot.jpg')}")


def make_grids() -> None:
    files = [
        "ab51691e-be6c-40dd-a024-33a92b3324d2-ae036590-40c7-4c68-95d3-42c6f5e8ea5b.jpg",
        "da6c483c-3c50-489d-8b23-1cb18b366001-62b4f619-7a1f-427c-9168-7e257fa71e21.jpg",
        "b78d627b-d422-401f-84f4-4032b2d613da-ca4a94c1-e183-4a0a-8e04-c99937269934.jpg",
        "S__193986569_0-ef556851-39d4-49fe-8af6-9f8ce734370a.jpg",
        "229577b2-8f44-408b-bd67-58b05abc96dd-f305c5fa-7c08-42c3-9ea1-861c05f46db4.jpg",
        "82a60912-e526-4548-92e9-c78d498a9722-05b8a312-d0e6-499e-8298-2a9dbbd86da9.jpg",
    ]
    for f in files:
        path = ASSETS / f
        im = Image.open(path).convert("RGB")
        W, H = im.size
        dr = ImageDraw.Draw(im)
        for x in range(0, W, 50):
            dr.line([(x, 0), (x, H)], fill=(180, 40, 40), width=1)
            dr.text((x + 2, 2), str(x), fill=(255, 80, 80))
        for y in range(0, H, 50):
            dr.line([(0, y), (W, y)], fill=(40, 180, 40), width=1)
            dr.text((2, y + 2), str(y), fill=(80, 255, 80))
        tag = f[:12]
        im.save(OUT / f"{tag}_grid.jpg", quality=85)
        print("grid", tag, W, H)


if __name__ == "__main__":
    make_grids()
    for name, data in PHOTOS.items():
        analyze(name, data)

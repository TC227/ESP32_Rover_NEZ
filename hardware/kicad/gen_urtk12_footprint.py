"""Generate EasyEDA Standard footprint for URTK1.2 (SinoGNSS K902 breakout).

Dimensions from caliper photos (2026-11-03):
  - PCB body ≈ 38.0 × 38.0 mm (excl. SMA / USB-C overhang)
  - Mounting holes ≈ 30.0 × 30.0 mm centers, Ø2.2 mm (M2 clearance)
  - Headers 2.54 mm pitch; row spacing 34.0 mm (pad centers ≈ 2.0 mm from edge)
  - Left 1×9  (SMA end → USB end): SCK … GND
  - Right 1×10 (SMA end → USB end): INT … GND
  - Origin = board center; SMA / RF toward +Y; USB-C / JST toward −Y

Import in EasyEDA Standard:
  File → Import → EasyEDA… → URTK1.2_K902.json
"""
from __future__ import annotations

import json
from pathlib import Path

OUT = Path(__file__).with_name("easyeda") / "URTK1.2_K902.json"
DOC = Path(__file__).with_name("easyeda") / "URTK1.2_README.md"

# --- Mechanical (mm) ---
BOARD_W = 38.0  # X: left/right headers
BOARD_H = 38.0  # Y: SMA(+Y) ↔ USB(−Y)
HOLE_DX = 30.0
HOLE_DY = 30.0
HOLE_D = 2.2
PITCH = 2.54
ROW_X = 17.0  # ±X of pad centers (38/2 - 2.0)
PAD_D = 1.7
DRILL = 1.0

# Pin1 toward +Y (SMA). GND toward −Y (USB).
LEFT = ["SCK", "TX_MISO", "RX_MOSI", "CS", "RX2", "TX2", "3V3", "5V", "GND"]  # 1..9
RIGHT = ["INT", "SAFE", "RST", "PPS", "RTK", "FENCE", "SCL", "SDA", "3V3", "GND"]  # 10..19

# EasyEDA unit: 1 unit = 0.254 mm (10 mil)
S = 1.0 / 0.254


def u(mm: float) -> float:
    return mm * S


def gid() -> str:
    gid.n += 1  # type: ignore[attr-defined]
    return f"gge{gid.n}"


gid.n = 0  # type: ignore[attr-defined]


def pad(number: str, name: str, x: float, y: float, rect: bool = False) -> str:
    """TH pad at mm coords, EasyEDA canvas origin at package origin."""
    ex, ey = u(x), -u(y)  # EasyEDA Y down
    shape = "RECT" if rect else "ELLIPSE"
    w = h = u(PAD_D)
    hole = u(DRILL) / 2.0  # radius
    net = name
    # PAD~shape~cx~cy~w~h~layer~net~number~holeR~points~rot~id~holeLen~slot~plated~locked~attr
    pts = ""
    if rect:
        half = w / 2
        pts = f"{ex-half} {ey-half} {ex+half} {ey-half} {ex+half} {ey+half} {ex-half} {ey+half}"
    return (
        f"PAD~{shape}~{ex:.4f}~{ey:.4f}~{w:.4f}~{h:.4f}~11~{net}~{number}~{hole:.4f}~"
        f"{pts}~0~{gid()}~0~~Y~0~{ex:.4f},{ey:.4f}"
    )


def hole(x: float, y: float) -> str:
    ex, ey = u(x), -u(y)
    r = u(HOLE_D) / 2.0
    return f"HOLE~{ex:.4f}~{ey:.4f}~{r:.4f}~{gid()}~0"


def track_rect(x0: float, y0: float, x1: float, y1: float, layer: int, width: float = 0.15) -> str:
    pts = [
        (u(x0), -u(y0)),
        (u(x1), -u(y0)),
        (u(x1), -u(y1)),
        (u(x0), -u(y1)),
        (u(x0), -u(y0)),
    ]
    coord = " ".join(f"{x:.4f} {y:.4f}" for x, y in pts)
    return f"TRACK~{u(width):.4f}~{layer}~~{coord}~{gid()}~0"


def text(txt: str, x: float, y: float, layer: int = 3, size: float = 0.8) -> str:
    # TEXT~type~x~y~rot~size?  (EasyEDA silk)
    return (
        f"TEXT~L~{u(x):.4f}~{-u(y):.4f}~{u(size):.4f}~0~0~{layer}~~"
        f"{len(txt)}~{txt}~~1~{gid()}~Arial~0"
    )


def silk_label(txt: str, x: float, y: float) -> str:
    return text(txt, x, y, layer=3, size=0.55)


def build() -> dict:
    shapes: list[str] = []

    # Courtyard / silk board outline
    hw, hh = BOARD_W / 2, BOARD_H / 2
    shapes.append(track_rect(-hw, -hh, hw, hh, layer=3, width=0.2))
    shapes.append(track_rect(-hw - 0.2, -hh - 0.2, hw + 0.2, hh + 0.2, layer=13, width=0.1))

    # SMA overhang keepout (+Y)
    shapes.append(track_rect(-5.0, hh, 5.0, hh + 10.0, layer=3, width=0.12))
    shapes.append(silk_label("SMA", -2.5, hh + 5.0))

    # USB-C overhang keepout (−Y)
    shapes.append(track_rect(-6.0, -hh - 3.5, -1.0, -hh, layer=3, width=0.12))
    shapes.append(silk_label("USB", -5.5, -hh - 2.0))
    shapes.append(track_rect(1.0, -hh - 3.5, 8.0, -hh, layer=3, width=0.12))
    shapes.append(silk_label("JST", 2.0, -hh - 2.0))

    # Module body hint
    shapes.append(track_rect(-11.0, -8.5, 11.0, 8.5, layer=3, width=0.12))
    shapes.append(silk_label("K902", -4.0, 0.5))
    shapes.append(silk_label("URTK1.2", -6.0, -2.0))

    # Mounting holes
    for hx, hy in [
        (-HOLE_DX / 2, HOLE_DY / 2),
        (HOLE_DX / 2, HOLE_DY / 2),
        (-HOLE_DX / 2, -HOLE_DY / 2),
        (HOLE_DX / 2, -HOLE_DY / 2),
    ]:
        shapes.append(hole(hx, hy))

    # GND of both rows share the same Y (right has one extra pin toward +Y / SMA).
    n_l, n_r = len(LEFT), len(RIGHT)
    gnd_y = -((n_r - 1) / 2) * PITCH  # bottom of the taller (right) column
    for i, name in enumerate(LEFT):
        y = gnd_y + (n_l - 1 - i) * PITCH
        num = str(i + 1)
        shapes.append(pad(num, name, -ROW_X, y, rect=(i == 0)))
        shapes.append(silk_label(name, -ROW_X - 5.2, y - 0.3))

    for i, name in enumerate(RIGHT):
        y = gnd_y + (n_r - 1 - i) * PITCH
        num = str(n_l + i + 1)
        shapes.append(pad(num, name, ROW_X, y, rect=(i == 0)))
        shapes.append(silk_label(name, ROW_X + 1.8, y - 0.3))

    # Reference / value placeholders
    shapes.append(text("M1", -hw, hh + 2.5, layer=3, size=1.0))
    shapes.append(text("URTK1.2", -hw, -hh - 5.5, layer=3, size=0.9))

    # Crosshair at origin
    shapes.append(
        f"TRACK~{u(0.12):.4f}~12~~{u(-2):.4f} {0:.4f} {u(2):.4f} {0:.4f}~{gid()}~0"
    )
    shapes.append(
        f"TRACK~{u(0.12):.4f}~12~~{0:.4f} {u(-2):.4f} {0:.4f} {u(2):.4f}~{gid()}~0"
    )

    box = {
        "x": -u(hw + 12),
        "y": -u(hh + 12),
        "width": u(BOARD_W + 24),
        "height": u(BOARD_H + 24),
    }

    return {
        "head": {
            "docType": "4",
            "editorVersion": "6.5.51",
            "newgId": True,
            "c_para": {
                "package": "URTK1.2_K902",
                "pre": "M",
                "Contributor": "ESP32_Rover_NEZ",
                "link": "",
                "addIntoLib": "yes",
            },
            "x": "0",
            "y": "0",
            "hasIdFlag": True,
        },
        "canvas": "CA~1000~1000~#000000~yes~#FFFFFF~10~1000~1000~line~0.5~mm~1~45~~0.5~0~0~0",
        "shape": shapes,
        "layers": [
            "1~TopLayer~#FF0000~true~true~true~",
            "2~BottomLayer~#0000FF~true~false~true~",
            "3~TopSilkLayer~#FFCC00~true~false~true~",
            "4~BottomSilkLayer~#66CC33~true~false~true~",
            "5~TopPasteMaskLayer~#808080~true~false~true~",
            "6~BottomPasteMaskLayer~#800000~true~false~true~",
            "7~TopSolderMaskLayer~#800080~true~false~true~0.3",
            "8~BottomSolderMaskLayer~#AA00FF~true~false~true~0.3",
            "9~Ratlines~#6464FF~true~false~true~",
            "10~BoardOutLine~#FF00FF~true~false~true~",
            "11~Multi-Layer~#C0C0C0~true~false~true~",
            "12~Document~#FFFFFF~true~false~true~",
            "13~TopAssembly~#33CC99~true~false~true~",
            "14~BottomAssembly~#5555FF~true~false~true~",
            "15~Mechanical~#F022F0~false~false~false~",
        ],
        "objects": [
            "All~true~false",
            "Component~true~true",
            "Prefix~true~true",
            "Name~true~false",
            "Track~true~true",
            "Pad~true~true",
            "Via~true~true",
            "Hole~true~true",
            "Copper_Area~true~true",
            "Circle~true~true",
            "Text~true~true",
        ],
        "BBox": box,
    }


def write_doc() -> None:
    lines = [
        "# URTK1.2 / SinoGNSS K902 — EasyEDA footprint",
        "",
        "ไฟล์: `URTK1.2_K902.json` (EasyEDA **Standard** package)",
        "",
        "## วิธีเปิด",
        "",
        "1. ไป https://easyeda.com/editor แล้วล็อกอิน",
        "2. **File → Import → EasyEDA…**",
        "3. เลือก `hardware/kicad/easyeda/URTK1.2_K902.json`",
        "4. บันทึกลง Library แล้ววางบนบอร์ดได้",
        "",
        "## ขนาดจากรูปคาลิเปอร์ (ประมาณ)",
        "",
        "| รายการ | ค่า |",
        "| --- | --- |",
        f"| ตัวบอร์ด (ไม่รวม SMA/USB) | **{BOARD_W:.1f} × {BOARD_H:.1f} mm** |",
        f"| รูยึด (ศูนย์ถึงศูนย์) | **{HOLE_DX:.1f} × {HOLE_DY:.1f} mm** Ø{HOLE_D} |",
        f"| พิตช์หัวพิน | **{PITCH} mm** |",
        f"| ระยะแถวซ้าย–ขวา (ศูนย์แพด) | **{ROW_X * 2:.1f} mm** |",
        "| ทิศ SMA / RF | **+Y** |",
        "| ทิศ USB-C + JST | **−Y** |",
        "",
        "## พินเอาต์",
        "",
        "### ซ้าย (ขา 1–9) — SPI / UART2  ปลาย +Y = SMA",
        "",
        "| # | ชื่อ |",
        "| --- | --- |",
    ]
    for i, n in enumerate(LEFT, 1):
        lines.append(f"| {i} | {n} |")
    lines += [
        "",
        "### ขวา (ขา 10–19) — control / I2C  ปลาย +Y = SMA",
        "",
        "| # | ชื่อ |",
        "| --- | --- |",
    ]
    for i, n in enumerate(RIGHT, 10):
        lines.append(f"| {i} | {n} |")
    lines += [
        "",
        "## หมายเหตุ",
        "",
        "- ค่าจากรูปคาลิเปอร์ (±0.5 mm) — ถ้าจะสั่งโรงงานให้วัดซ้ำด้วยดิจิทัลคาลิเปอร์",
        "- กรอบ `gnss-head` เดิมใช้ K902 **48×42** / แถวพิน **45 mm** คนละชุดกับ footprint นี้",
        "- TX2/RX2 = COM2 TTL 3.3 V (ต่อ ESP32 UART2)",
        "- JST บนบอร์ด: 5V / TX1 / RX1 / NC / NC / GND",
        "",
        "สร้างใหม่: `python hardware/kicad/gen_urtk12_footprint.py`",
        "",
    ]
    DOC.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    data = build()
    OUT.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    write_doc()
    print(f"wrote {OUT}")
    print(f"wrote {DOC}")
    print(f"pads: L={len(LEFT)} R={len(RIGHT)}  board={BOARD_W}x{BOARD_H}  row={ROW_X*2}")


if __name__ == "__main__":
    main()

"""Export EasyEDA-importable files from gnss-head.kicad_pcb."""
from __future__ import annotations

import json
import math
import re
import zipfile
from pathlib import Path

PCB = Path(__file__).with_name("gnss-head.kicad_pcb")
PRO = Path(__file__).with_name("gnss-head.kicad_pro")
SCH = Path(__file__).with_name("gnss-head.kicad_sch")
OUT = Path(__file__).with_name("easyeda")
CX, CY, DIA = 60.0, 60.0, 120.0
MM_TO_MIL = 1000.0 / 25.4


def parse_pcb(text: str):
    pads = []
    segs = []
    vias = []
    holes = []
    chunks = re.split(r"\n\t\(footprint ", text)
    for chunk in chunks[1:]:
        mat = re.search(r"\(at ([-\d.]+) ([-\d.]+)", chunk)
        if not mat:
            continue
        fx, fy = float(mat.group(1)), float(mat.group(2))
        for pm in re.finditer(
            r'\(pad "([^"]*)" (\w+) circle\s+\(at ([-\d.]+) ([-\d.]+).*?\(size ([-\d.]+).*?\(drill ([-\d.]+)',
            chunk,
            re.S,
        ):
            name, ptype, px, py, sz, dr = pm.group(1), pm.group(2), float(pm.group(3)), float(pm.group(4)), float(pm.group(5)), float(pm.group(6))
            netm = re.search(r'\(net (\d+) "([^"]*)"\)', pm.group(0)) if False else re.search(
                r'\(net (\d+) "([^"]*)"\)', chunk[pm.start() : pm.start() + 400]
            )
            net = netm.group(2) if netm else ""
            x, y = fx + px, fy + py
            if ptype == "npth":
                holes.append((x, y, dr))
            else:
                pads.append((x, y, sz, dr, net, name))
    for sm in re.finditer(
        r'\(segment \(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\) '
        r'\(width ([-\d.]+)\) \(layer "([^"]+)"\) \(net (\d+)',
        text,
    ):
        segs.append(
            (
                float(sm.group(1)),
                float(sm.group(2)),
                float(sm.group(3)),
                float(sm.group(4)),
                float(sm.group(5)),
                sm.group(6),
                int(sm.group(7)),
            )
        )
    for vm in re.finditer(
        r'\(via \(at ([-\d.]+) ([-\d.]+)\) \(size ([-\d.]+)\) \(drill ([-\d.]+)\).*?\(net (\d+)',
        text,
    ):
        vias.append((float(vm.group(1)), float(vm.group(2)), float(vm.group(3)), float(vm.group(4)), int(vm.group(5))))
    nets = {int(n): name for n, name in re.findall(r'\(net (\d+) "([^"]*)"\)', text)}
    return pads, segs, vias, holes, nets


# EasyEDA Std PCB stores everything in 10 mil. 1 unit = 0.254 mm.
S = 1.0 / 0.254
OX, OY = 4000.0, 3000.0  # board center in EasyEDA


def to_e(x: float, y: float) -> tuple[float, float]:
    return OX + (x - CX) * S, OY - (y - CY) * S


def u(mm: float) -> float:
    return mm * S


def gerber_file(name: str, body: str) -> str:
    return (
        f"G04 {name}*\n%FSLAX36Y36*%\n%MOMM*%\n%LPD*%\n"
        + body
        + "M02*\n"
    )


def xy(x: float, y: float) -> str:
    return f"X{int(round(x * 1e6))}Y{int(round(y * 1e6))}"


def write_gerbers(pads, segs, vias, holes, dest: Path) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    # Edge
    edge = ["%ADD10C,0.150000*%", "G36*"]
    n = 128
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        x, y = CX + 60 * math.cos(a), CY + 60 * math.sin(a)
        edge.append(xy(x, y) + ("D02*" if i == 0 else "D01*"))
    edge += ["G37*"]
    (dest / "gnss-head-Edge_Cuts.gbr").write_text(gerber_file("Edge.Cuts", "\n".join(edge) + "\n"), encoding="utf-8")

    for layer, fname in (("F.Cu", "gnss-head-F_Cu.gbr"), ("B.Cu", "gnss-head-B_Cu.gbr")):
        lines = ["%ADD10C,1.700000*%", "%ADD11C,0.350000*%", "%ADD12C,0.500000*%", "%ADD13C,0.800000*%", "%ADD14C,2.000000*%"]
        lines.append("D10*")
        for x, y, sz, dr, _net, _n in pads:
            lines.append(f"%ADD20C,{sz:.4f}*%")
            lines.append("D20*")
            lines.append(xy(x, y) + "D03*")
        lines.append("D13*")
        for x, y, sz, _dr, _net in vias:
            lines.append(xy(x, y) + "D03*")
        for x0, y0, x1, y1, w, ly, _net in segs:
            if ly != layer:
                continue
            ap = "D11*" if w < 0.45 else "D12*"
            lines.append(ap)
            lines.append(xy(x0, y0) + "D02*")
            lines.append(xy(x1, y1) + "D01*")
        (dest / fname).write_text(gerber_file(layer, "\n".join(lines) + "\n"), encoding="utf-8")

    # Excellon
    drill = ["M48", "METRIC,TZ", "T01C1.000", "T02C0.400", "T03C3.200", "T04C2.600", "%", "G05"]
    drill.append("T01")
    for x, y, sz, dr, _net, _n in pads:
        if 0.8 < dr < 1.3:
            drill.append(f"X{x:.3f}Y{y:.3f}")
    drill.append("T02")
    for x, y, _sz, dr, _net in vias:
        drill.append(f"X{x:.3f}Y{y:.3f}")
    drill.append("T03")
    for x, y, dr in holes:
        if dr >= 3.0:
            drill.append(f"X{x:.3f}Y{y:.3f}")
    drill.append("T04")
    for x, y, dr in holes:
        if dr < 3.0:
            drill.append(f"X{x:.3f}Y{y:.3f}")
    drill += ["T00", "M30"]
    (dest / "gnss-head.drl").write_text("\n".join(drill) + "\n", encoding="utf-8")


LAYERS = [
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
    "19~3DModel~#66CCFF~true~false~true~",
    "99~ComponentShapeLayer~#00CCCC~true~false~true~0.4",
    "100~LeadShapeLayer~#CC9999~true~false~true~",
    "101~ComponentPolarityLayer~#66FFCC~true~false~true~",
    "Hole~Hole~#222222~~false~true~",
    "DRCError~DRCError~#FAD609~~false~true~",
]

# Official EasyEDA / LCSC 3D models (JLCEDA official library)
MODEL_F15 = ("5c1467aba2e940869536815f86b3712f", "HDR-TH_15P-P2.54-V-F-1")
MODEL_F10 = ("7f52dc44ad4a48719f0311acf9f06112", "HDR-TH_10P-P2.54-F-LI-1X10P")
MODEL_M6 = ("52ffcc116df9420ab16e225cfff5249c", "HDR2.54-M-LI-1X6P")
MODEL_PH2 = ("ee6b32b5c03144688a5663b32f9648c4", "CONN-TH_PH2.00-LI-2P")

OBJECTS = [
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
    "Arc~true~true",
    "Solid_Region~true~true",
    "Text~true~true",
    "Image~true~true",
    "Rect~true~true",
    "Dimension~true~true",
    "Protractor~true~true",
]


def write_easyeda_std(pads, segs, vias, holes, nets) -> str:
    gid = 1

    def nid() -> str:
        nonlocal gid
        gid += 1
        return f"gge{gid}"

    shapes = []
    n = 64
    pts = []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        mx, my = to_e(CX + 60 * math.cos(a), CY + 60 * math.sin(a))
        pts.append(f"{mx:.4f} {my:.4f}")

    shapes.append(f"TRACK~0.6~10~~{' '.join(pts)}~{nid()}~0")
    shapes.append(f"CIRCLE~{OX:.4f}~{OY:.4f}~{u(60):.4f}~0.6~10~{nid()}~0~~")

    def ee_path(rect: tuple[float, float, float, float]) -> str:
        x0, y0, x1, y1 = rect
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        cmds = []
        for i, (x, y) in enumerate(corners):
            ex, ey = to_e(x, y)
            cmds.append(f"{'M' if i == 0 else 'L'} {ex:.4f} {ey:.4f}")
        return " ".join(cmds) + " Z"

    def ee_box_pts(rect: tuple[float, float, float, float]) -> str:
        x0, y0, x1, y1 = rect
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        return " ".join(f"{to_e(x,y)[0]:.4f} {to_e(x,y)[1]:.4f}" for x, y in corners)

    def pad_th(num: str, x: float, y: float, net: str, first: bool = False) -> str:
        ex, ey = to_e(x, y)
        w = u(1.7)
        hr = u(0.5)
        shape = "RECT" if first else "ELLIPSE"
        poly = ""
        if first:
            s = w / 2
            poly = f"{ex-s:.4f} {ey-s:.4f} {ex+s:.4f} {ey-s:.4f} {ex+s:.4f} {ey+s:.4f} {ex-s:.4f} {ey+s:.4f}"
        return (
            f"PAD~{shape}~{ex:.4f}~{ey:.4f}~{w:.4f}~{w:.4f}~11~{net}~{num}~{hr:.4f}~"
            f"{poly}~0~{nid()}~0~~Y~0~~~{ex:.4f},{ey:.4f}"
        )

    def text_pn(kind: str, x: float, y: float, txt: str, silk: str) -> str:
        ex, ey = to_e(x, y)
        mirror = "1" if silk == "4" else "0"
        return (
            f"TEXT~{kind}~{ex:.4f}~{ey:.4f}~0.8~0~{mirror}~{silk}~~5~{txt}~~1~{nid()}~Arial~0"
        )

    def text_label(txt: str, x: float, y: float, silk: str) -> str:
        ex, ey = to_e(x, y)
        mirror = "1" if silk == "4" else "0"
        return (
            f"TEXT~L~{ex:.4f}~{ey:.4f}~0.8~0~{mirror}~{silk}~~6~{txt}~~1~{nid()}~Arial~0"
        )

    def lib(cx: float, cy: float, package: str, pre: str, layer: str, kids: list[str]) -> str:
        ex, ey = to_e(cx, cy)
        c_para = f"package`{package}`pre`{pre}`Contributor`ESP32_Rover_NEZ`Add into BOM`yes`"
        # PCB footprint: LIB~x~y~c_para~rot~importFlag~gId~layer~uuid~utime~locked
        header = f"LIB~{ex:.4f}~{ey:.4f}~{c_para}~0~0~{nid()}~{layer}~~~0"
        return header + "".join(f"#@${k}" for k in kids)

    def header_device(
        ref: str,
        value: str,
        package: str,
        pins: list[tuple[str, float, float, str]],
        layer: str,
        model: tuple[str, str] | None,
        rot: str,
        z: str,
    ) -> str:
        xs = [p[1] for p in pins]
        ys = [p[2] for p in pins]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        along_y = abs(max(ys) - min(ys)) > abs(max(xs) - min(xs))
        if along_y:
            body = (min(xs) - 1.3, min(ys) - 1.4, max(xs) + 1.3, max(ys) + 1.4)
        else:
            body = (min(xs) - 1.4, min(ys) - 1.3, max(xs) + 1.4, max(ys) + 1.3)
        silk = "3" if layer == "1" else "4"
        kids = [
            text_pn("P", cx, max(ys) + 2.2, ref, silk),
            text_pn("N", cx, min(ys) - 2.2, value, silk),
            f"SOLIDREGION~99~~{ee_path(body)}~solid~{nid()}~~~~0",
            f"TRACK~1~{silk}~~{ee_box_pts(body)}~{nid()}~0",
        ]
        for i, (num, x, y, net) in enumerate(pins):
            kids.append(pad_th(num, x, y, net, first=(i == 0)))
        # No SVGNODE 3D in Standard — cloud models hang / fail on slow net.
        return lib(cx, cy, package, ref, layer, kids)

    def module_device(
        ref: str,
        value: str,
        package: str,
        body: tuple[float, float, float, float],
        layer: str,
        holes_xy: list[tuple[float, float]] | None = None,
        extra_pads: list[tuple[str, float, float, str]] | None = None,
    ) -> str:
        x0, y0, x1, y1 = body
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        silk = "3" if layer == "1" else "4"
        kids = [
            text_pn("P", x0 + 1.2, y1 + 1.6, ref, silk),
            text_pn("N", x0 + 1.2, y0 - 1.8, value, silk),
            f"SOLIDREGION~99~~{ee_path(body)}~solid~{nid()}~~~~0",
            f"SOLIDREGION~{silk}~~{ee_path(body)}~solid~{nid()}~~~~0",
            f"TRACK~1.2~{silk}~~{ee_box_pts(body)}~{nid()}~0",
        ]
        if layer == "2":
            kids.append(f"TRACK~1.2~3~~{ee_box_pts(body)}~{nid()}~0")
        if extra_pads:
            for i, (num, x, y, net) in enumerate(extra_pads):
                kids.append(pad_th(num, x, y, net, first=(i == 0)))
        if holes_xy:
            for hx, hy in holes_xy:
                ex, ey = to_e(hx, hy)
                kids.append(f"HOLE~{ex:.4f}~{ey:.4f}~{u(1.3):.4f}~{nid()}~0")
        return lib(cx, cy, package, ref, layer, kids)

    # --- Device components (EasyEDA LIB, official-style female/male + module bodies) ---
    pitch = 2.54
    k902_xl, k902_xr, k902_gnd_y = 37.5, 82.5, 53.0
    k902_left = [
        (str(i), k902_xl, k902_gnd_y + (9 - i) * pitch, net)
        for i, net in enumerate(
            ["", "", "", "", "UART2_TX", "UART2_RX", "", "5V", "GND"], start=1
        )
    ]
    k902_right = [
        (str(i), k902_xr, k902_gnd_y + (10 - i) * pitch, "GND" if i == 10 else "")
        for i in range(1, 11)
    ]
    esp = (100.0, 60.0)
    y0 = esp[1] - 17.78
    lx, rx = esp[0] - 12.7, esp[0] + 12.7
    left_nets = {5: "UART2_RX", 6: "UART2_TX", 0: "3V3", 1: "GND"}
    right_nets = {0: "VIN", 1: "GND", 5: "UART1_TX", 6: "UART1_RX", 7: "HC12_SET"}
    esp_l = [(str(i + 1), lx, y0 + i * pitch, left_nets.get(i, "")) for i in range(15)]
    esp_r = [(str(i + 1), rx, y0 + i * pitch, right_nets.get(i, "")) for i in range(15)]
    hc = (48.0, 26.0)
    hc_pins = [
        (str(i), hc[0], hc[1] + (i - 3) * pitch, net)
        for i, net in enumerate(["5V", "GND", "UART1_TX", "UART1_RX_HC", "HC12_SET"], start=1)
    ]
    jp = (rx - 6.0, y0)
    jp_pins = [("1", jp[0] - 1.27, jp[1], "5V"), ("2", jp[0] + 1.27, jp[1], "VIN")]
    j5_pins = [("1", 9.0, 46.0, "5V"), ("2", 7.0, 46.0, "GND")]
    j8_pins = [("1", 9.0, 70.0, "PSU_K"), ("2", 7.0, 70.0, "GND")]
    j7_pins = [
        ("1", 26.0, 61.0, "5V"),
        ("2", 26.0, 59.0, "UART2_TX"),
        ("3", 26.0, 57.0, "UART2_RX"),
        ("4", 26.0, 55.0, ""),
        ("5", 26.0, 53.0, "GND"),
        ("6", 26.0, 51.0, ""),
    ]

    shapes.append(module_device("M1", "K902-48x42", "K902_48x42", (36, 39, 84, 81), "1",
                                [(39, 42), (81, 42), (39, 78), (81, 78)]))
    shapes.append(header_device("J2", "HDR-F-1x9", "HDR-TH_9P-P2.54-V-F", k902_left, "1", MODEL_F10, "0,0,90", "-12.6"))
    shapes.append(header_device("J6", "HDR-F-1x10", "HDR-TH_10P-P2.54-V-F", k902_right, "1", MODEL_F10, "0,0,90", "-12.6"))
    shapes.append(header_device("U1L", "HDR-F-1x15", "HDR-TH_15P-P2.54-V-F-1", esp_l, "1", MODEL_F15, "0,0,90", "-12.6"))
    shapes.append(header_device("U1R", "HDR-F-1x15", "HDR-TH_15P-P2.54-V-F-1", esp_r, "1", MODEL_F15, "0,0,90", "-12.6"))
    shapes.append(module_device("U1", "ESP32-DevKit-30", "ESP32_DevKit_30", (86.8, 41.5, 113.2, 78.5), "1"))
    shapes.append(header_device("JP1", "HDR-M-1x2", "HDR-TH_2P-P2.54-V", jp_pins, "1", MODEL_M6, "0,0,0", "-11.8"))
    shapes.append(header_device("J7", "JST-PH-6P", "CONN-TH_B6B-PH-K-S", j7_pins, "1", MODEL_M6, "0,0,90", "-13.8"))
    shapes.append(header_device("J3", "HC-12-1x5", "HDR-TH_5P-P2.54-V", hc_pins, "2", MODEL_M6, "180,0,90", "-11.8"))
    shapes.append(module_device("U2", "HC-12", "HC12_27x14", (49.5, 18.8, 77.3, 33.2), "2"))
    shapes.append(module_device(
        "U3", "USB-C-5V", "USBC_5V_25x20", (16, 78, 41, 98), "2",
        extra_pads=[("+", 32.5, 80.0, "5V"), ("-", 24.5, 80.0, "GND"), ("K", 28.5, 80.0, "PSU_K")],
    ))
    shapes.append(header_device("J5", "JST-PH-2P", "CONN-TH_B2B-PH-K-S", j5_pins, "2", MODEL_PH2, "180,0,0", "-13.8"))
    shapes.append(header_device("J8", "SW3-PWR-PH2", "CONN-TH_B2B-PH-K-S", j8_pins, "2", MODEL_PH2, "180,0,0", "-13.8"))
    shapes.append(module_device(
        "R10", "1k", "R_Axial_P7.62", (106, 48.5, 110, 51.5), "2",
        extra_pads=[("1", 110.0, 50.0, "UART1_RX"), ("2", 106.0, 50.0, "UART1_RX_HC")],
    ))
    shapes.append(module_device(
        "C6", "220u", "CP_Radial_D5", (39.5, 15.5, 44.5, 20.5), "2",
        extra_pads=[("1", 42.0, 20.0, "5V"), ("2", 42.0, 16.0, "GND")],
    ))

    shapes.append(text_label("K902 48x42", 38, 83, "3"))
    shapes.append(text_label("ESP32", 90, 40, "3"))
    shapes.append(text_label("J7 JST-PH6", 18, 63, "3"))
    shapes.append(text_label("J8 K-GND", 10, 66, "3"))
    shapes.append(text_label("SW3 PWR SW", 10, 73.6, "3"))
    shapes.append(text_label("HC-12", 50, 34.5, "4"))
    shapes.append(text_label("USB-C 5V", 17, 99, "4"))
    shapes.append(text_label("J5 BAT", 10, 49, "4"))
    shapes.append(text_label("SW3 PWR SW", 10, 75, "4"))
    shapes.append(text_label("K", 29.2, 77.5, "4"))
    shapes.append(
        f"TRACK~0.8~3~~{to_e(57,60)[0]:.4f} {to_e(57,60)[1]:.4f} {to_e(63,60)[0]:.4f} {to_e(63,60)[1]:.4f}~{nid()}~0"
    )
    shapes.append(
        f"TRACK~0.8~3~~{to_e(60,57)[0]:.4f} {to_e(60,57)[1]:.4f} {to_e(60,63)[0]:.4f} {to_e(60,63)[1]:.4f}~{nid()}~0"
    )

    layer_of = {"F.Cu": "1", "B.Cu": "2"}
    for x0, y0, x1, y1, w, ly, neti in segs:
        xa, ya = to_e(x0, y0)
        xb, yb = to_e(x1, y1)
        net = nets.get(neti, "")
        shapes.append(
            f"TRACK~{u(w):.4f}~{layer_of[ly]}~{net}~{xa:.4f} {ya:.4f} {xb:.4f} {yb:.4f}~{nid()}~0"
        )
    for x, y, sz, dr, neti in vias:
        mx, my = to_e(x, y)
        shapes.append(
            f"VIA~{mx:.4f}~{my:.4f}~{u(sz):.4f}~{nets.get(neti,'')}~{u(dr/2):.4f}~{nid()}~0"
        )
    for x, y, dr in holes:
        mx, my = to_e(x, y)
        shapes.append(f"HOLE~{mx:.4f}~{my:.4f}~{u(dr / 2):.4f}~{nid()}~0")

    rbox = u(62)
    doc = {
        "head": {
            "docType": "3",
            "editorVersion": "6.5.51",
            "newgId": True,
            "c_para": {"title": "gnss-head", "Contributor": "ESP32_Rover_NEZ"},
            "x": str(int(OX)),
            "y": str(int(OY)),
            "hasIdFlag": False,
        },
        "canvas": (
            f"CA~1000~1000~#000000~yes~#FFFFFF~10~1000~1000~line~0.5~mm~1~45~"
            f"visible~0.5~{int(OX)}~{int(OY)}"
        ),
        "shape": shapes,
        "layers": LAYERS,
        "objects": OBJECTS,
        "BBox": {"x": OX - rbox, "y": OY - rbox, "width": rbox * 2, "height": rbox * 2},
        "preference": {"hideFootprints": "", "hideNets": ""},
        "DRCRULE": {
            "Default": {"trackWidth": 1, "clearance": 0.6, "viaHoleDiameter": 2.4, "viaHoleD": 1.2},
            "isRealtime": False,
            "checkObjectToCopperarea": True,
            "showDRCRangeLine": True,
        },
        "routerRule": {},
        "netColors": {},
    }
    return json.dumps(doc, ensure_ascii=False)


def write_easyeda_pro(pads, segs, vias, holes, nets) -> tuple[str, str]:
    """EasyEDA Pro v2 archive: project.json + PCB/{uuid} JSON Lines."""
    pcb_uuid = "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6"
    n = 0

    def eid() -> str:
        nonlocal n
        n += 1
        return f"e{n}"

    def px(x: float, y: float) -> tuple[float, float]:
        return round(x, 4), round(DIA - y, 4)

    def rec(obj: list) -> str:
        return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))

    lines: list[str] = [
        rec(["DOCTYPE", "PCB", "1.6"]),
        rec(["HEAD", {"editorVersion": "2.2.48.10", "importFlag": 0}]),
        rec(["CANVAS", 0, 0, "mm", 5, 5, 1, 1, 0.1, 0.1, 1]),
    ]
    layer_defs = [
        (0, "TOP", "Top Layer", "#FF0000", "#880000"),
        (2, "BOTTOM", "Bottom Layer", "#0000FF", "#00007F"),
        (3, "TOP_SILK", "Top Silkscreen Layer", "#FFCC00", "#7F6600"),
        (4, "BOT_SILK", "Bottom Silkscreen Layer", "#66CC33", "#336619"),
        (5, "TOP_SOLDER_MASK", "Top Solder Mask Layer", "#800080", "#400040"),
        (6, "BOT_SOLDER_MASK", "Bottom Solder Mask Layer", "#AA00FF", "#55007F"),
        (7, "TOP_PASTE_MASK", "Top Paste Mask Layer", "#808080", "#404040"),
        (8, "BOT_PASTE_MASK", "Bottom Paste Mask Layer", "#800000", "#400000"),
        (11, "OUTLINE", "Board Outline Layer", "#FF00FF", "#7F007F"),
        (12, "MULTI", "Multi-Layer", "#C0C0C0", "#606060"),
        (13, "DOCUMENT", "Document Layer", "#FFFFFF", "#7F7F7F"),
    ]
    for num, typ, alias, ac, ic in layer_defs:
        lines.append(rec(["LAYER", num, typ, alias, 3, ac, 1, ic, 1]))
    lines.append(rec(["LAYER_PHYS", 0, "COPPER", 0.035, 0, 0, 0]))
    lines.append(rec(["LAYER_PHYS", 2, "COPPER", 0.035, 0, 0, 0]))

    for name in ("GND", "5V", "UART2_RX", "UART2_TX", "UART1_RX", "UART1_TX", "UART1_RX_HC", "HC12_SET", "VIN", "3V3", "PSU_K"):
        lines.append(rec(["NET", name, None, None, 0, None, 0, None]))

    # Circular outline as two 180° arcs
    oa, ob = px(CX + 60, CY)
    oc, od = px(CX - 60, CY)
    lines.append(rec(["ARC", eid(), 0, "", 11, oa, ob, oc, od, 180, 0.15, 0]))
    lines.append(rec(["ARC", eid(), 0, "", 11, oc, od, oa, ob, 180, 0.15, 0]))

    def box(x0: float, y0: float, x1: float, y1: float, layer: int) -> None:
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        for (a, b), (c, d) in zip(pts, pts[1:]):
            xa, ya = px(a, b)
            xb, yb = px(c, d)
            lines.append(rec(["LINE", eid(), 0, "", layer, xa, ya, xb, yb, 0.15, 0]))

    box(36, 39, 84, 81, 3)
    box(86.8, 41.5, 113.2, 78.5, 3)
    box(49.5, 18.8, 77.3, 33.2, 4)
    box(16, 78, 41, 98, 4)
    box(6.0, 68.5, 10.0, 71.5, 4)
    box(49.5, 18.8, 77.3, 33.2, 3)

    def label(txt: str, x: float, y: float, layer: int, mirror: int = 0) -> None:
        xa, ya = px(x, y)
        lines.append(rec(["STRING", eid(), 0, layer, xa, ya, txt, "Arial", 1.2, 0.15, 0, 0, 6, 0, 0, 0, mirror, 0]))

    label("K902 48x42", 38, 83, 3)
    label("ESP32", 90, 40, 3)
    label("HC-12", 50, 34.5, 4, 1)
    label("USB-C 5V", 17, 99, 4, 1)
    label("J5 BAT", 10, 49, 4, 1)
    label("SW3 PWR SW", 10, 75, 4, 1)
    label("J8 K-GND", 10, 66, 3)
    label("K", 29.2, 77.5, 4, 1)
    label("J7 JST-PH6", 18, 63, 3)

    layer_of = {"F.Cu": 0, "B.Cu": 2}
    for x0, y0, x1, y1, w, ly, neti in segs:
        xa, ya = px(x0, y0)
        xb, yb = px(x1, y1)
        net = nets.get(neti, "")
        lines.append(rec(["LINE", eid(), 0, net, layer_of[ly], xa, ya, xb, yb, round(w, 3), 0]))

    for x, y, sz, dr, neti in vias:
        xa, ya = px(x, y)
        net = nets.get(neti, "")
        lines.append(rec(["VIA", eid(), 0, net, "", xa, ya, round(dr, 3), round(sz, 3), 0, None, None, None, None, 0]))

    for i, (x, y, sz, dr, net, name) in enumerate(pads, start=1):
        xa, ya = px(x, y)
        lines.append(
            rec(
                [
                    "PAD",
                    eid(),
                    0,
                    net,
                    12,
                    str(name or i),
                    xa,
                    ya,
                    0,
                    ["ROUND", round(dr, 3), round(dr, 3)],
                    ["ROUND", round(sz, 3), round(sz, 3)],
                    [],
                    0,
                    0,
                    0,
                    1,
                    0,
                    None,
                    None,
                    None,
                    None,
                    0,
                ]
            )
        )

    for x, y, dr in holes:
        xa, ya = px(x, y)
        lines.append(
            rec(
                [
                    "PAD",
                    eid(),
                    0,
                    "",
                    12,
                    "",
                    xa,
                    ya,
                    0,
                    ["ROUND", round(dr, 3), round(dr, 3)],
                    ["ROUND", round(dr, 3), round(dr, 3)],
                    [],
                    0,
                    0,
                    0,
                    0,
                    0,
                    None,
                    None,
                    None,
                    None,
                    0,
                ]
            )
        )

    project = {
        "schematics": {},
        "pcbs": {pcb_uuid: "gnss-head"},
        "boards": {"Board1": {"pcb": pcb_uuid}},
        "symbols": {},
        "footprints": {},
        "devices": {},
        "config": {
            "title": "gnss-head",
            "cbbProject": False,
            "editorVersion": "2.2.48.10",
        },
    }
    return json.dumps(project, ensure_ascii=False, indent=2), "\n".join(lines) + "\n"


def main() -> None:
    text = PCB.read_text(encoding="utf-8")
    pads, segs, vias, holes, nets = parse_pcb(text)
    OUT.mkdir(exist_ok=True)
    gdir = OUT / "gerber"
    write_gerbers(pads, segs, vias, holes, gdir)
    js = write_easyeda_std(pads, segs, vias, holes, nets)
    if "COPPERAREA" in js:
        raise SystemExit("refusing COPPERAREA in Standard JSON (hides traces)")
    doc = json.loads(js)
    nshape = len(doc.get("shape") or [])
    if nshape < 20 or len(js) < 8000:
        raise SystemExit(f"Standard JSON empty/too small: bytes={len(js)} shapes={nshape}")
    for token in ("LIB~", "PAD~", "TRACK~", "J8", "PSU_K", "J5", "J7"):
        if token not in js:
            raise SystemExit(f"Standard JSON missing {token}")
    (OUT / "gnss-head.easyeda.json").write_text(js, encoding="utf-8")
    (OUT / "gnss-head.json").write_text(js, encoding="utf-8")
    z_std = OUT / "gnss-head-easyeda-std.zip"
    with zipfile.ZipFile(z_std, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("gnss-head.json", js)

    proj, epcb = write_easyeda_pro(pads, segs, vias, holes, nets)
    pcb_uuid = "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6"
    (OUT / "gnss-head-pro-project.json").write_text(proj, encoding="utf-8")
    z_pro = OUT / "gnss-head.epro"
    with zipfile.ZipFile(z_pro, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("project.json", proj)
        z.writestr(f"PCB/{pcb_uuid}", epcb)
        z.writestr(f"PCB/{pcb_uuid}.epcb", epcb)
    z_pro_zip = OUT / "gnss-head-easyeda-pro.zip"
    with zipfile.ZipFile(z_pro_zip, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("project.json", proj)
        z.writestr(f"PCB/{pcb_uuid}", epcb)
        z.writestr(f"PCB/{pcb_uuid}.epcb", epcb)

    z_kicad = OUT / "gnss-head-kicad-for-easyeda-pro.zip"
    with zipfile.ZipFile(z_kicad, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(PCB, PCB.name)
        if PRO.exists():
            z.write(PRO, PRO.name)
        if SCH.exists():
            z.write(SCH, SCH.name)

    z_gbr = OUT / "gnss-head-gerber.zip"
    with zipfile.ZipFile(z_gbr, "w", zipfile.ZIP_DEFLATED) as z:
        for p in gdir.iterdir():
            z.write(p, p.name)

    jpath = OUT / "gnss-head.json"
    print(f"pads={len(pads)} segs={len(segs)} vias={len(vias)} holes={len(holes)}")
    print(f"std json bytes={jpath.stat().st_size} shapes={nshape} svg3d={js.count('SVGNODE')}")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()

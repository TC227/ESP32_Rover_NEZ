"""Generate gnss-head.kicad_pcb — circular 120 mm breakout carrier.

Top (F.Cu): K9022 female 1x9+1x10 + ESP32 female 2x15 + JP1.
Bottom (B.Cu): HC-12 + USB-C 5V PSU (+/−/K) + BAT JST-PH 2.0 + J8 SW3 PWR jack + R10 + C6.
SW3 is an external momentary switch on a cable into J8 (K to GND). Not soldered on the carrier.

Origin: bounding-box lower-left. Circle center (60, 60) mm. Y up.
"""
from __future__ import annotations

import math
import re
import uuid
from pathlib import Path

OUT = Path(__file__).with_name("gnss-head.kicad_pcb")

CX, CY = 60.0, 60.0
R = 60.0
BOLT_R = 56.0  # 112 mm BCD, ~4 mm in from rim


def uid() -> str:
    return str(uuid.uuid4())


def net_decl(n: int, name: str) -> str:
    return f'\t(net {n} "{name}")\n'


def pad(name: str, x: float, y: float, net: int, net_name: str, drill: float = 1.0, size: float = 1.7) -> str:
    return f"""\t\t(pad "{name}" thru_hole circle
\t\t\t(at {x:.3f} {y:.3f})
\t\t\t(size {size} {size})
\t\t\t(drill {drill})
\t\t\t(layers "*.Cu" "*.Mask")
\t\t\t(remove_unused_layers no)
\t\t\t(net {net} "{net_name}")
\t\t\t(uuid "{uid()}")
\t\t)
"""


def npth(name: str, x: float, y: float, drill: float = 3.2) -> str:
    return f"""\t\t(pad "{name}" npth circle
\t\t\t(at {x:.3f} {y:.3f})
\t\t\t(size {drill} {drill})
\t\t\t(drill {drill})
\t\t\t(layers "F.Cu" "B.Cu")
\t\t\t(remove_unused_layers no)
\t\t\t(uuid "{uid()}")
\t\t)
"""


def silk_rect(x0: float, y0: float, x1: float, y1: float, layer: str = "F.SilkS") -> str:
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    lines = []
    for (a, b), (c, d) in zip(pts, pts[1:]):
        lines.append(
            f'\t(gr_line (start {a:.3f} {b:.3f}) (end {c:.3f} {d:.3f}) '
            f'(stroke (width 0.15) (type default)) (layer "{layer}") (uuid "{uid()}"))\n'
        )
    return "".join(lines)


def silk_text(txt: str, x: float, y: float, size: float = 1.2, layer: str = "F.SilkS", mirror: bool = False) -> str:
    just = "left bottom mirror" if mirror else "left bottom"
    return (
        f'\t(gr_text "{txt}" (at {x:.3f} {y:.3f} 0) (layer "{layer}") (uuid "{uid()}")\n'
        f'\t\t(effects (font (size {size} {size}) (thickness {size * 0.15})) (justify {just}))\n'
        f'\t)\n'
    )


def keepout(x0: float, y0: float, x1: float, y1: float, name: str) -> str:
    return f"""\t(rule_area
\t\t(name "{name}")
\t\t(layers "F.Cu" "B.Cu")
\t\t(uuid "{uid()}")
\t\t(keepout (tracks not_allowed) (vias not_allowed) (pads not_allowed) (copperpour not_allowed) (footprints allowed))
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy {x0:.3f} {y0:.3f})
\t\t\t\t(xy {x1:.3f} {y0:.3f})
\t\t\t\t(xy {x1:.3f} {y1:.3f})
\t\t\t\t(xy {x0:.3f} {y1:.3f})
\t\t\t)
\t\t)
\t)
"""


def segment(x0: float, y0: float, x1: float, y1: float, net: int, width: float = 0.4, layer: str = "F.Cu") -> str:
    return (
        f'\t(segment (start {x0:.3f} {y0:.3f}) (end {x1:.3f} {y1:.3f}) '
        f'(width {width}) (layer "{layer}") (net {net}) (uuid "{uid()}"))\n'
    )


def via(x: float, y: float, net: int) -> str:
    return (
        f'\t(via (at {x:.3f} {y:.3f}) (size 0.8) (drill 0.4) '
        f'(layers "F.Cu" "B.Cu") (net {net}) (uuid "{uid()}"))\n'
    )


def dist(x: float, y: float) -> float:
    return math.hypot(x - CX, y - CY)


def fp_header(name: str, layer: str, at: tuple[float, float], ref: str, value: str, descr: str = "") -> str:
    dx, dy = at
    d = f'\t\t(descr "{descr}")\n' if descr else ""
    return f"""\t(footprint "{name}"
\t\t(layer "{layer}")
\t\t(uuid "{uid()}")
\t\t(at {dx:.3f} {dy:.3f} 0)
{d}\t\t(property "Reference" "{ref}" (at 0 3.5 0) (layer "{layer.replace('.Cu', '.SilkS')}") (uuid "{uid()}")
\t\t\t(effects (font (size 1.0 1.0) (thickness 0.15)){" (justify mirror)" if layer.startswith("B.") else ""}))
\t\t(property "Value" "{value}" (at 0 -3.5 0) (layer "{layer.replace('.Cu', '.SilkS')}") (uuid "{uid()}")
\t\t\t(effects (font (size 0.9 0.9) (thickness 0.12)){" (justify mirror)" if layer.startswith("B.") else ""}))
\t\t(attr through_hole)
"""


NETS = [
    (0, ""),
    (1, "GND"),
    (2, "5V"),
    (3, "UART2_RX"),
    (4, "UART2_TX"),
    (5, "UART1_RX"),
    (6, "UART1_TX"),
    (7, "HC12_SET"),
    (8, "3V3"),
    (9, "VIN"),
    (10, "UART1_RX_HC"),
    (11, "PSU_K"),
    (12, "BUZZER"),
    (13, "I2C_SDA"),
    (14, "I2C_SCL"),
]

# --- Placement ---
# URTK1.2 / K902 body 38×38 mm, centered. SMA / pin1 toward +Y.
# Pin rows 2.0 mm in from the left and right edges → 34 mm between rows.
K902_W, K902_H = 38.0, 38.0
K902 = (CX - K902_W / 2, CY - K902_H / 2, CX + K902_W / 2, CY + K902_H / 2)  # (41,41)-(79,79)
assert abs((K902[0] + K902[2]) / 2 - CX) < 1e-9
assert abs((K902[1] + K902[3]) / 2 - CY) < 1e-9
ESP = (100.0, 60.0)  # right of K902, USB toward -Y, WROOM +Y
# K9022 photo: female sockets receive male headers from the module bottom.
# Left 1x9 (SCK..GND), right 1x10 (INT..GND), 2.54 mm, GND aligned, pin1 toward SMA (+Y).
K902_PITCH = 2.54
K902_XL, K902_XR = CX - 17.0, CX + 17.0  # 43 and 77; row spacing 34 mm
K902_GND_Y = CY - 4.5 * K902_PITCH  # right column is 10 pins; GND is the −Y end
HC_PINS = (48.0, 26.0)  # back, body +X under K902, ANT +X
PSU = (16.0, 78.0, 41.0, 98.0)  # top-left, USB-C toward +Y rim
J5 = (8.0, 46.0)  # BAT PH2.0 on back, left rim, cable out −X
J8 = (8.0, 70.0)  # SW3 PWR jack: PH2.0 male on back, left rim above J5, cable out −X
J7 = (26.0, 56.0)  # spare JST-PH 6P male on front, left of K902
R10_AT = (108.0, 50.0)
C6_AT = (42.0, 18.0)  # west of HC-12 pins, next to VCC

LX, RX = ESP[0] - 12.7, ESP[0] + 12.7
Y0 = ESP[1] - 17.78


def py(i: int) -> float:
    return Y0 + i * 2.54


gpio16 = (LX, py(5))
gpio17 = (LX, py(6))
gpio27 = (RX, py(5))
gpio26 = (RX, py(6))
gpio25 = (RX, py(7))
esp_vin = (RX, py(0))
esp_gnd_r = (RX, py(1))
esp_gnd_l = (LX, py(1))

def k902_left_y(i: int) -> float:
    """Left 1x9, pin1 SCK at +Y, pin9 GND."""
    return K902_GND_Y + (9 - i) * K902_PITCH


def k902_right_y(i: int) -> float:
    """Right 1x10, pin1 INT at +Y, pin10 GND."""
    return K902_GND_Y + (10 - i) * K902_PITCH


k902_rx2 = (K902_XL, k902_left_y(5))
k902_tx2 = (K902_XL, k902_left_y(6))
k902_5v = (K902_XL, k902_left_y(8))
k902_gnd_l = (K902_XL, k902_left_y(9))
k902_gnd_r = (K902_XR, k902_right_y(10))


def j7y(i: int) -> float:
    """J7 PH 6P, pin1 5V at +Y."""
    return J7[1] + (3.5 - i) * 2.0


j7_5v = (J7[0], j7y(1))
j7_tx = (J7[0], j7y(2))  # UART2_TX → module RX3
j7_rx = (J7[0], j7y(3))  # UART2_RX ← module TX3
j7_gnd = (J7[0], j7y(5))


def hcy(i: int) -> float:
    return HC_PINS[1] + (i - 3) * 2.54


hc_vcc = (HC_PINS[0], hcy(1))
hc_gnd = (HC_PINS[0], hcy(2))
hc_rxd = (HC_PINS[0], hcy(3))
hc_txd = (HC_PINS[0], hcy(4))
hc_set = (HC_PINS[0], hcy(5))

# PSU +/−/K inner (−Y) edge. USB-C toward +Y. Board coords after B.Cu X-mirror.
# K sits between + and −. Do not tie K to 5V.
psu_p = (32.5, 80.0)
psu_n = (24.5, 80.0)
psu_k = (28.5, 80.0)

# J5 on B.Cu: KiCad mirrors X, pad local -x lands at at_x+x.
# Define pads as front-view; record actual board coords after mirror.
j5_local_p = -1.00  # pin1 BAT+ local (front left)
j5_local_n = 1.00
j5_p = (J5[0] - j5_local_p, J5[1])  # board after B.Cu mirror: at - local
j5_n = (J5[0] - j5_local_n, J5[1])

j8_local_k = -1.00  # pin1 K
j8_local_g = 1.00
j8_k = (J8[0] - j8_local_k, J8[1])  # (9.0, 70.0)
j8_g = (J8[0] - j8_local_g, J8[1])  # (7.0, 70.0)

# R10 on B.Cu, front-view pad1 left (-2), pad2 right (+2)
r10_a = (R10_AT[0] - (-2.0), R10_AT[1])  # UART1_RX board
r10_b = (R10_AT[0] - (2.0), R10_AT[1])  # UART1_RX_HC board

# C6 on B.Cu, pads at local (0, ±2) — X=0, no mirror shift
c6_p = (C6_AT[0], C6_AT[1] + 2.0)
c6_n = (C6_AT[0], C6_AT[1] - 2.0)

hc_body = (HC_PINS[0] + 1.5, HC_PINS[1] - 7.2, HC_PINS[0] + 1.5 + 27.8, HC_PINS[1] + 7.2)

tht: list[tuple[str, float, float]] = []
parts: list[str] = []

# ----- TOP: ESP32 -----
left_names = ["3V3", "GND", "15", "2", "4", "16", "17", "5", "18", "19", "21", "RX0", "TX0", "22", "23"]
right_names = ["VIN", "GND2", "13", "12", "14", "27", "26", "25", "33", "32", "35", "34", "VN", "VP", "EN"]
left_nets = {"3V3": (8, "3V3"), "GND": (1, "GND"), "16": (3, "UART2_RX"), "17": (4, "UART2_TX"), "23": (12, "BUZZER"), "21": (13, "I2C_SDA"), "22": (14, "I2C_SCL")}
right_nets = {"VIN": (9, "VIN"), "GND2": (1, "GND"), "27": (6, "UART1_TX"), "26": (5, "UART1_RX"), "25": (7, "HC12_SET")}

esp_pads = []
for i, name in enumerate(left_names):
    n, nn = left_nets.get(name, (0, ""))
    esp_pads.append(pad(name, -12.7, -17.78 + i * 2.54, n, nn))
    tht.append((f"U1.{name}", LX, py(i)))
for i, name in enumerate(right_names):
    n, nn = right_nets.get(name, (0, ""))
    esp_pads.append(pad(name, 12.7, -17.78 + i * 2.54, n, nn))
    tht.append((f"U1.{name}", RX, py(i)))

parts.append(
    fp_header(
        "Carrier:ESP32_DevKit_FEMALE_2x15",
        "F.Cu",
        ESP,
        "U1",
        "ESP32-FEMALE-2x15",
        "Female 2x15 socket. DevKit male pins plug in. USB -Y",
    )
    + "".join(esp_pads)
    + "\t)\n"
)

# K902 female sockets — module male headers plug down from the bottom
k902_left_names = ["SCK", "TX_MISO", "RX_MOSI", "CS", "RX2", "TX2", "3V3", "5V", "GND"]
k902_left_nets = {
    "RX2": (4, "UART2_TX"),
    "TX2": (3, "UART2_RX"),
    "5V": (2, "5V"),
    "GND": (1, "GND"),
}
k902_l_pads = []
for i, name in enumerate(k902_left_names, start=1):
    n, nn = k902_left_nets.get(name, (0, ""))
    k902_l_pads.append(pad(str(i), 0, (9 - i) * K902_PITCH, n, nn))
    tht.append((f"J2.{i}", K902_XL, k902_left_y(i)))
parts.append(
    fp_header(
        "Connector_PinSocket_2.54mm:PinSocket_1x09_P2.54mm_Vertical",
        "F.Cu",
        (K902_XL, K902_GND_Y),
        "J2",
        "K902-L-FEMALE",
        "Female 1x9. Pin1 SCK +Y .. Pin9 GND. RX2/TX2 = COM2",
    )
    + "".join(k902_l_pads)
    + "\t)\n"
)

k902_right_names = ["INT", "SAFE", "RST", "PPS", "RTK", "FENCE", "SCL", "SDA", "3V3", "GND"]
k902_r_pads = []
for i, name in enumerate(k902_right_names, start=1):
    n, nn = (1, "GND") if name == "GND" else (0, "")
    k902_r_pads.append(pad(str(i), 0, (10 - i) * K902_PITCH, n, nn))
    tht.append((f"J6.{i}", K902_XR, k902_right_y(i)))
parts.append(
    fp_header(
        "Connector_PinSocket_2.54mm:PinSocket_1x10_P2.54mm_Vertical",
        "F.Cu",
        (K902_XR, K902_GND_Y),
        "J6",
        "K902-R-FEMALE",
        "Female 1x10. Pin1 INT +Y .. Pin10 GND",
    )
    + "".join(k902_r_pads)
    + "\t)\n"
)

jp_at = (esp_vin[0] - 6.0, esp_vin[1])  # inside the disk, left of VIN
jp_pads = [pad("1", -1.27, 0, 2, "5V"), pad("2", 1.27, 0, 9, "VIN")]
parts.append(
    fp_header("Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical", "F.Cu", jp_at, "JP1", "5V-VIN")
    + "".join(jp_pads)
    + "\t)\n"
)
tht += [("JP1.1", jp_at[0] - 1.27, jp_at[1]), ("JP1.2", jp_at[0] + 1.27, jp_at[1])]

# Spare JST-PH 6P male — cable from K902 J1 can plug here instead of stacking
j7_pads = [
    pad("1", 0, (3.5 - 1) * 2.0, 2, "5V", 0.9, 1.6),
    pad("2", 0, (3.5 - 2) * 2.0, 4, "UART2_TX", 0.9, 1.6),
    pad("3", 0, (3.5 - 3) * 2.0, 3, "UART2_RX", 0.9, 1.6),
    pad("4", 0, (3.5 - 4) * 2.0, 0, "", 0.9, 1.6),
    pad("5", 0, (3.5 - 5) * 2.0, 1, "GND", 0.9, 1.6),
    pad("6", 0, (3.5 - 6) * 2.0, 0, "", 0.9, 1.6),
]
parts.append(
    fp_header(
        "Connector_JST:JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical",
        "F.Cu",
        J7,
        "J7",
        "JST-PH-6P",
        "Spare male. Pin1 5V +Y, 2 TX, 3 RX, 4 EVT, 5 GND, 6 NC. Cable from K902 J1",
    )
    + "".join(j7_pads)
    + "\t)\n"
)
tht += [("J7.1", *j7_5v), ("J7.2", *j7_tx), ("J7.3", *j7_rx), ("J7.4", J7[0], j7y(4)), ("J7.5", *j7_gnd), ("J7.6", J7[0], j7y(6))]

# Active buzzer module, front, lower right. GPIO23 high = beep. Not under the antennas.
BZ = (98.0, 28.0)
bz_vcc = (BZ[0], BZ[1] + 2.54)
bz_io = (BZ[0], BZ[1])
bz_gnd = (BZ[0], BZ[1] - 2.54)
bz_pads = [
    pad("VCC", 0, 2.54, 2, "5V", 0.9, 1.6),
    pad("IO", 0, 0, 12, "BUZZER", 0.9, 1.6),
    pad("GND", 0, -2.54, 1, "GND", 0.9, 1.6),
]
parts.append(
    fp_header("Carrier:BUZZER_3P", "F.Cu", BZ, "BZ1", "BUZZER-ACTIVE", "GPIO23 high = beep. VCC=5V. Lower right, clear of antennas")
    + "".join(bz_pads)
    + "\t)\n"
)
tht += [("BZ1.VCC", *bz_vcc), ("BZ1.IO", *bz_io), ("BZ1.GND", *bz_gnd)]

# GY-273 magnetometer (HMC5883L/QMC5883L) — female 1x5 socket, top side, bottom-left.
# Far from K902 GNSS antenna (top), ESP32 antenna (top-right), HC-12 (bottom-center).
# Pin order assumed: 1=VCC 2=GND 3=SCL 4=SDA 5=DRDY (top→bottom) — VERIFY module silkscreen.
GY = (20.0, 30.0)
gy_pads = [
    pad("1", 0, (1 - 3) * 2.54, 8, "3V3"),   # VCC — module has own LDO, 3V3 ok
    pad("2", 0, (2 - 3) * 2.54, 1, "GND"),
    pad("3", 0, (3 - 3) * 2.54, 14, "I2C_SCL"),
    pad("4", 0, (4 - 3) * 2.54, 13, "I2C_SDA"),
    pad("5", 0, (5 - 3) * 2.54, 0, ""),       # DRDY — NC (polling)
]
parts.append(
    fp_header("Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical", "F.Cu", GY, "J4", "GY-273", "Magnetometer. Bottom-left, clear of antennas. Verify pin order vs silkscreen")
    + "".join(gy_pads)
    + "\t)\n"
)
for i in range(1, 6):
    tht.append((f"J4.{i}", GY[0], GY[1] + (i - 3) * 2.54))

# K902 mounting holes 30×30 mm, Ø2.2, centered on the 38×38 body
k_holes = [(CX + sx * 15.0, CY + sy * 15.0) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]
kh = [npth(f"MH{i+1}", x - CX, y - CY, 2.2) for i, (x, y) in enumerate(k_holes)]
parts.append(
    fp_header("Carrier:K902_38x38", "F.Cu", (CX, CY), "M1", "K902-38x38", "CENTERED 38x38  SMA +Y  pins 34 mm apart")
    + "".join(kh)
    + "\t)\n"
)
for i, (x, y) in enumerate(k_holes):
    tht.append((f"M1.MH{i+1}", x, y))

# ----- BOTTOM: HC-12 (pads X=0, B.Cu mirror-safe) -----
hc_pads = [
    pad("1", 0, (1 - 3) * 2.54, 2, "5V"),
    pad("2", 0, (2 - 3) * 2.54, 1, "GND"),
    pad("3", 0, (3 - 3) * 2.54, 6, "UART1_TX"),
    pad("4", 0, (4 - 3) * 2.54, 10, "UART1_RX_HC"),
    pad("5", 0, (5 - 3) * 2.54, 7, "HC12_SET"),
]
parts.append(
    fp_header("Carrier:HC12_1x05", "B.Cu", HC_PINS, "J3", "HC-12", "Bottom. Pin1 VCC .. Pin5 SET. Body +X ANT +X")
    + "".join(hc_pads)
    + "\t)\n"
)
tht += [("J3.1", *hc_vcc), ("J3.2", *hc_gnd), ("J3.3", *hc_rxd), ("J3.4", *hc_txd), ("J3.5", *hc_set)]

# PSU pads along inner edge. B.Cu mirrors X: board = at_x - local_x.
psu_anchor = (28.5, 80.0)
psu_fp_pads = [
    pad("+", -4.0, 0, 2, "5V", 1.2, 2.0),
    pad("-", 4.0, 0, 1, "GND", 1.2, 2.0),
    pad("K", 0.0, 0, 11, "PSU_K", 0.9, 1.6),
]
parts.append(
    fp_header("Carrier:USBC_5V_25x20", "B.Cu", psu_anchor, "U3", "USB-C-5V-25x20", "Top-left. USB-C toward +Y rim. Pads + − K")
    + "".join(psu_fp_pads)
    + "\t)\n"
)
tht += [("U3.+", *psu_p), ("U3.-", *psu_n), ("U3.K", *psu_k)]

# J5 BAT JST-PH 2.00 mm (NOT XH 2.54)
j5_pads = [
    pad("1", j5_local_p, 0, 2, "5V", 0.9, 1.6),
    pad("2", j5_local_n, 0, 1, "GND", 0.9, 1.6),
]
parts.append(
    fp_header("Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", "B.Cu", J5, "J5", "BAT-PH2.0", "Left rim. Pin1 BAT+ Pin2 GND. Cable exits -X")
    + "".join(j5_pads)
    + "\t)\n"
)
tht += [("J5.1", *j5_p), ("J5.2", *j5_n)]

# J8 = SW3 PWR jack. External momentary switch cable. Pin1 K Pin2 GND. Not battery.
j8_pads = [
    pad("1", j8_local_k, 0, 11, "PSU_K", 0.9, 1.6),
    pad("2", j8_local_g, 0, 1, "GND", 0.9, 1.6),
]
parts.append(
    fp_header("Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", "B.Cu", J8, "J8", "SW3-PWR-PH2.0", "External SW3. Pin1 K Pin2 GND. Cable exits -X. Not 5V / not BAT")
    + "".join(j8_pads)
    + "\t)\n"
)
tht += [("J8.1", *j8_k), ("J8.2", *j8_g)]

# R10 on B.Cu — front-view ±2, board coords mirrored
r_pads = [pad("1", -2.0, 0, 5, "UART1_RX"), pad("2", 2.0, 0, 10, "UART1_RX_HC")]
parts.append(
    fp_header("Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal", "B.Cu", R10_AT, "R10", "1k")
    + "".join(r_pads)
    + "\t)\n"
)
tht += [("R10.1", *r10_a), ("R10.2", *r10_b)]

c_pads = [pad("1", 0, 2.0, 2, "5V", 0.9, 1.6), pad("2", 0, -2.0, 1, "GND", 0.9, 1.6)]
parts.append(
    fp_header("Capacitor_THT:CP_Radial_D5.0mm_P2.50mm", "B.Cu", C6_AT, "C6", "220u")
    + "".join(c_pads)
    + "\t)\n"
)
tht += [("C6.1", *c6_p), ("C6.2", *c6_n)]

hole_xy = []
for i in range(4):
    ang = math.radians(i * 90.0)  # on the +X, +Y, −X, −Y axes
    hx = CX + BOLT_R * math.cos(ang)
    hy = CY + BOLT_R * math.sin(ang)
    hole_xy.append((hx, hy))
    parts.append(f"""\t(footprint "MountingHole:MountingHole_3.2mm_M3"
\t\t(layer "F.Cu")
\t\t(uuid "{uid()}")
\t\t(at {hx:.3f} {hy:.3f} 0)
\t\t(property "Reference" "H{i+1}" (at 0 4.2 0) (layer "F.SilkS") (uuid "{uid()}")
\t\t\t(effects (font (size 0.9 0.9) (thickness 0.12))))
\t\t(property "Value" "M3" (at 0 -4.2 0) (layer "F.SilkS") (hide yes) (uuid "{uid()}")
\t\t\t(effects (font (size 0.8 0.8) (thickness 0.12))))
\t\t(attr through_hole exclude_from_pos_files)
{npth("", 0, 0, 3.2)}\t)
""")
    tht.append((f"H{i+1}", hx, hy))

graphics: list[str] = []
graphics.append(
    f'\t(gr_circle (center {CX:.3f} {CY:.3f}) (end {CX+R:.3f} {CY:.3f}) '
    f'(stroke (width 0.15) (type default)) (fill none) (layer "Edge.Cuts") (uuid "{uid()}"))\n'
)
graphics.append(
    f'\t(gr_circle (center {CX:.3f} {CY:.3f}) (end {CX+BOLT_R:.3f} {CY:.3f}) '
    f'(stroke (width 0.12) (type dash)) (fill none) (layer "Dwgs.User") (uuid "{uid()}"))\n'
)

# F.SilkS top
graphics.append(silk_rect(*K902))
graphics.append(silk_rect(*K902, "F.CrtYd"))
graphics.append(silk_text("K902 38x38  SMA +Y", 42.0, 80.6, 1.0))
graphics.append(silk_text("pins 34 mm", 42.0, 39.2, 0.8))
graphics.append(silk_rect(ESP[0] - 13.2, ESP[1] - 18.5, ESP[0] + 13.2, ESP[1] + 18.5))
graphics.append(silk_text("ESP32 FEMALE 2x15  USB -Y", ESP[0] - 12, ESP[1] - 20.2, 0.85))
graphics.append(silk_text("J7 JST-PH 6P SPARE", J7[0] - 8.5, j7y(1) + 2.2, 0.75))
graphics.append(silk_text("SW3 PWR SW", 10.0, 73.6, 0.8))
graphics.append(silk_text("J8 K-GND  ext", 10.0, 66.0, 0.7))
graphics.append(silk_text("5V", J7[0] - 5.2, j7y(1) - 0.35, 0.55))
graphics.append(silk_text("TX", J7[0] - 5.2, j7y(2) - 0.35, 0.55))
graphics.append(silk_text("RX", J7[0] - 5.2, j7y(3) - 0.35, 0.55))
graphics.append(silk_text("EVT", J7[0] - 5.6, j7y(4) - 0.35, 0.55))
graphics.append(silk_text("GND", J7[0] - 5.6, j7y(5) - 0.35, 0.55))
graphics.append(silk_text("NC", J7[0] - 5.2, j7y(6) - 0.35, 0.55))
graphics.append(silk_text("DIA 120  4xM3 BCD 112", 42, 8.5, 1.0))
graphics.append(silk_rect(BZ[0] - 6.0, BZ[1] - 6.0, BZ[0] + 6.0, BZ[1] + 6.0))
graphics.append(silk_text("BZ1 GPIO23", BZ[0] - 8.0, BZ[1] + 7.2, 0.7))
for i, name in enumerate(k902_left_names, start=1):
    graphics.append(silk_text(name, K902_XL + 1.6, k902_left_y(i) - 0.35, 0.55))
for i, name in enumerate(k902_right_names, start=1):
    graphics.append(silk_text(name, K902_XR - 5.4, k902_right_y(i) - 0.35, 0.55))
graphics.append(
    f'\t(gr_line (start {CX-3:.3f} {CY:.3f}) (end {CX+3:.3f} {CY:.3f}) '
    f'(stroke (width 0.12) (type default)) (layer "Dwgs.User") (uuid "{uid()}"))\n'
)
graphics.append(
    f'\t(gr_line (start {CX:.3f} {CY-3:.3f}) (end {CX:.3f} {CY+3:.3f}) '
    f'(stroke (width 0.12) (type default)) (layer "Dwgs.User") (uuid "{uid()}"))\n'
)

# B.SilkS bottom — absolute board coords, mirrored text (readable from back)
graphics.append(silk_rect(*hc_body, "B.SilkS"))
graphics.append(silk_rect(*hc_body, "B.CrtYd"))
graphics.append(silk_text("HC-12 ANT +X", hc_body[0] + 1, hc_body[3] + 1.2, 0.9, "B.SilkS", True))
graphics.append(silk_text("VCC", hc_vcc[0] - 6.2, hc_vcc[1] - 0.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("GND", hc_gnd[0] - 6.2, hc_gnd[1] - 0.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("RXD", hc_rxd[0] - 6.2, hc_rxd[1] - 0.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("TXD", hc_txd[0] - 6.2, hc_txd[1] - 0.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("SET", hc_set[0] - 6.2, hc_set[1] - 0.4, 0.7, "B.SilkS", True))
graphics.append(silk_rect(*PSU, "B.SilkS"))
graphics.append(silk_rect(*PSU, "B.CrtYd"))
graphics.append(silk_text("USB-C 5V  USB +Y", PSU[0] + 0.4, PSU[3] + 1.2, 0.85, "B.SilkS", True))
graphics.append(silk_text("+", psu_p[0] + 1.2, psu_p[1] - 0.3, 1.0, "B.SilkS", True))
graphics.append(silk_text("-", psu_n[0] + 1.2, psu_n[1] - 0.3, 1.0, "B.SilkS", True))
graphics.append(silk_text("K", psu_k[0] + 1.0, psu_k[1] - 2.2, 0.9, "B.SilkS", True))
graphics.append(silk_text("BAT PH2.0", J5[0] + 2.2, J5[1] + 4.2, 0.9, "B.SilkS", True))
graphics.append(silk_text("BAT+", j5_p[0] + 1.4, j5_p[1] + 2.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("GND", j5_n[0] + 1.4, j5_n[1] - 2.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("SW3 PWR SW", J8[0] + 2.2, J8[1] + 5.0, 0.85, "B.SilkS", True))
graphics.append(silk_text("J8 K-GND  not BAT", J8[0] + 2.2, J8[1] + 3.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("K", j8_k[0] + 1.4, j8_k[1] + 2.2, 0.7, "B.SilkS", True))
graphics.append(silk_text("GND", j8_g[0] + 1.4, j8_g[1] - 2.4, 0.7, "B.SilkS", True))
graphics.append(silk_text("5V: USB-C OR BAT  one source", 14, 76.6, 0.75, "B.SilkS", True))
graphics.append(silk_text("C6 220u", C6_AT[0] + 2.2, C6_AT[1], 0.7, "B.SilkS", True))
graphics.append(silk_text("R10 1k", R10_AT[0] - 8, R10_AT[1] + 2.4, 0.7, "B.SilkS", True))
graphics.append(silk_rect(GY[0], GY[1] - 7.0, GY[0] + 14.0, GY[1] + 7.0, "F.SilkS"))
graphics.append(silk_text("GY-273 MAG", GY[0] + 0.5, GY[1] + 8.0, 0.8))
graphics.append(silk_text("1=VCC 2=GND 3=SCL 4=SDA 5=DRDY", GY[0] - 1.0, GY[1] - 8.5, 0.6))

graphics.append(keepout(40, 86, 80, 114, "K902_GNSS_keepout"))
graphics.append(keepout(88, 78, 114, 96, "ESP32_24G_keepout"))
graphics.append(keepout(72, 16, 90, 36, "HC12_433_keepout"))

def on_disk(x: float, y: float, margin: float = 2.0) -> bool:
    return dist(x, y) <= (R - margin)


tracks: list[str] = []
# UART2 on F.Cu: approach K902 TX2/RX2 from inside the module, not along the pin column
# West of the left pin row (x=43) so the verticals do not run through the header.
tracks.append(segment(*gpio16, 39.0, gpio16[1], 3, 0.35))
tracks.append(segment(39.0, gpio16[1], 39.0, k902_tx2[1], 3, 0.35))
tracks.append(segment(39.0, k902_tx2[1], *k902_tx2, 3, 0.35))
tracks.append(segment(*gpio17, 41.0, gpio17[1], 4, 0.35))
tracks.append(segment(41.0, gpio17[1], 41.0, k902_rx2[1], 4, 0.35))
tracks.append(segment(41.0, k902_rx2[1], *k902_rx2, 4, 0.35))
tracks.append(segment(*j7_rx, 39.0, j7_rx[1], 3, 0.35))
tracks.append(segment(39.0, j7_rx[1], 39.0, gpio16[1], 3, 0.35))
tracks.append(segment(*j7_tx, 41.0, j7_tx[1], 4, 0.35))
tracks.append(segment(41.0, j7_tx[1], 41.0, gpio17[1], 4, 0.35))

# BUZZER on B.Cu: GPIO23 is the top of the ESP left header. Jog east of the HC-12 SET trace.
gpio23 = (LX, py(14))
tracks.append(segment(*gpio23, 82.0, gpio23[1], 12, 0.4, "B.Cu"))
tracks.append(segment(82.0, gpio23[1], 82.0, 46.0, 12, 0.4, "B.Cu"))
tracks.append(segment(82.0, 46.0, 93.0, 46.0, 12, 0.4, "B.Cu"))
tracks.append(segment(93.0, 46.0, 93.0, bz_io[1], 12, 0.4, "B.Cu"))
tracks.append(segment(93.0, bz_io[1], *bz_io, 12, 0.4, "B.Cu"))
# 5V from JP1, staying below the DevKit body.
tracks.append(segment(jp_at[0] - 1.27, jp_at[1], jp_at[0] - 1.27, bz_vcc[1], 2, 0.6))
tracks.append(segment(jp_at[0] - 1.27, bz_vcc[1], *bz_vcc, 2, 0.6))
# GND up the gap between K902 and ESP, below the UART2 traces.
tracks.append(segment(*bz_gnd, 83.0, bz_gnd[1], 1, 0.5))
tracks.append(segment(83.0, bz_gnd[1], 83.0, esp_gnd_l[1], 1, 0.5))
tracks.append(segment(83.0, esp_gnd_l[1], *esp_gnd_l, 1, 0.5))

# UART1 on B.Cu: approach HC-12 from the pin side (west), ANT hangs +X
tracks.append(segment(*gpio27, 90.0, gpio27[1], 6, 0.35, "B.Cu"))
tracks.append(segment(90.0, gpio27[1], 90.0, 40.0, 6, 0.35, "B.Cu"))
tracks.append(segment(90.0, 40.0, 44.0, 40.0, 6, 0.35, "B.Cu"))
tracks.append(segment(44.0, 40.0, 44.0, hc_rxd[1], 6, 0.35, "B.Cu"))
tracks.append(segment(44.0, hc_rxd[1], *hc_rxd, 6, 0.35, "B.Cu"))
tracks.append(segment(*gpio26, *r10_a, 5, 0.35, "B.Cu"))
tracks.append(segment(*r10_b, 88.0, r10_b[1], 10, 0.35, "B.Cu"))
tracks.append(segment(88.0, r10_b[1], 88.0, 38.0, 10, 0.35, "B.Cu"))
tracks.append(segment(88.0, 38.0, 41.0, 38.0, 10, 0.35, "B.Cu"))
tracks.append(segment(41.0, 38.0, 41.0, hc_txd[1], 10, 0.35, "B.Cu"))
tracks.append(segment(41.0, hc_txd[1], *hc_txd, 10, 0.35, "B.Cu"))
tracks.append(segment(*gpio25, 86.0, gpio25[1], 7, 0.35, "B.Cu"))
tracks.append(segment(86.0, gpio25[1], 86.0, 42.0, 7, 0.35, "B.Cu"))
tracks.append(segment(86.0, 42.0, 38.0, 42.0, 7, 0.35, "B.Cu"))
tracks.append(segment(38.0, 42.0, 38.0, hc_set[1], 7, 0.35, "B.Cu"))
tracks.append(segment(38.0, hc_set[1], *hc_set, 7, 0.35, "B.Cu"))

# 5V on B.Cu — down from PSU + pad, not across PSU GND
v5x = 33.0
tracks.append(segment(*psu_p, v5x, psu_p[1], 2, 1.0, "B.Cu"))
tracks.append(segment(v5x, psu_p[1], v5x, j5_p[1], 2, 1.0, "B.Cu"))
tracks.append(segment(v5x, j5_p[1], *j5_p, 2, 0.8, "B.Cu"))
tracks.append(segment(v5x, j5_p[1], v5x, c6_p[1], 2, 0.8, "B.Cu"))
tracks.append(segment(v5x, c6_p[1], *c6_p, 2, 0.8, "B.Cu"))
tracks.append(segment(*c6_p, hc_vcc[0], c6_p[1], 2, 0.8, "B.Cu"))
tracks.append(segment(hc_vcc[0], c6_p[1], *hc_vcc, 2, 0.8, "B.Cu"))
jp5 = (jp_at[0] - 1.27, 36.0)
tracks.append(segment(v5x, j5_p[1], v5x, 36.0, 2, 1.0, "B.Cu"))
tracks.append(segment(v5x, 36.0, jp5[0], 36.0, 2, 1.0, "B.Cu"))
tracks.append(via(*jp5, 2))
tracks.append(segment(jp5[0], 36.0, jp_at[0] - 1.27, jp_at[1], 2, 0.8))
tracks.append(segment(*esp_vin, jp_at[0] + 1.27, esp_vin[1], 9, 0.8))
tracks.append(segment(v5x, psu_p[1], v5x, k902_5v[1], 2, 0.8, "B.Cu"))
tracks.append(segment(v5x, k902_5v[1], *k902_5v, 2, 0.8, "B.Cu"))
tracks.append(segment(v5x, j7_5v[1], *j7_5v, 2, 0.8, "B.Cu"))
tracks.append(via(*j7_gnd, 1))

# GND — rim bus at x=7 so it does not cross PSU_K (K stays east of this bus)
tracks.append(segment(*psu_n, j5_n[0], psu_n[1], 1, 0.6, "B.Cu"))
tracks.append(segment(j5_n[0], psu_n[1], *j5_n, 1, 0.6, "B.Cu"))
tracks.append(via(*psu_n, 1))
tracks.append(via(*hc_gnd, 1))
tracks.append(via(*esp_gnd_l, 1))
tracks.append(via(*esp_gnd_r, 1))
tracks.append(via(*c6_n, 1))
tracks.append(via(*j8_g, 1))

# I2C to GY-273 (J4): ESP32 GPIO21=SDA GPIO22=SCL, 3V3 power.
# F.Cu channel between K902 (x<=79) and ESP32 (x>=87.3), then B.Cu hops to pads.
gpio21 = (LX, py(10))
gpio22 = (LX, py(13))
esp_3v3 = (LX, py(0))
# SDA net13 — escape y67.62, vertical x85.6, trunk y38.0, via (22,38.0), B.Cu to pad4 (20,32.54)
tracks.append(segment(*gpio21, 85.6, gpio21[1], 13, 0.35))
tracks.append(segment(85.6, gpio21[1], 85.6, 38.0, 13, 0.35))
tracks.append(segment(85.6, 38.0, 22.0, 38.0, 13, 0.35))
tracks.append(via(22.0, 38.0, 13))
tracks.append(segment(22.0, 38.0, 22.0, 32.54, 13, 0.35, "B.Cu"))
tracks.append(segment(22.0, 32.54, 20.0, 32.54, 13, 0.35, "B.Cu"))
# SCL net14 — escape y75.24, vertical x84.8, trunk y38.8, via (24.5,38.8), B.Cu to pad3 (20,30.0)
tracks.append(segment(*gpio22, 84.8, gpio22[1], 14, 0.35))
tracks.append(segment(84.8, gpio22[1], 84.8, 38.8, 14, 0.35))
tracks.append(segment(84.8, 38.8, 24.5, 38.8, 14, 0.35))
tracks.append(via(24.5, 38.8, 14))
tracks.append(segment(24.5, 38.8, 24.5, 30.0, 14, 0.35, "B.Cu"))
tracks.append(segment(24.5, 30.0, 20.0, 30.0, 14, 0.35, "B.Cu"))
# 3V3 net8 — escape y42.22, vertical x86.4, trunk y37.0, via (27,37.0), B.Cu to pad1 (20,24.92)
tracks.append(segment(*esp_3v3, 86.4, esp_3v3[1], 8, 0.4))
tracks.append(segment(86.4, esp_3v3[1], 86.4, 37.0, 8, 0.4))
tracks.append(segment(86.4, 37.0, 27.0, 37.0, 8, 0.4))
tracks.append(via(27.0, 37.0, 8))
tracks.append(segment(27.0, 37.0, 27.0, 24.92, 8, 0.4, "B.Cu"))
tracks.append(segment(27.0, 24.92, 20.0, 24.92, 8, 0.4, "B.Cu"))

# PSU_K on B.Cu: module K → J8.1 (external SW3). Never tie to 5V.
tracks.append(segment(*psu_k, psu_k[0], 74.0, 11, 0.5, "B.Cu"))
tracks.append(segment(psu_k[0], 74.0, 12.0, 74.0, 11, 0.5, "B.Cu"))
tracks.append(segment(12.0, 74.0, 12.0, j8_k[1], 11, 0.5, "B.Cu"))
tracks.append(segment(12.0, j8_k[1], *j8_k, 11, 0.5, "B.Cu"))

nseg = 64
rz = R - 1.5
circle_pts = " ".join(
    f"(xy {CX + rz * math.cos(2 * math.pi * i / nseg):.3f} {CY + rz * math.sin(2 * math.pi * i / nseg):.3f})"
    for i in range(nseg)
)
zone = f"""\t(zone
\t\t(net 1)
\t\t(net_name "GND")
\t\t(layer "F.Cu")
\t\t(uuid "{uid()}")
\t\t(hatch (thickness 0.5) (gap 0.7) (orientation 0))
\t\t(connect_pads (clearance 0.3))
\t\t(min_thickness 0.25)
\t\t(filled_areas_thickness no)
\t\t(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
\t\t(polygon (pts {circle_pts}))
\t)
\t(zone
\t\t(net 1)
\t\t(net_name "GND")
\t\t(layer "B.Cu")
\t\t(uuid "{uid()}")
\t\t(hatch (thickness 0.5) (gap 0.7) (orientation 0))
\t\t(connect_pads (clearance 0.3))
\t\t(min_thickness 0.25)
\t\t(filled_areas_thickness no)
\t\t(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
\t\t(polygon (pts {circle_pts}))
\t)
"""

layers = """\t(layers
\t\t(0 "F.Cu" signal)
\t\t(31 "B.Cu" signal)
\t\t(32 "B.Adhes" user "B.Adhesive")
\t\t(33 "F.Adhes" user "F.Adhesive")
\t\t(34 "B.Paste" user)
\t\t(35 "F.Paste" user)
\t\t(36 "B.SilkS" user "B.Silkscreen")
\t\t(37 "F.SilkS" user "F.Silkscreen")
\t\t(38 "B.Mask" user)
\t\t(39 "F.Mask" user)
\t\t(40 "Dwgs.User" user "User.Drawings")
\t\t(41 "Cmts.User" user "User.Comments")
\t\t(42 "Eco1.User" user "User.Eco1")
\t\t(43 "Eco2.User" user "User.Eco2")
\t\t(44 "Edge.Cuts" user)
\t\t(45 "Margin" user)
\t\t(46 "B.CrtYd" user "B.Courtyard")
\t\t(47 "F.CrtYd" user "F.Courtyard")
\t\t(48 "B.Fab" user "B.Fabrication")
\t\t(49 "F.Fab" user "F.Fabrication")
\t)
"""

header = f"""(kicad_pcb
\t(version 20241229)
\t(generator "gnss-head-carrier")
\t(generator_version "9.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(title_block
\t\t(title "GNSS Head Carrier")
\t\t(date "2026-11-03")
\t\t(rev "19")
\t\t(company "ESP32_Rover_NEZ")
\t\t(comment 1 "DIA 120  K902 38x38 CENTER  SMA +Y  ESP FEMALE 2x15  PSU +Y  J8 SW3-PWR ext  HC-12 ANT+X")
\t)
{layers}\t(setup
\t\t(pad_to_mask_clearance 0.05)
\t\t(allow_soldermask_bridges_in_footprints no)
\t\t(tenting front back)
\t)
"""

body = header + "".join(net_decl(n, name) for n, name in NETS)
body += "".join(parts)
body += "".join(graphics)
body += "".join(tracks)
body += zone
body += ")\n"
OUT.write_text(body, encoding="utf-8")

print(f"wrote {OUT} bytes={OUT.stat().st_size}")
print(f"HC body={tuple(round(v,2) for v in hc_body)} pins={HC_PINS} ANT=+X")
print(f"K902 L 1x9 x={K902_XL} GND={k902_gnd_l} TX2={k902_tx2} RX2={k902_rx2} 5V={k902_5v}")
print(f"K902 R 1x10 x={K902_XR} GND={k902_gnd_r}")
print(f"J5 PH2.0 board pads BAT+={j5_p} GND={j5_n}")
print(f"J8 SW3-PWR PH2.0 K={j8_k} GND={j8_g}")
print(f"U3 K={psu_k} +={psu_p} -={psu_n}")
print(f"J7 PH6 spare 5V={j7_5v} TX={j7_tx} RX={j7_rx} GND={j7_gnd}")
print(f"R10 board pads RX={r10_a} RX_HC={r10_b}")
print(f"holes {[ (round(x,2), round(y,2)) for x,y in hole_xy ]}")

corners = {
    "K902": [(K902[0], K902[1]), (K902[2], K902[1]), (K902[0], K902[3]), (K902[2], K902[3])],
    "ESP": [(ESP[0] - 14, ESP[1] - 26), (ESP[0] + 14, ESP[1] - 26), (ESP[0] - 14, ESP[1] + 26), (ESP[0] + 14, ESP[1] + 26)],
    "HC": [(hc_body[0], hc_body[1]), (hc_body[2], hc_body[1]), (hc_body[0], hc_body[3]), (hc_body[2], hc_body[3])],
    "PSU": [(PSU[0], PSU[1]), (PSU[2], PSU[1]), (PSU[0], PSU[3]), (PSU[2], PSU[3])],
}
for name, pts in corners.items():
    ds = [dist(*p) for p in pts]
    print(f"  {name} r_max={max(ds):.2f} outside={[p for p,d in zip(pts,ds) if d>59.5]}")

# THT clearance 2.4 mm between different refs
hits = []
for i, (a, xa, ya) in enumerate(tht):
    for b, xb, yb in tht[i + 1 :]:
        if a.split(".")[0] == b.split(".")[0]:
            continue
        d = math.hypot(xa - xb, ya - yb)
        if d < 2.4:
            hits.append((a, b, round(d, 2)))
print("tht collisions", hits or "none")

off = []
for sm in re.finditer(
    r'\(segment \(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\) \(width ([-\d.]+)\) \(layer "([^"]+)"\)',
    body,
):
    x0, y0, x1, y1 = (float(sm.group(i)) for i in range(1, 5))
    for t in (0.0, 0.5, 1.0):
        x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        if not on_disk(x, y, 1.5):
            off.append((round(x, 2), round(y, 2), sm.group(6), round(dist(x, y), 2)))
            break
print("tracks off-disk", off or "none")
for v in re.finditer(r"\(via \(at ([-\d.]+) ([-\d.]+)\)", body):
    x, y = float(v.group(1)), float(v.group(2))
    if not on_disk(x, y, 1.5):
        print("via off-disk", round(x, 2), round(y, 2), round(dist(x, y), 2))

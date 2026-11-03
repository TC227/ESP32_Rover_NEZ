"""SVG preview of URTK1.2 footprint for visual check."""
from pathlib import Path

OUT = Path(__file__).with_name("easyeda") / "URTK1.2_preview.svg"

BOARD_W, BOARD_H = 38.0, 38.0
HOLE_DX, HOLE_DY, HOLE_D = 30.0, 30.0, 2.2
PITCH, ROW_X = 2.54, 17.0
LEFT = ["SCK", "TX_MISO", "RX_MOSI", "CS", "RX2", "TX2", "3V3", "5V", "GND"]
RIGHT = ["INT", "SAFE", "RST", "PPS", "RTK", "FENCE", "SCL", "SDA", "3V3", "GND"]

SCALE = 8  # px per mm
OX, OY = 60, 70


def sx(x: float) -> float:
    return OX + (x + BOARD_W / 2 + 8) * SCALE


def sy(y: float) -> float:
    return OY + (BOARD_H / 2 + 12 - y) * SCALE


def main() -> None:
    w = int((BOARD_W + 24) * SCALE + OX)
    h = int((BOARD_H + 28) * SCALE + OY)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}" style="background:#1a1a1a">',
        "<style>text{font-family:monospace;font-size:9px;fill:#ffcc00}"
        ".dim{fill:#88ccff;font-size:10px}.title{font-size:14px;fill:#fff}</style>",
        f'<text class="title" x="20" y="28">URTK1.2 / K902  {BOARD_W}×{BOARD_H} mm</text>',
        # board
        f'<rect x="{sx(-BOARD_W/2)}" y="{sy(BOARD_H/2)}" width="{BOARD_W*SCALE}" '
        f'height="{BOARD_H*SCALE}" fill="#222" stroke="#ffcc00" stroke-width="2"/>',
        # module
        f'<rect x="{sx(-11)}" y="{sy(8.5)}" width="{22*SCALE}" height="{17*SCALE}" '
        f'fill="#666" stroke="#aaa" stroke-width="1"/>',
        f'<text x="{sx(-4)}" y="{sy(0)}" fill="#fff">K902</text>',
        # SMA
        f'<rect x="{sx(-5)}" y="{sy(BOARD_H/2+10)}" width="{10*SCALE}" height="{10*SCALE}" '
        f'fill="none" stroke="#daa520" stroke-width="1.5" stroke-dasharray="4"/>',
        f'<text x="{sx(-3)}" y="{sy(BOARD_H/2+5)}" fill="#daa520">SMA +Y</text>',
        # USB
        f'<rect x="{sx(-6)}" y="{sy(-BOARD_H/2)}" width="{5*SCALE}" height="{3.5*SCALE}" '
        f'fill="none" stroke="#88f" stroke-width="1.5" stroke-dasharray="4"/>',
        f'<text x="{sx(-5.5)}" y="{sy(-BOARD_H/2-2)}" fill="#88f">USB</text>',
    ]
    # holes
    for hx, hy in [
        (-HOLE_DX / 2, HOLE_DY / 2),
        (HOLE_DX / 2, HOLE_DY / 2),
        (-HOLE_DX / 2, -HOLE_DY / 2),
        (HOLE_DX / 2, -HOLE_DY / 2),
    ]:
        parts.append(
            f'<circle cx="{sx(hx)}" cy="{sy(hy)}" r="{(HOLE_D/2)*SCALE}" '
            f'fill="none" stroke="#fff" stroke-width="1.5"/>'
        )
    gnd_y = -((len(RIGHT) - 1) / 2) * PITCH
    for i, name in enumerate(LEFT):
        y = gnd_y + (len(LEFT) - 1 - i) * PITCH
        parts.append(
            f'<circle cx="{sx(-ROW_X)}" cy="{sy(y)}" r="{0.85*SCALE}" '
            f'fill="#c0c0c0" stroke="#f44" stroke-width="1"/>'
        )
        parts.append(f'<text x="{sx(-ROW_X-6.5)}" y="{sy(y)+3}">{i+1} {name}</text>')
    for i, name in enumerate(RIGHT):
        y = gnd_y + (len(RIGHT) - 1 - i) * PITCH
        parts.append(
            f'<circle cx="{sx(ROW_X)}" cy="{sy(y)}" r="{0.85*SCALE}" '
            f'fill="#c0c0c0" stroke="#f44" stroke-width="1"/>'
        )
        parts.append(f'<text x="{sx(ROW_X+1.5)}" y="{sy(y)+3}">{len(LEFT)+i+1} {name}</text>')
    # dims
    parts.append(
        f'<text class="dim" x="{sx(0)-40}" y="{sy(-BOARD_H/2-7)}">'
        f'holes {HOLE_DX}×{HOLE_DY}  rows ±{ROW_X} (= {ROW_X*2} mm)</text>'
    )
    parts.append("</svg>")
    OUT.write_text("\n".join(parts), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()

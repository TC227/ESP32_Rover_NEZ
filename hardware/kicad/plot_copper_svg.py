"""Export F.Cu / B.Cu artwork SVG from gnss-head.kicad_pcb (no KiCad CLI)."""
from __future__ import annotations

import math
import re
from pathlib import Path

PCB = Path(__file__).with_name("gnss-head.kicad_pcb")
OUT = Path(__file__).with_name("plot")
CX, CY, R = 60.0, 60.0, 60.0


def flip_y(y: float) -> float:
    return 120.0 - y


def parse():
    text = PCB.read_text(encoding="utf-8")
    pads: list[tuple[float, float, float, str]] = []  # x, y, size, layer_hint
    segs: list[tuple[float, float, float, float, float, str]] = []
    vias: list[tuple[float, float, float]] = []

    fp_re = re.compile(
        r'\(footprint .*?\n\t\t\(layer "([^"]+)"\).*?\n\t\t\(at ([-\d.]+) ([-\d.]+)',
        re.S,
    )
    # Walk footprints by splitting
    chunks = re.split(r'\n\t\(footprint ', text)
    for chunk in chunks[1:]:
        mlayer = re.search(r'\(layer "([^"]+)"\)', chunk)
        mat = re.search(r'\(at ([-\d.]+) ([-\d.]+)', chunk)
        if not mat:
            continue
        fx, fy = float(mat.group(1)), float(mat.group(2))
        for pm in re.finditer(
            r'\(pad "[^"]*" \w+ circle\s+\(at ([-\d.]+) ([-\d.]+).*?\(size ([-\d.]+)',
            chunk,
            re.S,
        ):
            px, py, sz = float(pm.group(1)), float(pm.group(2)), float(pm.group(3))
            layer = mlayer.group(1) if mlayer else "F.Cu"
            pads.append((fx + px, fy + py, sz, layer))

    for sm in re.finditer(
        r'\(segment \(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\) '
        r'\(width ([-\d.]+)\) \(layer "([^"]+)"\)',
        text,
    ):
        segs.append(tuple(float(sm.group(i)) for i in range(1, 6)) + (sm.group(6),))  # type: ignore

    for vm in re.finditer(r'\(via \(at ([-\d.]+) ([-\d.]+)\) \(size ([-\d.]+)\)', text):
        vias.append((float(vm.group(1)), float(vm.group(2)), float(vm.group(3))))

    return pads, segs, vias


def svg_layer(name: str, pads, segs, vias, mirror: bool) -> str:
    def tx(x: float) -> float:
        return (120.0 - x) if mirror else x

    clip = f'<clipPath id="disk"><circle cx="{CX}" cy="{flip_y(CY)}" r="{R}"/></clipPath>'
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 120 120" width="120mm" height="120mm">',
        clip,
        f'<rect width="120" height="120" fill="#1a1510"/>',
        f'<g clip-path="url(#disk)">',
        f'<circle cx="{CX}" cy="{flip_y(CY)}" r="{R}" fill="#2a2118"/>',
    ]
    layer = "B.Cu" if "B" in name else "F.Cu"
    for x0, y0, x1, y1, w, ly in segs:
        if ly != layer:
            continue
        parts.append(
            f'<line x1="{tx(x0):.3f}" y1="{flip_y(y0):.3f}" x2="{tx(x1):.3f}" y2="{flip_y(y1):.3f}" '
            f'stroke="#c9a227" stroke-width="{w:.3f}" stroke-linecap="round"/>'
        )
    for x, y, sz, _hint in pads:
        parts.append(
            f'<circle cx="{tx(x):.3f}" cy="{flip_y(y):.3f}" r="{sz/2:.3f}" fill="#d4b84a"/>'
            f'<circle cx="{tx(x):.3f}" cy="{flip_y(y):.3f}" r="0.45" fill="#1a1510"/>'
        )
    for x, y, sz in vias:
        parts.append(
            f'<circle cx="{tx(x):.3f}" cy="{flip_y(y):.3f}" r="{sz/2:.3f}" fill="#e8d48a"/>'
            f'<circle cx="{tx(x):.3f}" cy="{flip_y(y):.3f}" r="0.2" fill="#1a1510"/>'
        )
    parts.append("</g>")
    parts.append(
        f'<circle cx="{CX}" cy="{flip_y(CY)}" r="{R-0.15}" fill="none" stroke="#e8d48a" stroke-width="0.3"/>'
    )
    parts.append(f'<text x="4" y="8" fill="#e8d48a" font-size="3.2" font-family="sans-serif">{name}</text>')
    parts.append("</svg>")
    return "\n".join(parts)


def vis_marks(pads, segs, vias, layer: str, mirror: bool) -> str:
    def tx(x: float) -> float:
        return (120.0 - x) if mirror else x

    out = []
    for x0, y0, x1, y1, w, ly in segs:
        if ly != layer:
            continue
        sw = max(0.8, w * 2.2)
        out.append(
            f'<line x1="{tx(x0):.2f}" y1="{flip_y(y0):.2f}" x2="{tx(x1):.2f}" y2="{flip_y(y1):.2f}" '
            f'stroke="var(--foreground)" stroke-width="{sw:.2f}" stroke-linecap="round" '
            f'vector-effect="non-scaling-stroke"/>'
        )
    for x, y, sz, _ in pads:
        out.append(
            f'<circle cx="{tx(x):.2f}" cy="{flip_y(y):.2f}" r="{max(0.7, sz/2):.2f}" '
            f'fill="var(--muted)" stroke="var(--foreground)" stroke-width="0.6" vector-effect="non-scaling-stroke"/>'
        )
    for x, y, sz in vias:
        out.append(
            f'<circle cx="{tx(x):.2f}" cy="{flip_y(y):.2f}" r="{sz/2:.2f}" '
            f'fill="var(--card)" stroke="var(--foreground)" stroke-width="0.8" vector-effect="non-scaling-stroke"/>'
        )
    return "\n".join(out)


def main() -> None:
    pads, segs, vias = parse()
    OUT.mkdir(exist_ok=True)
    (OUT / "gnss-head-F_Cu.svg").write_text(svg_layer("F.Cu  front", pads, segs, vias, False), encoding="utf-8")
    (OUT / "gnss-head-B_Cu.svg").write_text(svg_layer("B.Cu  back (mirrored)", pads, segs, vias, True), encoding="utf-8")
    print(f"pads={len(pads)} segs={len(segs)} vias={len(vias)} -> {OUT}")


if __name__ == "__main__":
    main()

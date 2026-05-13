from __future__ import annotations

import contextlib
from io import StringIO

from build123d import (
    Align,
    Axis,
    Color,
    Compound,
    Face,
    Kind,
    Side,
    Wire,
    extrude,
    import_svg,
)


SVG_VIEWBOX_SIZE = 24.0
ICON_WIDTH = 72.0
ICON_SCALE = ICON_WIDTH / SVG_VIEWBOX_SIZE
SVG_STROKE_WIDTH = 2.0
STROKE_RADIUS = SVG_STROKE_WIDTH * ICON_SCALE / 2.0
ICON_EXTRUSION_DEPTH = 6.0

# docs/src/app/globals.css dark primary: oklch(0.6620 0.2110 351.0000)
DOCS_PRIMARY_MAGENTA = Color(
    0.9231487821459046,
    0.29744656234257233,
    0.6368871734610005,
    1.0,
)


def _icon_node_svg(icon_node: list[tuple[str, dict[str, str]]]) -> str:
    elements: list[str] = []
    for tag, attrs in icon_node:
        rendered_attrs = " ".join(
            f'{name}="{value}"'
            for name, value in attrs.items()
            if name != "key"
        )
        elements.append(f"  <{tag} {rendered_attrs} />")
    body = "\n".join(elements)
    return f"""\
<svg
  xmlns="http://www.w3.org/2000/svg"
  width="24"
  height="24"
  viewBox="0 0 24 24"
  fill="none"
  stroke="currentColor"
  stroke-width="2"
  stroke-linecap="round"
  stroke-linejoin="round"
>
{body}
</svg>
"""


def _import_centerline_wires(icon_node: list[tuple[str, dict[str, str]]]) -> list[Wire]:
    capture = StringIO()
    with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
        shapes = import_svg(
            StringIO(_icon_node_svg(icon_node)),
            align=Align.CENTER,
            ignore_visibility=True,
        )

    wires: list[Wire] = []
    for shape in shapes:
        if isinstance(shape, Face):
            wire = shape.outer_wire()
        elif isinstance(shape, Wire):
            wire = shape
        else:
            continue
        if wire is not None and wire.length > 1.0e-6:
            wires.append(wire.scale(ICON_SCALE))
    return wires


def _closed_stroke_face(wire: Wire) -> Face:
    try:
        outer = wire.offset_2d(
            STROKE_RADIUS,
            kind=Kind.ARC,
            side=Side.BOTH,
            closed=True,
        )
        inner = wire.offset_2d(
            -STROKE_RADIUS,
            kind=Kind.ARC,
            side=Side.BOTH,
            closed=True,
        )
        return Face(outer).make_holes([inner])
    except (IndexError, RuntimeError, ValueError):
        outline = wire.offset_2d(
            STROKE_RADIUS,
            kind=Kind.ARC,
            side=Side.BOTH,
            closed=True,
        )
        return Face(outline).make_holes([wire])


def _open_stroke_faces(wire: Wire) -> list[Face]:
    try:
        outline = wire.offset_2d(
            STROKE_RADIUS,
            kind=Kind.ARC,
            side=Side.BOTH,
            closed=True,
        )
        return [Face(outline)]
    except (IndexError, RuntimeError, ValueError):
        faces: list[Face] = []
        for edge in wire.edges():
            outline = edge.offset_2d(
                STROKE_RADIUS,
                kind=Kind.ARC,
                side=Side.BOTH,
                closed=True,
            )
            faces.append(Face(outline))
        return faces


def _stroke_faces(icon_node: list[tuple[str, dict[str, str]]]) -> list[Face]:
    faces: list[Face] = []
    for wire in _import_centerline_wires(icon_node):
        if wire.is_closed:
            faces.append(_closed_stroke_face(wire))
        else:
            faces.extend(_open_stroke_faces(wire))
    return faces


def build_lucide_icon(
    label: str,
    icon_node: list[tuple[str, dict[str, str]]],
) -> Compound:
    stroke_solids = []
    for index, face in enumerate(_stroke_faces(icon_node), start=1):
        stroke = extrude(
            face,
            amount=ICON_EXTRUSION_DEPTH / 2.0,
            both=True,
        )
        stroke.label = f"{label}_stroke_{index:02d}"
        stroke.color = DOCS_PRIMARY_MAGENTA
        stroke_solids.append(stroke)

    icon = Compound(stroke_solids)
    icon = icon.rotate(Axis.X, 90)
    icon.label = label
    icon.color = DOCS_PRIMARY_MAGENTA
    return icon

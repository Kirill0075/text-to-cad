from __future__ import annotations

from lucide_icon_builder import build_lucide_icon


DISPLAY_NAME = "Lucide Globe Icon"

ICON_NODE = [
    ("circle", {"cx": "12", "cy": "12", "r": "10"}),
    ("path", {"d": "M12 2a14.5 14.5 0 0 0 0 20 14.5 14.5 0 0 0 0-20"}),
    ("path", {"d": "M2 12h20"}),
]


def gen_step():
    return build_lucide_icon("globe_icon", ICON_NODE)

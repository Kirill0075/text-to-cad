from __future__ import annotations

from lucide_icon_builder import build_lucide_icon


DISPLAY_NAME = "Lucide Search Icon"

ICON_NODE = [
    ("path", {"d": "m21 21-4.34-4.34"}),
    ("circle", {"cx": "11", "cy": "11", "r": "8"}),
]


def gen_step():
    return build_lucide_icon("search_icon", ICON_NODE)

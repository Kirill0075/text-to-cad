from __future__ import annotations

from lucide_icon_builder import build_lucide_icon


DISPLAY_NAME = "Lucide Compass Icon"

ICON_NODE = [
    ("circle", {"cx": "12", "cy": "12", "r": "10"}),
    (
        "path",
        {
            "d": "m16.24 7.76-1.804 5.411a2 2 0 0 1-1.265 1.265L7.76 16.24l1.804-5.411a2 2 0 0 1 1.265-1.265z"
        },
    ),
]


def gen_step():
    return build_lucide_icon("compass_icon", ICON_NODE)

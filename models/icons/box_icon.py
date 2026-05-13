from __future__ import annotations

from lucide_icon_builder import build_lucide_icon


DISPLAY_NAME = "Lucide Box Icon"

ICON_NODE = [
    (
        "path",
        {
            "d": "M21 8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16Z"
        },
    ),
    ("path", {"d": "m3.3 7 8.7 5 8.7-5"}),
    ("path", {"d": "M12 22V12"}),
]


def gen_step():
    return build_lucide_icon("box_icon", ICON_NODE)

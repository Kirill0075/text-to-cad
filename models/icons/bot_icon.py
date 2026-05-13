from __future__ import annotations

from lucide_icon_builder import build_lucide_icon


DISPLAY_NAME = "Lucide Bot Icon"

ICON_NODE = [
    ("path", {"d": "M12 8V4H8"}),
    ("rect", {"width": "16", "height": "12", "x": "4", "y": "8", "rx": "2"}),
    ("path", {"d": "M2 14h2"}),
    ("path", {"d": "M20 14h2"}),
    ("path", {"d": "M15 13v2"}),
    ("path", {"d": "M9 13v2"}),
]


def gen_step():
    return build_lucide_icon("bot_icon", ICON_NODE)

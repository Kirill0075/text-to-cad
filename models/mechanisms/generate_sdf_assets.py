from __future__ import annotations

from pathlib import Path
import sys

import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
CAD_SCRIPT_DIR = REPO_ROOT / "skills" / "cad" / "scripts"
if str(CAD_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(CAD_SCRIPT_DIR))

from common.glb_topology import read_step_topology_index_from_glb
from sdf_common import SDF_ASSET_DIR_NAME, occurrence_asset_path, _primary_assembly_step
from sdf_profiles import MECHANISM_PROFILES


def main() -> int:
    root = Path(__file__).resolve().parent
    total = 0
    for mechanism_key in sorted(MECHANISM_PROFILES):
        mechanism_dir = root / mechanism_key
        count = export_mechanism_occurrence_meshes(mechanism_dir)
        total += count
        print(f"{mechanism_key}: wrote {count} occurrence meshes")
    print(f"wrote {total} SDF occurrence meshes")
    return 0


def export_mechanism_occurrence_meshes(mechanism_dir: Path) -> int:
    assembly_step = primary_assembly_step(mechanism_dir)
    assembly_glb = assembly_step.with_name(f".{assembly_step.name}.glb")
    scene = trimesh.load(assembly_glb, force="scene")
    topology = read_step_topology_index_from_glb(assembly_glb)
    if not topology or not isinstance(topology.get("assembly"), dict):
        raise ValueError(f"Missing assembly topology in {assembly_glb}")

    asset_dir = mechanism_dir / SDF_ASSET_DIR_NAME
    asset_dir.mkdir(parents=True, exist_ok=True)
    for stale_asset in asset_dir.glob("*.stl"):
        stale_asset.unlink()

    count = 0
    leaves_by_occurrence = {
        str(leaf.get("occurrenceId") or leaf.get("id") or "").strip(): leaf
        for leaf in flatten_leaf_parts(topology["assembly"]["root"])
    }

    for node_index, occurrence_id in enumerate(scene.graph.nodes_geometry, start=1):
        if not occurrence_id:
            continue
        graph_entry = scene.graph.get(occurrence_id)
        if graph_entry is None:
            raise ValueError(f"{assembly_glb} is missing scene node {occurrence_id}")
        _transform, geometry_name = graph_entry
        if geometry_name not in scene.geometry:
            continue
        leaf = leaves_by_occurrence.get(occurrence_id) or {"displayName": str(geometry_name or occurrence_id)}
        display_name = str(leaf.get("displayName") or occurrence_id).strip()
        asset_id = f"node_{node_index:03d}_{name_for_sdf(str(geometry_name or occurrence_id))}"
        output_path = occurrence_asset_path(mechanism_dir, asset_id, display_name)
        mesh = scene.geometry[geometry_name].copy()
        mesh.export(output_path)
        count += 1
    return count


def primary_assembly_step(mechanism_dir: Path) -> Path:
    return _primary_assembly_step(mechanism_dir)


def flatten_leaf_parts(node: dict[str, object]):
    children = node.get("children") if isinstance(node, dict) else None
    if not children and node.get("nodeType") == "part":
        yield node
        return
    for child in children if isinstance(children, list) else []:
        yield from flatten_leaf_parts(child)


def name_for_sdf(value: str) -> str:
    import re

    normalized = re.sub(r"[^0-9A-Za-z_]+", "_", value.strip())
    normalized = re.sub(r"_+", "_", normalized).strip("_").lower()
    if not normalized:
        return "unnamed"
    if normalized[0].isdigit():
        return f"m_{normalized}"
    return normalized


if __name__ == "__main__":
    raise SystemExit(main())

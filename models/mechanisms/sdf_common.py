from __future__ import annotations

import math
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
CAD_SCRIPT_DIR = REPO_ROOT / "skills" / "cad" / "scripts"
if str(CAD_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(CAD_SCRIPT_DIR))

from common.glb_topology import read_step_topology_index_from_glb

SDF_VERSION = "1.12"
ROOT_LINK = "mechanism_root"
MECHANISM_ASSETS_DIR_NAME = "assets"
SOURCE_STEP_ASSET_DIR_NAME = ".source_step_files"
SDF_ASSET_DIR_NAME = "assets/.sdf_meshes"
PRIMARY_ASSEMBLY_STEP_OVERRIDES = {
    # The downloaded STEP archive includes a flat Assembly0 export and the
    # actual Inventor top-level Assembly1/Assembly1T mechanism states. Use the
    # Inventor top-level assembly for the SDF so rigid subassembly intent is
    # available to the mechanism translator.
    "180_degree_flip_mechanism": "Assembly1.stp",
}


def text(parent: ET.Element, tag: str, value: object, attrib: dict[str, str] | None = None) -> ET.Element:
    child = ET.SubElement(parent, tag, attrib or {})
    child.text = str(value)
    return child


def pose(parent: ET.Element, values: tuple[float, float, float, float, float, float], *, relative_to: str) -> ET.Element:
    return text(parent, "pose", " ".join(_fmt_number(value) for value in values), {"relative_to": relative_to})


def gen_mechanism_sdf(mechanism_dir: Path) -> ET.Element:
    mechanism_dir = mechanism_dir.resolve()
    mechanism_key = mechanism_dir.name
    records = assembly_occurrence_records(mechanism_dir)
    _validate_occurrence_mesh_assets(records)

    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    model = ET.SubElement(sdf, "model", {"name": _name_for_sdf(mechanism_key)})
    text(model, "static", "false")
    ET.SubElement(model, "link", {"name": ROOT_LINK})

    for record in records:
        _add_occurrence_link(model, record)
    for record in records:
        _add_occurrence_joint(model, record)

    return sdf


def assembly_occurrence_records(mechanism_dir: Path) -> list[dict[str, object]]:
    mechanism_dir = mechanism_dir.resolve()
    assembly_step = _primary_assembly_step(mechanism_dir)
    assembly_glb = assembly_step.with_name(f".{assembly_step.name}.glb")
    if not assembly_glb.is_file():
        raise FileNotFoundError(f"Missing assembly GLB artifact for {assembly_step.name}: {assembly_glb}")

    scene = trimesh.load(assembly_glb, force="scene")
    topology = read_step_topology_index_from_glb(assembly_glb)
    if not topology or not isinstance(topology.get("assembly"), dict):
        raise ValueError(f"Missing assembly topology in {assembly_glb}")

    leaves_by_occurrence = _leaf_metadata_by_occurrence(topology["assembly"]["root"])
    records = []
    for node_index, occurrence_id in enumerate(scene.graph.nodes_geometry, start=1):
        if not occurrence_id:
            continue
        graph_entry = scene.graph.get(occurrence_id)
        if graph_entry is None:
            raise ValueError(f"Assembly GLB is missing scene node {occurrence_id}")
        transform, geometry_name = graph_entry
        if geometry_name not in scene.geometry:
            continue
        mesh = scene.geometry[geometry_name]
        leaf = leaves_by_occurrence.get(occurrence_id) or {
            "displayName": str(geometry_name or occurrence_id),
            "occurrenceId": occurrence_id,
        }
        asset_id = f"node_{node_index:03d}_{_name_for_sdf(str(geometry_name or occurrence_id))}"
        records.append(_occurrence_record(mechanism_dir, occurrence_id, asset_id, leaf, transform, mesh))

    if not records:
        raise ValueError(f"No assembly occurrences found in {assembly_glb}")
    _assign_unique_link_names(records)
    return records


def occurrence_asset_path(mechanism_dir: Path, occurrence_id: str, display_name: str) -> Path:
    asset_dir = mechanism_dir.resolve() / SDF_ASSET_DIR_NAME
    return asset_dir / f"{_name_for_sdf(occurrence_id)}_{_name_for_sdf(display_name)}.stl"


def _primary_assembly_step(mechanism_dir: Path) -> Path:
    mechanism_dir = mechanism_dir.resolve()
    mechanism_name = mechanism_dir.name
    source_assets_dir = mechanism_dir / MECHANISM_ASSETS_DIR_NAME / SOURCE_STEP_ASSET_DIR_NAME
    override_name = PRIMARY_ASSEMBLY_STEP_OVERRIDES.get(mechanism_name, "")
    if override_name:
        override_path = source_assets_dir / override_name
        if override_path.is_file():
            return override_path
    candidates = [
        mechanism_dir / f"{mechanism_name}.step",
        mechanism_dir / f"{mechanism_name}.stp",
        mechanism_dir / "Assembly0.stp",
        mechanism_dir / "Assembly1.stp",
        source_assets_dir / "Assembly0.stp",
        source_assets_dir / "Assembly1.stp",
    ]
    for preferred in candidates:
        if preferred.is_file():
            return preferred
    assemblies = sorted(
        [*mechanism_dir.glob("Assembly*.stp"), *source_assets_dir.glob("Assembly*.stp")],
        key=lambda path: path.name.lower(),
    )
    if not assemblies:
        raise FileNotFoundError(f"No Assembly*.stp file found in {mechanism_dir}")
    return assemblies[0]


def _flatten_leaf_parts(node: dict[str, object]):
    children = node.get("children") if isinstance(node, dict) else None
    if not children and node.get("nodeType") == "part":
        yield node
        return
    for child in children if isinstance(children, list) else []:
        yield from _flatten_leaf_parts(child)


def _leaf_metadata_by_occurrence(node: dict[str, object]) -> dict[str, dict[str, object]]:
    result: dict[str, dict[str, object]] = {}

    def visit(current: dict[str, object], ancestors: list[str]) -> None:
        children = current.get("children") if isinstance(current, dict) else None
        node_type = current.get("nodeType") if isinstance(current, dict) else None
        display_name = str(current.get("displayName") or current.get("name") or "").strip()
        if not children and node_type == "part":
            occurrence_id = str(current.get("occurrenceId") or current.get("id") or "").strip()
            if occurrence_id:
                result[occurrence_id] = {
                    **current,
                    "assemblyPath": list(ancestors),
                }
            return
        next_ancestors = ancestors
        if node_type == "assembly" and ancestors and display_name:
            next_ancestors = [*ancestors, display_name]
        elif node_type == "assembly" and display_name and display_name != "root":
            next_ancestors = [display_name]
        for child in children if isinstance(children, list) else []:
            visit(child, next_ancestors)

    visit(node, [])
    return result


def _sorted_occurrence_ids(values) -> list[str]:
    def key(value: object):
        text_value = str(value)
        parts = re.split(r"(\d+)", text_value)
        return [int(part) if part.isdigit() else part for part in parts]

    return sorted((str(value) for value in values), key=key)


def _occurrence_record(
    mechanism_dir: Path,
    occurrence_id: str,
    asset_id: str,
    leaf: dict[str, object],
    transform,
    mesh,
) -> dict[str, object]:
    display_name = str(leaf.get("displayName") or occurrence_id).strip()
    base_name = display_name.split(":", 1)[0].strip() or display_name
    transform_matrix = _matrix_array(transform)
    transform_values = _matrix_to_pose(transform_matrix)
    asset_path = occurrence_asset_path(mechanism_dir, asset_id, display_name)
    bounds = getattr(mesh, "bounds", None)
    extents = tuple(float(value) for value in getattr(mesh, "extents", (0.0, 0.0, 0.0)))
    if bounds is None:
        bounds_tuple = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    else:
        bounds_tuple = (
            tuple(float(value) for value in bounds[0]),
            tuple(float(value) for value in bounds[1]),
        )
    return {
        "occurrence_id": occurrence_id,
        "asset_id": asset_id,
        "display_name": display_name,
        "base_name": base_name,
        "assembly_path": tuple(str(value) for value in leaf.get("assemblyPath", ())),
        "asset_path": asset_path,
        "mesh_uri": f"{SDF_ASSET_DIR_NAME}/{asset_path.name}",
        "bounds": bounds_tuple,
        "extents": extents,
        "pose": transform_values,
        "matrix": transform_matrix,
        "link_name": "",
    }


def _matrix_array(transform) -> np.ndarray:
    matrix = np.array(transform.tolist() if hasattr(transform, "tolist") else transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 transform matrix, got {matrix.shape}")
    return matrix


def _matrix_to_pose(transform) -> tuple[float, float, float, float, float, float]:
    matrix = _matrix_array(transform)
    x = float(matrix[0][3])
    y = float(matrix[1][3])
    z = float(matrix[2][3])
    roll, pitch, yaw = _rpy_from_rotation(matrix)
    return (x, y, z, roll, pitch, yaw)


def _rpy_from_rotation(matrix) -> tuple[float, float, float]:
    r00 = float(matrix[0][0])
    r10 = float(matrix[1][0])
    r20 = float(matrix[2][0])
    r21 = float(matrix[2][1])
    r22 = float(matrix[2][2])
    r01 = float(matrix[0][1])
    r11 = float(matrix[1][1])

    pitch = math.atan2(-r20, math.sqrt((r00 * r00) + (r10 * r10)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return (roll, pitch, yaw)


def _assign_unique_link_names(records: list[dict[str, object]]) -> None:
    used: dict[str, int] = {}
    for record in records:
        base_name = f"occ_{_name_for_sdf(str(record['asset_id']))}_{_name_for_sdf(str(record['base_name']))}"
        count = used.get(base_name, 0)
        used[base_name] = count + 1
        record["link_name"] = base_name if count == 0 else f"{base_name}_{count + 1}"


def _validate_occurrence_mesh_assets(records: list[dict[str, object]]) -> None:
    missing = [str(record["mesh_uri"]) for record in records if not Path(str(record["asset_path"])).is_file()]
    if missing:
        sample = ", ".join(missing[:4])
        raise FileNotFoundError(
            "Missing generated SDF occurrence mesh assets. Run "
            "`./.venv/bin/python models/mechanisms/generate_sdf_assets.py` first. "
            f"Missing sample: {sample}"
        )


def _add_occurrence_link(model: ET.Element, record: dict[str, object]) -> None:
    link_name = str(record["link_name"])
    link = ET.SubElement(model, "link", {"name": link_name})
    pose(link, record["pose"], relative_to=ROOT_LINK)
    occurrence_name = _name_for_sdf(str(record["occurrence_id"]))
    visual = ET.SubElement(link, "visual", {"name": f"{occurrence_name}_visual"})
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    text(mesh, "uri", record["mesh_uri"])


def _add_occurrence_joint(model: ET.Element, record: dict[str, object]) -> None:
    link_name = str(record["link_name"])
    joint = ET.SubElement(model, "joint", {"name": f"{link_name}_joint", "type": "fixed"})
    text(joint, "parent", ROOT_LINK)
    text(joint, "child", link_name)


def _name_for_sdf(value: str) -> str:
    normalized = re.sub(r"[^0-9A-Za-z_]+", "_", value.strip())
    normalized = re.sub(r"_+", "_", normalized).strip("_").lower()
    if not normalized:
        return "unnamed"
    if normalized[0].isdigit():
        return f"m_{normalized}"
    return normalized


def _fmt_number(value: float) -> str:
    numeric = 0.0 if abs(float(value)) < 1e-12 else float(value)
    return f"{numeric:.9g}"

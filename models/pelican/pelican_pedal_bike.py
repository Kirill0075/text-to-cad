from __future__ import annotations

import math
from pathlib import Path
from xml.sax.saxutils import escape


# URDF uses meters. This is a stylized kinematic companion to the STEP model in
# pelican_riding_bicycle.py in this folder, scaled to roughly the same overall size.

ARTIFACT_DIR = Path(__file__).resolve().parent
MESH_DIR = ARTIFACT_DIR / "meshes"

ROBOT_NAME = "pelican_pedal_bike"
BIKE_BOTTOM_BRACKET = (0.0, 0.0, 0.049)
PELICAN_BODY_ORIGIN = (-0.012, 0.0, 0.120)
PEDAL_CRANK_RADIUS = 0.020

LEFT_THIGH_LENGTH = 0.0214627388
LEFT_SHANK_LENGTH = 0.020
LEFT_HIP_ORIGIN = (0.0075672630, -0.006, -0.050)
LEFT_HIP_OFFSET = -0.208028951
LEFT_KNEE_OFFSET = 0.208028951

RIGHT_THIGH_LENGTH = 0.0274264407
RIGHT_SHANK_LENGTH = 0.020
RIGHT_HIP_ORIGIN = (0.0087451063, 0.006, -0.0437673851)
RIGHT_HIP_OFFSET = -0.118957581
RIGHT_KNEE_OFFSET = math.pi - RIGHT_HIP_OFFSET

MATERIALS = {
    "rubber_black": "0.003 0.003 0.003 1",
    "rim_silver": "0.70 0.72 0.70 1",
    "spoke_steel": "0.82 0.84 0.82 1",
    "frame_blue": "0.05 0.28 0.75 1",
    "seat_brown": "0.28 0.12 0.05 1",
    "pelican_white": "0.96 0.95 0.88 1",
    "wing_gray": "0.66 0.68 0.64 1",
    "feather_dark": "0.08 0.08 0.075 1",
    "bill_yellow": "1.00 0.78 0.26 1",
    "pouch_orange": "0.95 0.45 0.12 1",
    "leg_orange": "0.98 0.50 0.12 1",
    "eye_black": "0.0 0.0 0.0 1",
}

Vector3 = tuple[float, float, float]


def _fmt(value: float) -> str:
    if abs(value) < 5e-10:
        value = 0.0
    return f"{value:.9g}"


def _vec(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(_fmt(float(value)) for value in values)


def _v_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_mul(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _v_dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _v_len(a: Vector3) -> float:
    return math.sqrt(_v_dot(a, a))


def _v_norm(a: Vector3) -> Vector3:
    length = _v_len(a)
    if length <= 1e-12:
        raise ValueError("zero-length vector")
    return (a[0] / length, a[1] / length, a[2] / length)


def _matrix_from_z_axis(direction: Vector3) -> tuple[tuple[float, float, float], ...]:
    z_axis = _v_norm(direction)
    helper = (0.0, 0.0, 1.0)
    if abs(_v_dot(z_axis, helper)) > 0.92:
        helper = (1.0, 0.0, 0.0)
    x_axis = _v_norm(_v_cross(helper, z_axis))
    y_axis = _v_cross(z_axis, x_axis)
    return (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )


def _rpy_from_matrix(matrix: tuple[tuple[float, float, float], ...]) -> Vector3:
    r00, r01, r02 = matrix[0]
    r10, r11, r12 = matrix[1]
    r20, r21, r22 = matrix[2]
    del r01, r02, r11, r12
    pitch = math.atan2(-r20, math.sqrt((r00 * r00) + (r10 * r10)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return (roll, pitch, yaw)


def _rpy_for_z_axis(direction: Vector3) -> Vector3:
    return _rpy_from_matrix(_matrix_from_z_axis(direction))


def _triangle_normal(triangle: tuple[Vector3, Vector3, Vector3]) -> Vector3:
    p1, p2, p3 = triangle
    normal = _v_cross(_v_sub(p2, p1), _v_sub(p3, p1))
    if _v_len(normal) <= 1e-12:
        return (0.0, 0.0, 1.0)
    return _v_norm(normal)


def _write_ascii_stl(path: Path, name: str, triangles: list[tuple[Vector3, Vector3, Vector3]]) -> None:
    lines = [f"solid {name}"]
    for triangle in triangles:
        normal = _triangle_normal(triangle)
        lines.append(f"  facet normal {_vec(normal)}")
        lines.append("    outer loop")
        for vertex in triangle:
            lines.append(f"      vertex {_vec(vertex)}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    text = "\n".join(lines) + "\n"
    if path.exists() and path.read_text(encoding="utf-8") == text:
        return
    path.write_text(text, encoding="utf-8")


def _box_triangles() -> list[tuple[Vector3, Vector3, Vector3]]:
    vertices = {
        "lll": (-0.5, -0.5, -0.5),
        "llh": (-0.5, -0.5, 0.5),
        "lhl": (-0.5, 0.5, -0.5),
        "lhh": (-0.5, 0.5, 0.5),
        "hll": (0.5, -0.5, -0.5),
        "hlh": (0.5, -0.5, 0.5),
        "hhl": (0.5, 0.5, -0.5),
        "hhh": (0.5, 0.5, 0.5),
    }
    faces = [
        ("lll", "hll", "hhl", "lhl"),
        ("llh", "lhh", "hhh", "hlh"),
        ("lll", "llh", "hlh", "hll"),
        ("lhl", "hhl", "hhh", "lhh"),
        ("lll", "lhl", "lhh", "llh"),
        ("hll", "hlh", "hhh", "hhl"),
    ]
    triangles: list[tuple[Vector3, Vector3, Vector3]] = []
    for a, b, c, d in faces:
        triangles.append((vertices[a], vertices[b], vertices[c]))
        triangles.append((vertices[a], vertices[c], vertices[d]))
    return triangles


def _cylinder_triangles(segments: int = 32) -> list[tuple[Vector3, Vector3, Vector3]]:
    triangles: list[tuple[Vector3, Vector3, Vector3]] = []
    top = (0.0, 0.0, 0.5)
    bottom = (0.0, 0.0, -0.5)
    for index in range(segments):
        a0 = math.tau * index / segments
        a1 = math.tau * (index + 1) / segments
        p0b = (0.5 * math.cos(a0), 0.5 * math.sin(a0), -0.5)
        p1b = (0.5 * math.cos(a1), 0.5 * math.sin(a1), -0.5)
        p0t = (p0b[0], p0b[1], 0.5)
        p1t = (p1b[0], p1b[1], 0.5)
        triangles.append((p0b, p1b, p1t))
        triangles.append((p0b, p1t, p0t))
        triangles.append((top, p0t, p1t))
        triangles.append((bottom, p1b, p0b))
    return triangles


def _cone_triangles(segments: int = 32) -> list[tuple[Vector3, Vector3, Vector3]]:
    triangles: list[tuple[Vector3, Vector3, Vector3]] = []
    tip = (0.0, 0.0, 0.5)
    base_center = (0.0, 0.0, -0.5)
    for index in range(segments):
        a0 = math.tau * index / segments
        a1 = math.tau * (index + 1) / segments
        p0 = (0.5 * math.cos(a0), 0.5 * math.sin(a0), -0.5)
        p1 = (0.5 * math.cos(a1), 0.5 * math.sin(a1), -0.5)
        triangles.append((p0, p1, tip))
        triangles.append((base_center, p0, p1))
    return triangles


def _sphere_triangles(segments: int = 32, rings: int = 16) -> list[tuple[Vector3, Vector3, Vector3]]:
    triangles: list[tuple[Vector3, Vector3, Vector3]] = []

    def point(theta: float, phi: float) -> Vector3:
        return (
            0.5 * math.sin(phi) * math.cos(theta),
            0.5 * math.sin(phi) * math.sin(theta),
            0.5 * math.cos(phi),
        )

    for ring in range(rings):
        phi0 = math.pi * ring / rings
        phi1 = math.pi * (ring + 1) / rings
        for segment in range(segments):
            theta0 = math.tau * segment / segments
            theta1 = math.tau * (segment + 1) / segments
            p00 = point(theta0, phi0)
            p01 = point(theta1, phi0)
            p10 = point(theta0, phi1)
            p11 = point(theta1, phi1)
            if ring == 0:
                triangles.append((p00, p11, p10))
            elif ring == rings - 1:
                triangles.append((p00, p01, p10))
            else:
                triangles.append((p00, p01, p11))
                triangles.append((p00, p11, p10))
    return triangles


def _torus_triangles(segments: int = 48, tube_segments: int = 14) -> list[tuple[Vector3, Vector3, Vector3]]:
    major_radius = 0.45
    minor_radius = 0.055
    triangles: list[tuple[Vector3, Vector3, Vector3]] = []

    def point(theta: float, phi: float) -> Vector3:
        radial = major_radius + minor_radius * math.cos(phi)
        return (
            radial * math.cos(theta),
            radial * math.sin(theta),
            minor_radius * math.sin(phi),
        )

    for segment in range(segments):
        theta0 = math.tau * segment / segments
        theta1 = math.tau * (segment + 1) / segments
        for tube in range(tube_segments):
            phi0 = math.tau * tube / tube_segments
            phi1 = math.tau * (tube + 1) / tube_segments
            p00 = point(theta0, phi0)
            p01 = point(theta0, phi1)
            p10 = point(theta1, phi0)
            p11 = point(theta1, phi1)
            triangles.append((p00, p10, p11))
            triangles.append((p00, p11, p01))
    return triangles


def _ensure_meshes() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    _write_ascii_stl(MESH_DIR / "unit_box.stl", "unit_box", _box_triangles())
    _write_ascii_stl(MESH_DIR / "unit_cylinder_z.stl", "unit_cylinder_z", _cylinder_triangles())
    _write_ascii_stl(MESH_DIR / "unit_cone_z.stl", "unit_cone_z", _cone_triangles())
    _write_ascii_stl(MESH_DIR / "unit_sphere.stl", "unit_sphere", _sphere_triangles())
    _write_ascii_stl(MESH_DIR / "unit_torus_z.stl", "unit_torus_z", _torus_triangles())


def _material_xml() -> str:
    return "\n".join(
        f'  <material name="{escape(name)}"><color rgba="{rgba}" /></material>'
        for name, rgba in MATERIALS.items()
    )


def _visual_mesh(
    name: str,
    mesh_name: str,
    material: str,
    *,
    xyz: Vector3 = (0.0, 0.0, 0.0),
    rpy: Vector3 = (0.0, 0.0, 0.0),
    scale: Vector3 = (1.0, 1.0, 1.0),
) -> str:
    return "\n".join(
        [
            f'    <visual name="{escape(name)}">',
            f'      <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}" />',
            "      <geometry>",
            f'        <mesh filename="meshes/{escape(mesh_name)}" scale="{_vec(scale)}" />',
            "      </geometry>",
            f'      <material name="{escape(material)}" />',
            "    </visual>",
        ]
    )


def _visual_sphere(name: str, material: str, xyz: Vector3, scale_xyz: Vector3) -> str:
    return _visual_mesh(name, "unit_sphere.stl", material, xyz=xyz, scale=scale_xyz)


def _visual_box(
    name: str,
    material: str,
    xyz: Vector3,
    size: Vector3,
    rpy: Vector3 = (0.0, 0.0, 0.0),
) -> str:
    return _visual_mesh(name, "unit_box.stl", material, xyz=xyz, rpy=rpy, scale=size)


def _visual_cylinder_between(name: str, material: str, p1: Vector3, p2: Vector3, diameter: float) -> str:
    vector = _v_sub(p2, p1)
    length = _v_len(vector)
    midpoint = _v_mul(_v_add(p1, p2), 0.5)
    return _visual_mesh(
        name,
        "unit_cylinder_z.stl",
        material,
        xyz=midpoint,
        rpy=_rpy_for_z_axis(vector),
        scale=(diameter, diameter, length),
    )


def _visual_cone_between(
    name: str,
    material: str,
    base: Vector3,
    tip: Vector3,
    base_diameter: float,
    *,
    flatten_y: float = 1.0,
) -> str:
    vector = _v_sub(tip, base)
    length = _v_len(vector)
    midpoint = _v_mul(_v_add(base, tip), 0.5)
    return _visual_mesh(
        name,
        "unit_cone_z.stl",
        material,
        xyz=midpoint,
        rpy=_rpy_for_z_axis(vector),
        scale=(base_diameter, base_diameter * flatten_y, length),
    )


def _collision_box(size: Vector3, xyz: Vector3 = (0.0, 0.0, 0.0), rpy: Vector3 = (0.0, 0.0, 0.0)) -> str:
    return "\n".join(
        [
            "    <collision>",
            f'      <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}" />',
            f'      <geometry><box size="{_vec(size)}" /></geometry>',
            "    </collision>",
        ]
    )


def _collision_cylinder(
    radius: float,
    length: float,
    xyz: Vector3 = (0.0, 0.0, 0.0),
    rpy: Vector3 = (0.0, 0.0, 0.0),
) -> str:
    return "\n".join(
        [
            "    <collision>",
            f'      <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}" />',
            f'      <geometry><cylinder radius="{_fmt(radius)}" length="{_fmt(length)}" /></geometry>',
            "    </collision>",
        ]
    )


def _collision_sphere(radius: float, xyz: Vector3 = (0.0, 0.0, 0.0)) -> str:
    return "\n".join(
        [
            "    <collision>",
            f'      <origin xyz="{_vec(xyz)}" rpy="0 0 0" />',
            f'      <geometry><sphere radius="{_fmt(radius)}" /></geometry>',
            "    </collision>",
        ]
    )


def _inertial_xml(mass: float) -> str:
    inertia = max(mass * 1.0e-4, 1.0e-7)
    return "\n".join(
        [
            "    <inertial>",
            '      <origin xyz="0 0 0" rpy="0 0 0" />',
            f'      <mass value="{_fmt(mass)}" />',
            (
                f'      <inertia ixx="{_fmt(inertia)}" ixy="0" ixz="0" '
                f'iyy="{_fmt(inertia)}" iyz="0" izz="{_fmt(inertia)}" />'
            ),
            "    </inertial>",
        ]
    )


def _link_xml(name: str, visuals: list[str], collision: str | None, mass: float | None) -> str:
    body: list[str] = [f'  <link name="{escape(name)}">']
    if mass is not None:
        body.append(_inertial_xml(mass))
    body.extend(visuals)
    if collision is not None:
        body.append(collision)
    body.append("  </link>")
    return "\n".join(body)


def _joint_xml(
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    *,
    xyz: Vector3 = (0.0, 0.0, 0.0),
    rpy: Vector3 = (0.0, 0.0, 0.0),
    axis: Vector3 | None = None,
    mimic: tuple[str, float, float] | None = None,
    default_deg: float | None = None,
    limit: tuple[float, float] | None = None,
) -> str:
    default_attr = "" if default_deg is None else f' default_deg="{_fmt(default_deg)}"'
    lines = [
        f'  <joint name="{escape(name)}" type="{escape(joint_type)}"{default_attr}>',
        f'    <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}" />',
        f'    <parent link="{escape(parent)}" />',
        f'    <child link="{escape(child)}" />',
    ]
    if joint_type != "fixed":
        lines.append(f'    <axis xyz="{_vec(axis or (0.0, 1.0, 0.0))}" />')
    if limit is not None:
        lower, upper = limit
        lines.append(
            f'    <limit lower="{_fmt(lower)}" upper="{_fmt(upper)}" effort="1" velocity="6.28318531" />'
        )
    if mimic is not None:
        driver, multiplier, offset = mimic
        lines.append(
            f'    <mimic joint="{escape(driver)}" multiplier="{_fmt(multiplier)}" offset="{_fmt(offset)}" />'
        )
    lines.append("  </joint>")
    return "\n".join(lines)


def _bike_frame_visuals() -> list[str]:
    rear = (-0.055, 0.0, 0.044)
    front = (0.055, 0.0, 0.044)
    bottom = (0.0, 0.0, 0.049)
    seat = (-0.015, 0.0, 0.086)
    head_low = (0.043, 0.0, 0.066)
    head_high = (0.038, 0.0, 0.088)
    handle = (0.064, 0.0, 0.101)
    visuals = [
        _visual_cylinder_between("seat_tube", "frame_blue", bottom, seat, 0.0046),
        _visual_cylinder_between("top_tube", "frame_blue", seat, head_high, 0.0042),
        _visual_cylinder_between("down_tube", "frame_blue", bottom, head_low, 0.0050),
        _visual_cylinder_between("head_tube", "frame_blue", head_low, head_high, 0.0052),
        _visual_cylinder_between("left_chain_stay", "frame_blue", (bottom[0], -0.004, bottom[2]), (rear[0], -0.004, rear[2]), 0.0032),
        _visual_cylinder_between("right_chain_stay", "frame_blue", (bottom[0], 0.004, bottom[2]), (rear[0], 0.004, rear[2]), 0.0032),
        _visual_cylinder_between("left_seat_stay", "frame_blue", (seat[0], -0.004, seat[2]), (rear[0], -0.004, rear[2]), 0.0030),
        _visual_cylinder_between("right_seat_stay", "frame_blue", (seat[0], 0.004, seat[2]), (rear[0], 0.004, rear[2]), 0.0030),
        _visual_cylinder_between("front_left_fork_blade", "frame_blue", (head_low[0], -0.004, head_low[2]), (front[0], -0.004, front[2]), 0.0032),
        _visual_cylinder_between("front_right_fork_blade", "frame_blue", (head_low[0], 0.004, head_low[2]), (front[0], 0.004, front[2]), 0.0032),
        _visual_cylinder_between("handlebar_stem", "frame_blue", head_high, handle, 0.0034),
        _visual_cylinder_between("handlebar_crossbar", "frame_blue", (0.064, -0.022, 0.101), (0.064, 0.022, 0.101), 0.0032),
        _visual_box("brown_saddle", "seat_brown", (-0.020, 0.0, 0.097), (0.024, 0.017, 0.005)),
        _visual_box("left_black_grip", "rubber_black", (0.064, -0.024, 0.101), (0.008, 0.006, 0.006)),
        _visual_box("right_black_grip", "rubber_black", (0.064, 0.024, 0.101), (0.008, 0.006, 0.006)),
    ]
    return visuals


def _wheel_visuals(name_prefix: str) -> list[str]:
    visuals = [
        _visual_mesh(
            f"{name_prefix}_black_tire",
            "unit_torus_z.stl",
            "rubber_black",
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            scale=(0.088, 0.088, 0.088),
        ),
        _visual_mesh(
            f"{name_prefix}_silver_rim",
            "unit_torus_z.stl",
            "rim_silver",
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            scale=(0.066, 0.066, 0.066),
        ),
        _visual_cylinder_between(f"{name_prefix}_hub", "rim_silver", (0.0, -0.008, 0.0), (0.0, 0.008, 0.0), 0.008),
    ]
    spoke_radius = 0.030
    for index in range(16):
        angle = math.tau * index / 16.0
        rim_point = (math.cos(angle) * spoke_radius, 0.0, math.sin(angle) * spoke_radius)
        hub_point = (0.0, -0.0015 if index % 2 else 0.0015, 0.0)
        visuals.append(_visual_cylinder_between(f"{name_prefix}_spoke_{index + 1:02d}", "spoke_steel", hub_point, rim_point, 0.0010))
    return visuals


def _crank_visuals(name_prefix: str, side_sign: float) -> list[str]:
    crank_radius = PEDAL_CRANK_RADIUS
    pedal_center = (0.0, side_sign * 0.012, -crank_radius)
    return [
        _visual_cylinder_between(f"{name_prefix}_crank_arm", "spoke_steel", (0.0, 0.0, 0.0), pedal_center, 0.0024),
        _visual_box(f"{name_prefix}_pedal_pad", "rubber_black", pedal_center, (0.017, 0.004, 0.004)),
        _visual_cylinder_between(f"{name_prefix}_pedal_axle", "spoke_steel", (0.0, side_sign * 0.005, -crank_radius), (0.0, side_sign * 0.017, -crank_radius), 0.0015),
    ]


def _pelican_body_visuals() -> list[str]:
    return [
        _visual_sphere("round_body", "pelican_white", (0.0, 0.0, 0.0), (0.048, 0.030, 0.058)),
        _visual_sphere("chest_belly", "pelican_white", (0.013, 0.0, -0.008), (0.036, 0.026, 0.040)),
        _visual_sphere("left_wing", "wing_gray", (-0.010, -0.014, 0.000), (0.036, 0.006, 0.050)),
        _visual_sphere("right_wing", "wing_gray", (-0.010, 0.014, 0.000), (0.036, 0.006, 0.050)),
        _visual_sphere("left_wing_tip", "feather_dark", (-0.023, -0.017, -0.016), (0.020, 0.004, 0.018)),
        _visual_sphere("right_wing_tip", "feather_dark", (-0.023, 0.017, -0.016), (0.020, 0.004, 0.018)),
        _visual_cylinder_between("lower_neck", "pelican_white", (0.006, 0.0, 0.022), (0.024, 0.0, 0.037), 0.010),
        _visual_sphere("head", "pelican_white", (0.040, 0.0, 0.041), (0.022, 0.017, 0.020)),
        _visual_cone_between("upper_bill", "bill_yellow", (0.049, 0.0, 0.041), (0.087, 0.0, 0.041), 0.010, flatten_y=1.45),
        _visual_sphere("lower_bill_pouch", "pouch_orange", (0.064, 0.0, 0.032), (0.046, 0.014, 0.014)),
        _visual_sphere("left_eye", "eye_black", (0.043, -0.008, 0.045), (0.003, 0.0015, 0.003)),
        _visual_sphere("right_eye", "eye_black", (0.043, 0.008, 0.045), (0.003, 0.0015, 0.003)),
        _visual_sphere("head_crest", "pelican_white", (0.032, 0.0, 0.052), (0.008, 0.008, 0.010)),
        _visual_sphere("short_tail", "feather_dark", (-0.027, 0.0, -0.012), (0.020, 0.012, 0.010)),
    ]


def _leg_segment_visuals(name_prefix: str, length: float, side_sign: float, include_foot: bool = False) -> list[str]:
    visuals = [
        _visual_cylinder_between(f"{name_prefix}_orange_segment", "leg_orange", (0.0, 0.0, 0.0), (0.0, 0.0, -length), 0.0040)
    ]
    if include_foot:
        visuals.append(
            _visual_sphere(
                f"{name_prefix}_webbed_foot",
                "leg_orange",
                (0.0, side_sign * 0.006, -length),
                (0.017, 0.007, 0.004),
            )
        )
    return visuals


def _robot_links() -> list[str]:
    return [
        _link_xml("world", [], None, None),
        _link_xml(
            "bike_frame_link",
            _bike_frame_visuals(),
            _collision_box((0.14, 0.06, 0.09), (0.0, 0.0, 0.065)),
            0.7,
        ),
        _link_xml(
            "rear_wheel_link",
            _wheel_visuals("rear_wheel"),
            _collision_cylinder(0.044, 0.012, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            0.18,
        ),
        _link_xml(
            "front_wheel_link",
            _wheel_visuals("front_wheel"),
            _collision_cylinder(0.044, 0.012, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            0.18,
        ),
        _link_xml(
            "pedal_cycle_link",
            _crank_visuals("left", -1.0),
            _collision_box((0.020, 0.030, 0.034), (0.0, -0.006, -0.014)),
            0.04,
        ),
        _link_xml(
            "opposite_crank_link",
            _crank_visuals("right", 1.0),
            _collision_box((0.020, 0.030, 0.034), (0.0, 0.006, -0.014)),
            0.04,
        ),
        _link_xml(
            "pelican_body_link",
            _pelican_body_visuals(),
            _collision_sphere(0.030),
            0.45,
        ),
        _link_xml(
            "left_thigh_link",
            _leg_segment_visuals("left_thigh", LEFT_THIGH_LENGTH, -1.0),
            _collision_cylinder(0.0025, LEFT_THIGH_LENGTH, (0.0, 0.0, -LEFT_THIGH_LENGTH / 2.0)),
            0.025,
        ),
        _link_xml(
            "left_shank_link",
            _leg_segment_visuals("left_shank", LEFT_SHANK_LENGTH, -1.0, include_foot=True),
            _collision_cylinder(0.0024, LEFT_SHANK_LENGTH, (0.0, 0.0, -LEFT_SHANK_LENGTH / 2.0)),
            0.025,
        ),
        _link_xml(
            "right_thigh_link",
            _leg_segment_visuals("right_thigh", RIGHT_THIGH_LENGTH, 1.0),
            _collision_cylinder(0.0025, RIGHT_THIGH_LENGTH, (0.0, 0.0, -RIGHT_THIGH_LENGTH / 2.0)),
            0.025,
        ),
        _link_xml(
            "right_shank_link",
            _leg_segment_visuals("right_shank", RIGHT_SHANK_LENGTH, 1.0, include_foot=True),
            _collision_cylinder(0.0024, RIGHT_SHANK_LENGTH, (0.0, 0.0, -RIGHT_SHANK_LENGTH / 2.0)),
            0.025,
        ),
    ]


def _robot_joints() -> list[str]:
    driver = "pedal_cycle_joint"
    return [
        _joint_xml("world_to_bike_frame", "fixed", "world", "bike_frame_link"),
        _joint_xml(
            "rear_wheel_spin_joint",
            "continuous",
            "bike_frame_link",
            "rear_wheel_link",
            xyz=(-0.055, 0.0, 0.044),
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, -3.2, 0.0),
        ),
        _joint_xml(
            "front_wheel_spin_joint",
            "continuous",
            "bike_frame_link",
            "front_wheel_link",
            xyz=(0.055, 0.0, 0.044),
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, -3.2, 0.0),
        ),
        _joint_xml(
            driver,
            "continuous",
            "bike_frame_link",
            "pedal_cycle_link",
            xyz=BIKE_BOTTOM_BRACKET,
            axis=(0.0, 1.0, 0.0),
            default_deg=25.0,
        ),
        _joint_xml(
            "opposite_crank_mimic_joint",
            "continuous",
            "bike_frame_link",
            "opposite_crank_link",
            xyz=BIKE_BOTTOM_BRACKET,
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, 1.0, math.pi),
        ),
        _joint_xml(
            "pelican_body_fixed_joint",
            "fixed",
            "bike_frame_link",
            "pelican_body_link",
            xyz=PELICAN_BODY_ORIGIN,
        ),
        _joint_xml(
            "left_hip_pedal_mimic_joint",
            "continuous",
            "pelican_body_link",
            "left_thigh_link",
            xyz=LEFT_HIP_ORIGIN,
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, 0.0, LEFT_HIP_OFFSET),
        ),
        _joint_xml(
            "left_knee_pedal_mimic_joint",
            "continuous",
            "left_thigh_link",
            "left_shank_link",
            xyz=(0.0, 0.0, -LEFT_THIGH_LENGTH),
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, 1.0, LEFT_KNEE_OFFSET),
        ),
        _joint_xml(
            "right_hip_pedal_mimic_joint",
            "continuous",
            "pelican_body_link",
            "right_thigh_link",
            xyz=RIGHT_HIP_ORIGIN,
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, 0.0, RIGHT_HIP_OFFSET),
        ),
        _joint_xml(
            "right_knee_pedal_mimic_joint",
            "continuous",
            "right_thigh_link",
            "right_shank_link",
            xyz=(0.0, 0.0, -RIGHT_THIGH_LENGTH),
            axis=(0.0, 1.0, 0.0),
            mimic=(driver, 1.0, RIGHT_KNEE_OFFSET),
        ),
    ]


def _explorer_metadata() -> dict[str, object]:
    return {
        "schemaVersion": 3,
        "kind": "texttocad-urdf-explorer",
        "jointDefaultsByName": {
            "pedal_cycle_joint": 25,
        },
        "poses": [
            {"name": "pedal stroke 0 deg", "jointValuesByName": {"pedal_cycle_joint": 0}},
            {"name": "pedal stroke 90 deg", "jointValuesByName": {"pedal_cycle_joint": 90}},
            {"name": "pedal stroke 180 deg", "jointValuesByName": {"pedal_cycle_joint": 180}},
            {"name": "pedal stroke 270 deg", "jointValuesByName": {"pedal_cycle_joint": 270}},
        ],
    }


def gen_urdf() -> dict[str, object]:
    _ensure_meshes()
    xml = "\n".join(
        [
            '<?xml version="1.0" ?>',
            f'<robot name="{ROBOT_NAME}">',
            _material_xml(),
            *_robot_links(),
            *_robot_joints(),
            "</robot>",
            "",
        ]
    )
    return {
        "xml": xml,
        "explorer_metadata": _explorer_metadata(),
    }

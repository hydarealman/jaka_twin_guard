#!/usr/bin/env python3
"""Extract and validate a mate-aware SolidWorks robot graph.

The extraction backend is the open-source ``sw2robot`` project.  Its extract
phase must run on Windows with SolidWorks installed because it reads mates,
component transforms, named coordinate systems and reference axes through the
SolidWorks COM API.  Validation is dependency-free and can run on any machine
after ``graph.json`` has been copied here.

This tool intentionally stops before replacing the production URDF.  A graph
is accepted only when it came from the requested mechanical-zero configuration,
contains all eight datum frames and has enough finite, unit-length mate axes to
describe a six-axis arm.  The generated graph is then an auditable input to the
MoveIt integration instead of another set of hand-entered ``xyz/rpy`` values.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree


MECHANICAL_CONFIGURATION = "初始位置"
REQUIRED_FRAMES = (
    "Origin_global",
    "J1坐标系",
    "J2坐标系",
    "J3坐标系",
    "J4坐标系",
    "J5坐标系",
    "J6坐标系",
)
TCP_FRAME_ALIASES = ("TCP坐标系", "TCP_URDF")

# The signed ROS/URDF ranges agreed with the mechanical team on 2026-08-27.
# A joint axis carries the positive direction.  Limits must always satisfy
# lower <= upper; reversing those XML attributes is not a way to reverse an
# axis in URDF.
JOINT_LIMITS = {
    "J1": (-math.radians(120.0), math.radians(120.0)),
    "J2": (0.0, math.radians(145.0)),
    "J3": (-math.pi, 0.0),
    "J4": (-math.radians(165.0), math.radians(165.0)),
    "J5": (-math.pi / 2.0, math.pi / 2.0),
    "J6": (-math.pi, math.pi),
}


class GraphValidationError(RuntimeError):
    """The extracted graph cannot be trusted as a robot kinematic source."""


def _finite_vector(value: object, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _matrix_is_rigid(value: object, tolerance: float = 1e-5) -> bool:
    matrix = _finite_vector(value, 16)
    if matrix is None:
        return False
    if any(abs(matrix[12 + index] - expected) > tolerance
           for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        return False
    rows = [matrix[0:3], matrix[4:7], matrix[8:11]]
    for row in rows:
        if abs(sum(item * item for item in row) - 1.0) > tolerance:
            return False
    for first, second in ((0, 1), (0, 2), (1, 2)):
        if abs(sum(rows[first][i] * rows[second][i] for i in range(3))) > tolerance:
            return False
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    return abs(determinant - 1.0) <= tolerance


def validate_graph(graph_path: Path, expected_configuration: str) -> dict:
    """Validate provenance, datums and numerical integrity of ``graph.json``."""
    try:
        graph = json.loads(graph_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise GraphValidationError(f"cannot read graph JSON: {error}") from error

    errors: list[str] = []
    if graph.get("configuration") != expected_configuration:
        errors.append(
            "top-level configuration is "
            f"{graph.get('configuration')!r}, expected {expected_configuration!r}"
        )

    components = graph.get("components")
    if not isinstance(components, list) or not components:
        errors.append("no SolidWorks components were extracted")
        components = []
    bad_components = [
        str(component.get("name", "<unnamed>"))
        for component in components
        if not isinstance(component, dict)
        or not _matrix_is_rigid(component.get("world"))
    ]
    if bad_components:
        errors.append(
            "invalid/non-rigid component world matrices: "
            + ", ".join(bad_components[:8])
        )

    coordinate_systems = graph.get("coordinate_systems")
    if not isinstance(coordinate_systems, list):
        coordinate_systems = []
    frames = {
        item.get("name"): item
        for item in coordinate_systems
        if isinstance(item, dict) and isinstance(item.get("name"), str)
    }
    missing_frames = [name for name in REQUIRED_FRAMES if name not in frames]
    if missing_frames:
        errors.append("missing named SolidWorks frames: " + ", ".join(missing_frames))
    if not any(name in frames for name in TCP_FRAME_ALIASES):
        errors.append(
            "missing tool-center-point frame (expected one of: "
            + ", ".join(TCP_FRAME_ALIASES)
            + ")"
        )
    invalid_frames = [
        name for name, item in frames.items()
        if name in REQUIRED_FRAMES
        and not _matrix_is_rigid(item.get("document_from_frame"))
    ]
    if invalid_frames:
        errors.append("invalid datum transforms: " + ", ".join(invalid_frames))

    axis_records: list[tuple[str, str, list[float], list[float]]] = []
    for edge in graph.get("edges") or []:
        if not isinstance(edge, dict):
            continue
        point = _finite_vector(edge.get("axis_point"), 3)
        direction = _finite_vector(edge.get("axis_dir"), 3)
        if point is None or direction is None:
            continue
        norm = math.sqrt(sum(item * item for item in direction))
        if abs(norm - 1.0) <= 1e-5:
            axis_records.append((
                str(edge.get("a", "?")), str(edge.get("b", "?")),
                point, direction,
            ))
    if len(axis_records) < 6:
        errors.append(
            f"only {len(axis_records)} finite unit mate axes were extracted; "
            "a six-axis arm needs at least 6"
        )

    if errors:
        raise GraphValidationError("\n- ".join(["graph validation failed", *errors]))

    return {
        "graph": str(graph_path.resolve()),
        "robot_name": graph.get("robot_name"),
        "source_assembly": graph.get("source_assembly"),
        "configuration": graph.get("configuration"),
        "components": len(components),
        "mate_edges": len(graph.get("edges") or []),
        "unit_mate_axes": len(axis_records),
        "required_frames": list(REQUIRED_FRAMES),
        "tcp_frame_aliases": list(TCP_FRAME_ALIASES),
    }


def _vector_attribute(element: ElementTree.Element, attribute: str, length: int) -> list[float] | None:
    raw = element.get(attribute)
    if raw is None:
        return None
    return _finite_vector(raw.split(), length)


def _resolve_package_mesh(urdf_path: Path, filename: str) -> Path | None:
    if filename.startswith("package://"):
        package_relative = filename[len("package://"):]
        if "/" not in package_relative:
            return None
        _, relative = package_relative.split("/", 1)
        return urdf_path.parent.parent / Path(relative)
    mesh_path = Path(filename)
    return mesh_path if mesh_path.is_absolute() else urdf_path.parent / mesh_path


def _stl_triangle_count(path: Path) -> int | None:
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) >= 84:
        count = int.from_bytes(data[80:84], "little")
        if len(data) == 84 + 50 * count:
            return count
    try:
        text = data.decode("ascii", errors="strict").lower()
    except UnicodeDecodeError:
        return None
    count = text.count("facet normal")
    return count if text.lstrip().startswith("solid") else None


def audit_classic_urdf(urdf_path: Path, limit_tolerance: float = 1e-3) -> dict:
    """Audit a package emitted by the classic SolidWorks URDF Exporter.

    ``check_urdf`` accepts reversed limit attributes and empty binary STL files,
    so both conditions need explicit checks before a CAD export may enter the
    production description package.
    """
    try:
        root = ElementTree.parse(urdf_path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise GraphValidationError(f"cannot read URDF XML: {error}") from error

    errors: list[str] = []
    joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
    links = {link.get("name", ""): link for link in root.findall("link")}
    missing_joints = [name for name in JOINT_LIMITS if name not in joints]
    if missing_joints:
        errors.append("missing joints: " + ", ".join(missing_joints))

    parent_by_child: dict[str, str] = {}
    joint_report: dict[str, dict[str, object]] = {}
    for name, expected_limits in JOINT_LIMITS.items():
        joint = joints.get(name)
        if joint is None:
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        parent_name = parent.get("link") if parent is not None else None
        child_name = child.get("link") if child is not None else None
        if not parent_name or parent_name not in links:
            errors.append(f"{name}: missing/unknown parent link {parent_name!r}")
        if not child_name or child_name not in links:
            errors.append(f"{name}: missing/unknown child link {child_name!r}")
        elif child_name in parent_by_child:
            errors.append(f"{name}: child link {child_name!r} has more than one parent")
        else:
            parent_by_child[child_name] = parent_name or ""

        xyz = _vector_attribute(origin, "xyz", 3) if origin is not None else None
        rpy = _vector_attribute(origin, "rpy", 3) if origin is not None else None
        direction = _vector_attribute(axis, "xyz", 3) if axis is not None else None
        if xyz is None or rpy is None:
            errors.append(f"{name}: origin xyz/rpy is missing or non-finite")
        if direction is None:
            errors.append(f"{name}: axis is missing or non-finite")
            axis_norm = None
        else:
            axis_norm = math.sqrt(sum(item * item for item in direction))
            if abs(axis_norm - 1.0) > 1e-5:
                errors.append(f"{name}: axis norm is {axis_norm:.9g}, expected 1")

        actual_limits: tuple[float, float] | None = None
        try:
            if limit is None:
                raise ValueError
            actual_limits = (float(limit.get("lower", "")), float(limit.get("upper", "")))
            if not all(math.isfinite(value) for value in actual_limits):
                raise ValueError
        except ValueError:
            errors.append(f"{name}: lower/upper limit is missing or non-finite")
        if actual_limits is not None:
            lower, upper = actual_limits
            if lower > upper:
                errors.append(f"{name}: invalid reversed limits [{lower:.9g}, {upper:.9g}]")
            expected_lower, expected_upper = expected_limits
            if (
                abs(lower - expected_lower) > limit_tolerance
                or abs(upper - expected_upper) > limit_tolerance
            ):
                errors.append(
                    f"{name}: limits [{lower:.9g}, {upper:.9g}] do not match "
                    f"mechanical contract [{expected_lower:.9g}, {expected_upper:.9g}]"
                )
        joint_report[name] = {
            "parent": parent_name,
            "child": child_name,
            "origin_xyz": xyz,
            "origin_rpy": rpy,
            "axis": direction,
            "axis_norm": axis_norm,
            "limits": actual_limits,
        }

    roots = sorted(name for name in links if name and name not in parent_by_child)
    if len(roots) != 1:
        errors.append(f"expected one root link, found {len(roots)}: {', '.join(roots)}")

    meshes: dict[str, dict[str, object]] = {}
    for link_name, link in links.items():
        visual_mesh = link.find("visual/geometry/mesh")
        if visual_mesh is None or not visual_mesh.get("filename"):
            errors.append(f"{link_name}: visual mesh is missing")
            continue
        filename = visual_mesh.get("filename", "")
        mesh_path = _resolve_package_mesh(urdf_path, filename)
        if mesh_path is None or not mesh_path.is_file():
            errors.append(f"{link_name}: mesh does not exist: {filename}")
            continue
        triangle_count = _stl_triangle_count(mesh_path)
        if triangle_count is None:
            errors.append(f"{link_name}: mesh is not a readable STL: {mesh_path}")
        elif triangle_count == 0:
            errors.append(f"{link_name}: mesh contains zero triangles: {mesh_path.name}")
        meshes[link_name] = {
            "path": str(mesh_path.resolve()),
            "bytes": mesh_path.stat().st_size,
            "triangles": triangle_count,
        }

    if errors:
        raise GraphValidationError("\n- ".join(["URDF package audit failed", *errors]))
    return {
        "urdf": str(urdf_path.resolve()),
        "robot_name": root.get("name"),
        "root_link": roots[0],
        "links": len(links),
        "joints": joint_report,
        "meshes": meshes,
    }


def extract_graph(
    assembly: Path,
    output: Path,
    robot_name: str,
    configuration: str,
    visible: bool,
) -> Path:
    if os.name != "nt":
        raise RuntimeError("sw2robot extraction requires Windows + SolidWorks")
    if assembly.suffix.lower() != ".sldasm":
        raise RuntimeError(f"expected a .SLDASM, got: {assembly}")
    if not assembly.is_file():
        raise RuntimeError(f"assembly does not exist: {assembly}")
    if importlib.util.find_spec("win32com") is None:
        raise RuntimeError("pywin32 is not installed in this Python environment")
    if importlib.util.find_spec("sw2robot") is None:
        raise RuntimeError(
            "sw2robot is not installed; install "
            "https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2"
        )

    command = [
        sys.executable,
        "-m",
        "sw2robot.exporter.extract",
        str(assembly.resolve()),
        "-o",
        str(output.resolve()),
        "-n",
        robot_name,
        "--configuration",
        configuration,
    ]
    if visible:
        command.append("--visible")
    subprocess.run(command, check=True)
    graph_path = output.resolve() / robot_name / "graph.json"
    validate_graph(graph_path, configuration)
    return graph_path


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("graph", type=Path)
    validate_parser.add_argument("--configuration", default=MECHANICAL_CONFIGURATION)

    audit_parser = subparsers.add_parser("audit-urdf")
    audit_parser.add_argument("urdf", type=Path)
    audit_parser.add_argument("--limit-tolerance", type=float, default=1e-3)

    extract_parser = subparsers.add_parser("extract")
    extract_parser.add_argument("assembly", type=Path)
    extract_parser.add_argument("output", type=Path)
    extract_parser.add_argument("--name", default="fruit_arm")
    extract_parser.add_argument("--configuration", default=MECHANICAL_CONFIGURATION)
    extract_parser.add_argument("--visible", action="store_true")

    args = parser.parse_args()
    try:
        if args.command == "validate":
            report = validate_graph(args.graph, args.configuration)
        elif args.command == "audit-urdf":
            report = audit_classic_urdf(args.urdf, args.limit_tolerance)
        else:
            graph_path = extract_graph(
                args.assembly,
                args.output,
                args.name,
                args.configuration,
                args.visible,
            )
            report = validate_graph(graph_path, args.configuration)
    except (GraphValidationError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(2, f"ERROR: {error}\n")

    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

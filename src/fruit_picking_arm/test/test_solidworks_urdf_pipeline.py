#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / "src" / "fruit_picking_arm" / "tools" / "solidworks_urdf_pipeline.py"
SPEC = importlib.util.spec_from_file_location("solidworks_urdf_pipeline", TOOL)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def valid_graph() -> dict:
    return {
        "robot_name": "fruit_arm",
        "source_assembly": "mechanical_zero.SLDASM",
        "configuration": "初始位置",
        "components": [{"name": "base", "world": IDENTITY}],
        "coordinate_systems": [
            {"name": name, "document_from_frame": IDENTITY}
            for name in MODULE.REQUIRED_FRAMES
        ] + [{"name": "TCP坐标系", "document_from_frame": IDENTITY}],
        "edges": [
            {
                "a": f"link_{index}",
                "b": f"link_{index + 1}",
                "axis_point": [float(index), 0.0, 0.0],
                "axis_dir": [0.0, 0.0, 1.0],
            }
            for index in range(6)
        ],
    }


def write_graph(tmp_path: Path, graph: dict) -> Path:
    path = tmp_path / "graph.json"
    path.write_text(json.dumps(graph, ensure_ascii=False), encoding="utf-8")
    return path


def test_accepts_mechanical_zero_graph_with_all_datums(tmp_path: Path) -> None:
    report = MODULE.validate_graph(write_graph(tmp_path, valid_graph()), "初始位置")
    assert report["unit_mate_axes"] == 6
    assert report["required_frames"] == list(MODULE.REQUIRED_FRAMES)


def test_rejects_default_assembly_configuration(tmp_path: Path) -> None:
    graph = valid_graph()
    graph["configuration"] = "默认"
    with pytest.raises(MODULE.GraphValidationError, match="expected '初始位置'"):
        MODULE.validate_graph(write_graph(tmp_path, graph), "初始位置")


def test_rejects_missing_joint_frame(tmp_path: Path) -> None:
    graph = valid_graph()
    graph["coordinate_systems"] = [
        frame for frame in graph["coordinate_systems"]
        if frame["name"] != "J6坐标系"
    ]
    with pytest.raises(MODULE.GraphValidationError, match="J6坐标系"):
        MODULE.validate_graph(write_graph(tmp_path, graph), "初始位置")


def test_rejects_missing_tcp_frame(tmp_path: Path) -> None:
    graph = valid_graph()
    graph["coordinate_systems"] = [
        frame for frame in graph["coordinate_systems"]
        if frame["name"] != "TCP坐标系"
    ]
    with pytest.raises(MODULE.GraphValidationError, match="tool-center-point"):
        MODULE.validate_graph(write_graph(tmp_path, graph), "初始位置")


def test_rejects_non_unit_or_insufficient_axes(tmp_path: Path) -> None:
    graph = valid_graph()
    graph["edges"][0]["axis_dir"] = [0.0, 0.0, 2.0]
    with pytest.raises(MODULE.GraphValidationError, match="only 5"):
        MODULE.validate_graph(write_graph(tmp_path, graph), "初始位置")


def write_binary_stl(path: Path, triangles: int) -> None:
    path.write_bytes(bytes(80) + triangles.to_bytes(4, "little") + bytes(50 * triangles))


def write_urdf_package(tmp_path: Path, *, empty_link3: bool = False, reverse_j1: bool = False) -> Path:
    package = tmp_path / "robot"
    meshes = package / "meshes"
    urdf_dir = package / "urdf"
    meshes.mkdir(parents=True)
    urdf_dir.mkdir()
    links = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
    for index, link in enumerate(links):
        write_binary_stl(meshes / f"{link}.STL", 0 if empty_link3 and link == "Link3" else 1)
    link_xml = "\n".join(
        f'''<link name="{link}"><visual><geometry><mesh filename="package://robot/meshes/{link}.STL"/></geometry></visual></link>'''
        for link in links
    )
    joint_xml = []
    for index, (name, limits) in enumerate(MODULE.JOINT_LIMITS.items(), start=1):
        lower, upper = limits
        if name == "J1" and reverse_j1:
            lower, upper = upper, lower
        joint_xml.append(
            f'''<joint name="{name}" type="revolute"><origin xyz="0 0 0" rpy="0 0 0"/>'''
            f'''<parent link="{links[index - 1]}"/><child link="{links[index]}"/>'''
            f'''<axis xyz="0 0 1"/><limit lower="{lower}" upper="{upper}" effort="1" velocity="1"/></joint>'''
        )
    urdf = urdf_dir / "robot.urdf"
    urdf.write_text(
        '<robot name="robot">' + link_xml + "\n".join(joint_xml) + '</robot>',
        encoding="utf-8",
    )
    return urdf


def test_audit_accepts_complete_classic_export(tmp_path: Path) -> None:
    report = MODULE.audit_classic_urdf(write_urdf_package(tmp_path))
    assert report["links"] == 7
    assert report["meshes"]["Link3"]["triangles"] == 1


def test_audit_rejects_empty_stl(tmp_path: Path) -> None:
    with pytest.raises(MODULE.GraphValidationError, match="zero triangles"):
        MODULE.audit_classic_urdf(write_urdf_package(tmp_path, empty_link3=True))


def test_audit_rejects_reversed_limits(tmp_path: Path) -> None:
    with pytest.raises(MODULE.GraphValidationError, match="reversed limits"):
        MODULE.audit_classic_urdf(write_urdf_package(tmp_path, reverse_j1=True))

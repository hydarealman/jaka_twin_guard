#!/usr/bin/env python3
"""Convert the supplied robot STEP assembly into URDF-ready link meshes.

The source assembly is AP203 and does not contain robot joints.  This tool uses
the XCAF assembly hierarchy and component names to split the drawing into the
seven rigid bodies used by the ROS model.  Joint centres/axes are documented in
``docs/CUSTOM_ARM_CAD_MODEL.md`` and intentionally kept out of this converter.

The source drawing uses millimetres.  Generated STL files also use millimetres;
the URDF applies ``scale=0.001``.  Each mesh is expressed in its own URDF link
frame so that the drawing pose is the zero-joint pose.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound, TopoDS_Shape
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.gp import gp_Trsf


# Link-frame origins expressed in the converted ROS-oriented CAD coordinates,
# in millimetres.  Link_00 is located at the floor directly below joint 1.
LINK_ORIGINS_MM = {
    "Link_00": (0.0, 0.0, 0.0),
    "Link_01": (0.0, 0.0, 40.975),
    "Link_02": (-48.101, 56.781, 135.000),
    "Link_03": (351.311, 41.000, 319.581),
    "Link_04": (309.856, 8.200, 377.022),
    "Link_05": (-62.318, -26.250, 372.132),
    "Link_06": (-89.201, 28.700, 353.175),
    "gripper_base": (-147.349, 28.700, 319.809),
}


def label_name(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString().strip()
    return ""


def referred_label(label: TDF_Label) -> TDF_Label:
    target = TDF_Label()
    if XCAFDoc_ShapeTool.GetReferredShape_s(label, target):
        return target
    return label


def safe_ascii_step(source: Path, destination: Path) -> Path:
    """Replace invalid high bytes in the STEP header without changing source."""
    raw = source.read_bytes()
    try:
        raw.decode("ascii")
        return source
    except UnicodeDecodeError:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(bytes(byte if byte < 128 else 95 for byte in raw))
        return destination


def load_xcaf(step_path: Path):
    app = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("robot-cad"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), document)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(step_path))
    if int(status) != 1:
        raise RuntimeError(f"STEP reader failed with status {status}: {step_path}")
    if not reader.Transfer(document):
        raise RuntimeError(f"STEP transfer failed: {step_path}")
    return document, XCAFDoc_DocumentTool.ShapeTool_s(document.Main())


def iter_leaf_shapes(shape_tool) -> Iterable[dict]:
    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)

    def visit(label, parent_location, path, top_name):
        target = referred_label(label)
        instance_name = label_name(label)
        definition_name = label_name(target)
        # STEP exporters commonly name component instances ``NAUO123`` while
        # the referred definition carries the useful Chinese part/assembly
        # name.  Prefer the definition so grouping remains deterministic.
        name = definition_name or instance_name or "unnamed"
        location = parent_location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
        current_path = path + [name]
        current_top = top_name or name

        children = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(target, children, False)
        if children.Length() > 0:
            for index in range(1, children.Length() + 1):
                yield from visit(children.Value(index), location, current_path, current_top)
            return

        shape = XCAFDoc_ShapeTool.GetShape_s(target)
        if shape.IsNull():
            return
        yield {
            "name": definition_name or name,
            "top": current_top,
            "path": current_path,
            "shape": shape.Moved(location),
        }

    identity = TopLoc_Location()
    for index in range(1, free.Length() + 1):
        root = free.Value(index)
        root_target = referred_label(root)
        children = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(root_target, children, False)
        root_location = XCAFDoc_ShapeTool.GetLocation_s(root)
        if children.Length() == 0:
            yield from visit(root, identity, [], "")
        else:
            for child_index in range(1, children.Length() + 1):
                yield from visit(children.Value(child_index), root_location, [], "")


def compact(text: str) -> str:
    return re.sub(r"[\s_\-—－()（）]+", "", text).lower()


def classify(record: dict) -> str:
    """Map a named assembly leaf to a rigid URDF link or gripper asset."""
    top = compact(record["top"])
    name = compact(record["name"])
    path = compact("/".join(record["path"]))

    # The gripper is nested below the J6 assembly.  Its four identical distal
    # fingers remain on the body mesh for now; the ROS prismatic fingers use
    # simple collision/visual geometry so they can open and close reliably.
    if "电动夹爪" in path or "faef86" in path or "42a962" in path:
        return "gripper_base"

    if top.startswith("j1") or "基座旋转轴" in top:
        rotating_tokens = (
            "yaw轴转轴", "yaw轴平台", "大同步带轮", "j2电机座",
            "6248", "j1roll", "61822", "bearing",
        )
        return "Link_01" if any(token in name for token in rotating_tokens) else "Link_00"

    if top.startswith("j2") or "大臂俯仰轴" in top:
        return "Link_02"

    if top.startswith("j3j4") or "小臂俯仰旋转轴" in top:
        return "Link_04" if "j4" in name else "Link_03"

    if top.startswith("j5") or "末端俯仰轴" in top:
        downstream_tokens = (
            "j5转轴", "4310输出轴", "交叉滚子轴承盖板",
            "j6转轴限位", "j5—转轴", "j5转轴",
        )
        return "Link_05" if any(token in name for token in downstream_tokens) else "Link_04"

    if top.startswith("j6") or "末端旋转轴" in top:
        # The J6 motor stator belongs to link 5; its output stack belongs to 6.
        if "dm4310" in name and not any(token in name for token in ("输出", "法兰")):
            return "Link_05"
        return "Link_06"

    return "unassigned"


def cad_to_link_transform(group: str) -> gp_Trsf:
    """CAD(x,y,z) -> ROS(z,x,y), with base datum and per-link origin."""
    ox, oy, oz = LINK_ORIGINS_MM[group]
    transform = gp_Trsf()
    transform.SetValues(
        0.0, 0.0, 1.0, -274.626 - ox,
        1.0, 0.0, 0.0, -22.719 - oy,
        0.0, 1.0, 0.0, 172.512 - oz,
    )
    return transform


def make_compound(shapes: Iterable[TopoDS_Shape]) -> TopoDS_Compound:
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for shape in shapes:
        builder.Add(compound, shape)
    return compound


def export_group(group: str, records: list[dict], output: Path, deflection: float) -> dict:
    transform = cad_to_link_transform(group)
    transformed = []
    invalid = 0
    for record in records:
        shape = record["shape"]
        if not BRepCheck_Analyzer(shape).IsValid():
            invalid += 1
        transformed.append(BRepBuilderAPI_Transform(shape, transform, True, False).Shape())
    compound = make_compound(transformed)
    BRepMesh_IncrementalMesh(compound, deflection, False, 0.35, True).Perform()
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    # Binary STL keeps the exact same triangles while reducing this assembly
    # from roughly 225 MB of ASCII text to about 42 MB.
    writer.ASCIIMode = False
    if not writer.Write(compound, str(output)):
        raise RuntimeError(f"Failed to write {output}")
    return {"parts": len(records), "invalid_source_shapes": invalid, "file": str(output)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path, help="source STEP assembly")
    parser.add_argument("output", type=Path, help="output mesh directory")
    parser.add_argument("--deflection-mm", type=float, default=0.8)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()

    source = args.step.resolve()
    output = args.output.resolve()
    sanitized = safe_ascii_step(source, output.parent / "_generated" / "robot_arm_ascii.step")
    # Keep the XCAF document alive while traversing labels.  The Python wrapper
    # does not make the shape-tool handle own the document lifetime.
    document, shape_tool = load_xcaf(sanitized)

    grouped: dict[str, list[dict]] = defaultdict(list)
    audit_records = []
    for record in iter_leaf_shapes(shape_tool):
        group = classify(record)
        grouped[group].append(record)
        audit_records.append({
            "group": group,
            "name": record["name"],
            "top": record["top"],
            "path": record["path"],
        })

    expected = [f"Link_{index:02d}" for index in range(7)] + ["gripper_base"]
    missing = [group for group in expected if not grouped[group]]
    if missing:
        raise RuntimeError(f"No STEP parts assigned to: {', '.join(missing)}")

    results = {}
    for group in expected:
        results[group] = export_group(
            group, grouped[group], output / f"{group}.STL", args.deflection_mm
        )

    manifest = args.manifest or output.parent / "cad_mesh_manifest.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps({
        "source": str(source),
        "sanitized_input": str(sanitized),
        "units": "millimetres",
        "coordinate_mapping": "ros_x=cad_z-274.626; ros_y=cad_x-22.719; ros_z=cad_y+172.512",
        "group_counts": dict(Counter(item["group"] for item in audit_records)),
        "exports": results,
        "records": audit_records,
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Converted {len(audit_records)} leaf instances")
    for group in expected:
        print(f"  {group}: {len(grouped[group])} parts -> {output / (group + '.STL')}")
    if grouped["unassigned"]:
        print(f"WARNING: {len(grouped['unassigned'])} parts are unassigned; see {manifest}")
    print(f"Manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

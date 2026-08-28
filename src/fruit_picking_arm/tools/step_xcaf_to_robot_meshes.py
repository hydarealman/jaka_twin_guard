#!/usr/bin/env python3
"""Legacy STEP *mesh splitter* for the fruit-picking arm.

This is deliberately not a CAD-to-URDF kinematics converter.  AP203/AP214 STEP
provides geometry and assembly placements, but this script does not read the
SolidWorks mates that define rigid groups, revolute joints, joint axes or the
mechanical-zero configuration.  The historical implementation silently mixed
name-based part grouping, hand-entered joint origins and a shared CAD rotation;
that can produce a visually plausible model with incorrect MoveIt kinematics.

Production URDFs must be extracted from the Pack and Go ``.SLDASM`` with
``sw2robot`` (or another SolidWorks mate-aware exporter), then validated against
named ``BASE_URDF``, ``J1_URDF`` ... ``J6_URDF`` and ``TCP_URDF`` frames.  This
tool remains a mesh-only repair/audit tool and refuses to run unless the caller
explicitly acknowledges that rigid grouping is still name-based.  With
``--reference-urdf`` it uses the mechanical export's q=0 link frames instead
of the historical hand-entered frame constants; it still does not derive or
validate robot kinematics from STEP.

The source drawing uses millimetres.  Legacy output also uses millimetres;
``--reference-urdf`` output uses metres and is intended for ``scale=1``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

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

REFERENCE_OUTPUT_NAMES = {
    "Link_00": "base_link",
    "Link_01": "Link1",
    "Link_02": "Link2",
    "Link_03": "Link3",
    "Link_04": "Link4",
    "Link_05": "Link5",
    "Link_06": "Link6",
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

    # The gripper is nested below the J6 assembly.  SolidWorks has used both
    # FAE4M86M+D (the complete actuator assembly) and FAEF86 (distal fingers)
    # in delivered drawings.  Missing the former silently baked the gripper
    # motor, housing and linkage into Link6, so J6 could never be a pure arm
    # rigid body.  Keep every gripper leaf out of Link6; the dedicated gripper
    # converter owns its fixed and moving meshes.
    if (
        "电动夹爪" in path
        or "fae4m86m" in path
        or "faef86" in path
        or "42a962" in path
    ):
        return "gripper_base"

    # Optional battery shells are fixed to the base plate in the supplied
    # assembly.  Older drawings did not contain them and left them unassigned.
    if "电池" in path:
        return "Link_00"

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
        # Although its CAD name says "output shaft", this long J5 motor part
        # sits on the upstream forearm side of the J5 axis.  Assigning it to
        # Link_05 makes it rotate away from the forearm and appear suspended.
        if "4310输出轴" in name:
            return "Link_04"
        downstream_tokens = (
            "j5转轴", "交叉滚子轴承盖板",
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


def _matrix_multiply(first, second):
    return tuple(
        tuple(sum(first[row][index] * second[index][column] for index in range(3))
              for column in range(3))
        for row in range(3)
    )


def _matrix_vector(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _rpy_matrix(roll: float, pitch: float, yaw: float):
    """Return the URDF fixed-axis Rz(yaw) * Ry(pitch) * Rx(roll) matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _parse_vector(element, attribute: str, default: str):
    raw = element.get(attribute, default) if element is not None else default
    values = tuple(float(value) for value in raw.split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid {attribute} vector in reference URDF: {raw!r}")
    return values


def reference_zero_link_transforms(urdf_path: Path):
    """Read base-from-link transforms for J1..J6 at q=0.

    The SolidWorks export's link meshes are expressed in these local frames.
    Reading the frames from the mechanical export avoids the old converter's
    hand-entered link origins and shared CAD-axis permutation.
    """
    try:
        root = ElementTree.parse(urdf_path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RuntimeError(f"cannot read reference URDF {urdf_path}: {error}") from error

    joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
    result = {"Link_00": (((1.0, 0.0, 0.0),
                           (0.0, 1.0, 0.0),
                           (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0))}
    previous_child = None
    source_links = {}
    for index in range(1, 7):
        name = f"J{index}"
        joint = joints.get(name)
        if joint is None:
            raise RuntimeError(f"reference URDF is missing joint {name}")
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        parent = parent_element.get("link") if parent_element is not None else None
        child = child_element.get("link") if child_element is not None else None
        if not parent or not child:
            raise RuntimeError(f"reference URDF joint {name} has no parent/child")
        if index == 1:
            source_links["Link_00"] = parent
        elif parent != previous_child:
            raise RuntimeError(
                f"reference URDF is not a serial J1..J6 chain: {name} parent "
                f"{parent!r} != previous child {previous_child!r}"
            )

        origin = joint.find("origin")
        translation = _parse_vector(origin, "xyz", "0 0 0")
        rotation = _rpy_matrix(*_parse_vector(origin, "rpy", "0 0 0"))
        parent_rotation, parent_translation = result[f"Link_{index - 1:02d}"]
        child_rotation = _matrix_multiply(parent_rotation, rotation)
        offset = _matrix_vector(parent_rotation, translation)
        child_translation = tuple(
            parent_translation[axis] + offset[axis] for axis in range(3)
        )
        group = f"Link_{index:02d}"
        result[group] = (child_rotation, child_translation)
        source_links[group] = child
        previous_child = child
    return result, source_links


def cad_to_reference_link_transform(rotation, translation) -> gp_Trsf:
    """Map STEP assembly millimetres into a reference URDF link frame/metres."""
    inverse_rotation = _transpose(rotation)
    inverse_translation = _matrix_vector(
        inverse_rotation, tuple(-value for value in translation)
    )
    transform = gp_Trsf()
    transform.SetValues(
        0.001 * inverse_rotation[0][0],
        0.001 * inverse_rotation[0][1],
        0.001 * inverse_rotation[0][2],
        inverse_translation[0],
        0.001 * inverse_rotation[1][0],
        0.001 * inverse_rotation[1][1],
        0.001 * inverse_rotation[1][2],
        inverse_translation[1],
        0.001 * inverse_rotation[2][0],
        0.001 * inverse_rotation[2][1],
        0.001 * inverse_rotation[2][2],
        inverse_translation[2],
    )
    return transform


def make_compound(shapes: Iterable[TopoDS_Shape]) -> TopoDS_Compound:
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for shape in shapes:
        builder.Add(compound, shape)
    return compound


def export_group(
    group: str,
    records: list[dict],
    output: Path,
    deflection: float,
    transform: gp_Trsf | None = None,
) -> dict:
    transform = transform or cad_to_link_transform(group)
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
    parser.add_argument(
        "--reference-urdf",
        type=Path,
        help=(
            "express Link_00..Link_06 in the q=0 link frames from a classic "
            "SolidWorks URDF export; output mesh units are metres"
        ),
    )
    parser.add_argument(
        "--unsafe-legacy-inferred-frames",
        action="store_true",
        help=(
            "acknowledge the name-inferred rigid grouping; reference-URDF "
            "mode fixes link frames but is still not independent proof of "
            "MoveIt kinematic correctness"
        ),
    )
    args = parser.parse_args()

    if not args.unsafe_legacy_inferred_frames:
        parser.error(
            "refusing an unsafe STEP-only URDF conversion: use the SolidWorks "
            "Pack and Go + sw2robot workflow; pass "
            "--unsafe-legacy-inferred-frames only to reproduce/audit old meshes"
        )

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

    arm_groups = [f"Link_{index:02d}" for index in range(7)]
    expected = arm_groups + ([] if args.reference_urdf else ["gripper_base"])
    missing = [group for group in expected if not grouped[group]]
    if missing:
        raise RuntimeError(f"No STEP parts assigned to: {', '.join(missing)}")

    reference_transforms = None
    reference_links = None
    reference_urdf = args.reference_urdf.resolve() if args.reference_urdf else None
    if reference_urdf:
        reference_transforms, reference_links = reference_zero_link_transforms(reference_urdf)

    results = {}
    for group in expected:
        filename = (
            REFERENCE_OUTPUT_NAMES[group] + ".STL"
            if reference_transforms is not None
            else group + ".STL"
        )
        transform = (
            cad_to_reference_link_transform(*reference_transforms[group])
            if reference_transforms is not None
            else None
        )
        results[group] = export_group(
            group, grouped[group], output / filename, args.deflection_mm, transform
        )

    manifest = args.manifest or output.parent / "cad_mesh_manifest.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps({
        "source": str(source),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "sanitized_input": str(sanitized),
        "units": "metres" if reference_urdf else "millimetres",
        "kinematics_validated": False,
        "warning": (
            "STEP rigid groups remain inferred from audited assembly names. "
            "Reference-URDF mode copies q=0 link frames from the mechanical "
            "SolidWorks export, but it is not independent proof of kinematic "
            "accuracy."
            if reference_urdf
            else
            "Legacy STEP-only mesh split. Rigid groups and link frames were "
            "inferred by names/manual constants; this manifest is not valid "
            "evidence for URDF or MoveIt kinematics."
        ),
        "coordinate_mapping": (
            "STEP assembly millimetres -> mechanical SolidWorks URDF q=0 link frames/metres"
            if reference_urdf
            else
            "ros_x=cad_z-274.626; ros_y=cad_x-22.719; ros_z=cad_y+172.512"
        ),
        "reference_urdf": str(reference_urdf) if reference_urdf else None,
        "reference_urdf_sha256": (
            hashlib.sha256(reference_urdf.read_bytes()).hexdigest()
            if reference_urdf else None
        ),
        "reference_links": reference_links,
        "group_counts": dict(Counter(item["group"] for item in audit_records)),
        "exports": results,
        "records": audit_records,
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Converted {len(audit_records)} leaf instances")
    for group in expected:
        print(f"  {group}: {len(grouped[group])} parts -> {results[group]['file']}")
    if grouped["unassigned"]:
        print(f"WARNING: {len(grouped['unassigned'])} parts are unassigned; see {manifest}")
    print(f"Manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

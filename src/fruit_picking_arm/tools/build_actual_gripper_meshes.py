#!/usr/bin/env python3
"""Build endpoint-accurate four-finger gripper meshes from two STEP poses.

The mechanical delivery contains closed and open STEP assemblies in the same
GRIPPER_MOUNT coordinate system.  This tool identifies fixed bodies, the centre
slider, four drive links and four two-piece fingers from their exact rigid-body
transform between the two endpoints.  Meshes are emitted in metres in local
URDF joint frames, with the closed assembly as q=0.

Only the two endpoints can be recovered exactly from two static STEP files.
The real screw/linkage motion is nonlinear, so URDF mimic joints interpolate
between the endpoints and are explicitly documented as an intermediate-pose
visual/collision approximation.  Command conversion remains the responsibility
of the real gripper controller and is not inferred here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from OCP.gp import gp_Trsf

from step_xcaf_to_robot_meshes import (
    compact,
    export_group,
    iter_leaf_shapes,
    load_xcaf,
    safe_ascii_step,
)


MOUNT_FROM_J6_M = (0.0422, 0.0, 0.0)
TCP_FROM_MOUNT_M = (0.0, 0.0, 0.124)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def shape_pose(record: dict) -> np.ndarray:
    transform = record["shape"].Location().Transformation()
    matrix = np.eye(4)
    for row in range(3):
        for column in range(3):
            matrix[row, column] = transform.Value(row + 1, column + 1)
    translation = transform.TranslationPart()
    matrix[:3, 3] = (translation.X(), translation.Y(), translation.Z())
    return matrix


def relative_motion(closed: dict, opened: dict) -> dict:
    delta = shape_pose(opened) @ np.linalg.inv(shape_pose(closed))
    rotation = delta[:3, :3]
    translation = delta[:3, 3]
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        axis = np.zeros(3)
        pivot = np.zeros(3)
        # A zero-rotation delta is an exact prismatic transform; its endpoint
        # fitting residual is zero, not the translation magnitude.
        residual = 0.0
    else:
        sine = math.sin(angle)
        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        ) / (2.0 * sine)
        axis /= np.linalg.norm(axis)
        pivot, _, _, _ = np.linalg.lstsq(
            np.eye(3) - rotation, translation, rcond=None
        )
        residual = float(
            np.linalg.norm((np.eye(3) - rotation) @ pivot - translation)
        )
    return {
        "angle_rad": angle,
        "axis": axis,
        "pivot_mm": pivot,
        "translation_mm": translation,
        "residual_mm": residual,
    }


def scale_about_origin(origin_mm: np.ndarray | tuple[float, ...]) -> gp_Trsf:
    origin = np.asarray(origin_mm, dtype=float)
    scale = 0.001
    transform = gp_Trsf()
    transform.SetValues(
        scale, 0.0, 0.0, -scale * origin[0],
        0.0, scale, 0.0, -scale * origin[1],
        0.0, 0.0, scale, -scale * origin[2],
    )
    return transform


def cardinal_name(pivot_mm: np.ndarray) -> str:
    x, y, _ = pivot_mm
    if abs(x) >= abs(y):
        return "pos_x" if x >= 0.0 else "neg_x"
    return "pos_y" if y >= 0.0 else "neg_y"


def json_vector(values) -> list[float]:
    return [float(value) for value in values]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("closed_step", type=Path)
    parser.add_argument("open_step", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--deflection-mm", type=float, default=0.25)
    args = parser.parse_args()

    closed_path = args.closed_step.resolve()
    open_path = args.open_step.resolve()
    output = args.output.resolve()
    # Sanitised STEP copies are parser scratch data, not runtime assets.  Keep
    # them outside the ROS description package so colcon cannot install them.
    generated = (
        Path(tempfile.gettempdir())
        / "fruit_arm_gripper_step_ascii"
        / f"{sha256(closed_path)[:12]}_{sha256(open_path)[:12]}"
    )
    closed_ascii = safe_ascii_step(closed_path, generated / "closed_ascii.step")
    open_ascii = safe_ascii_step(open_path, generated / "open_ascii.step")

    # Keep both XCAF documents alive while their shape handles are in use.
    closed_document, closed_tool = load_xcaf(closed_ascii)
    open_document, open_tool = load_xcaf(open_ascii)
    closed_records = list(iter_leaf_shapes(closed_tool))
    open_records = list(iter_leaf_shapes(open_tool))
    if len(closed_records) != len(open_records):
        raise RuntimeError(
            f"STEP leaf count differs: closed={len(closed_records)}, "
            f"open={len(open_records)}"
        )
    if len(closed_records) != 16:
        raise RuntimeError(
            f"expected the delivered 16-leaf gripper, got {len(closed_records)}"
        )

    items = []
    for index, (closed, opened) in enumerate(zip(closed_records, open_records)):
        if compact(closed["name"]) != compact(opened["name"]):
            raise RuntimeError(
                f"STEP leaf order/name mismatch at {index}: "
                f"{closed['name']!r} != {opened['name']!r}"
            )
        motion = relative_motion(closed, opened)
        angle_deg = math.degrees(motion["angle_rad"])
        translation_norm = float(np.linalg.norm(motion["translation_mm"]))
        if angle_deg < 1e-5 and translation_norm < 1e-5:
            kind = "fixed"
        elif angle_deg < 1e-5 and 10.0 < translation_norm < 16.0:
            kind = "slider"
        elif 8.0 < angle_deg < 14.0:
            kind = "drive"
        elif 45.0 < angle_deg < 55.0:
            kind = "finger"
        else:
            raise RuntimeError(
                f"unrecognised motion at leaf {index} {closed['name']!r}: "
                f"angle={angle_deg:.9f} deg, translation={translation_norm:.9f} mm"
            )
        items.append({
            "index": index,
            "closed": closed,
            "opened": opened,
            "kind": kind,
            **motion,
        })

    fixed = [item for item in items if item["kind"] == "fixed"]
    slider = [item for item in items if item["kind"] == "slider"]
    drives = [item for item in items if item["kind"] == "drive"]
    fingers = [item for item in items if item["kind"] == "finger"]
    if (len(fixed), len(slider), len(drives), len(fingers)) != (3, 1, 4, 8):
        raise RuntimeError(
            "unexpected rigid-body split: "
            f"fixed={len(fixed)}, slider={len(slider)}, "
            f"drive={len(drives)}, finger parts={len(fingers)}"
        )

    finger_groups: dict[str, list[dict]] = {}
    for item in fingers:
        name = cardinal_name(item["pivot_mm"])
        finger_groups.setdefault(name, []).append(item)
    if set(finger_groups) != {"pos_x", "neg_x", "pos_y", "neg_y"}:
        raise RuntimeError(f"unexpected finger pivots: {sorted(finger_groups)}")
    if any(len(group) != 2 for group in finger_groups.values()):
        raise RuntimeError("each finger must contain exactly two STEP leaf parts")

    drive_groups = {cardinal_name(item["pivot_mm"]): item for item in drives}
    if set(drive_groups) != set(finger_groups):
        raise RuntimeError(f"unexpected drive-link pivots: {sorted(drive_groups)}")

    output.mkdir(parents=True, exist_ok=True)
    exports = {}
    exports["gripper_base"] = export_group(
        "gripper_base",
        [item["closed"] for item in fixed],
        output / "gripper_base.STL",
        args.deflection_mm,
        scale_about_origin((0.0, 0.0, 0.0)),
    )
    exports["center_slider"] = export_group(
        "center_slider",
        [slider[0]["closed"]],
        output / "center_slider.STL",
        args.deflection_mm,
        scale_about_origin((0.0, 0.0, 0.0)),
    )

    joints = {}
    slider_travel = float(np.linalg.norm(slider[0]["translation_mm"])) * 0.001
    slider_axis = slider[0]["translation_mm"] / np.linalg.norm(
        slider[0]["translation_mm"]
    )
    joints["center_slider"] = {
        "type": "prismatic",
        "origin_m": [0.0, 0.0, 0.0],
        "axis": json_vector(slider_axis),
        "travel": slider_travel,
        "endpoint_residual_mm": slider[0]["residual_mm"],
    }

    for cardinal in sorted(drive_groups):
        item = drive_groups[cardinal]
        key = f"drive_{cardinal}"
        exports[key] = export_group(
            key,
            [item["closed"]],
            output / f"{key}.STL",
            args.deflection_mm,
            scale_about_origin(item["pivot_mm"]),
        )
        joints[key] = {
            "type": "revolute",
            "origin_m": json_vector(item["pivot_mm"] * 0.001),
            "axis": json_vector(item["axis"]),
            "travel": item["angle_rad"],
            "endpoint_residual_mm": item["residual_mm"],
        }

    for cardinal in sorted(finger_groups):
        group = finger_groups[cardinal]
        reference = group[0]
        for item in group[1:]:
            if (
                np.linalg.norm(item["pivot_mm"] - reference["pivot_mm"]) > 1e-5
                or abs(float(np.dot(item["axis"], reference["axis"]))) < 0.999999
                or abs(item["angle_rad"] - reference["angle_rad"]) > 1e-8
            ):
                raise RuntimeError(f"finger {cardinal} parts do not share one motion")
        key = f"finger_{cardinal}"
        exports[key] = export_group(
            key,
            [item["closed"] for item in group],
            output / f"{key}.STL",
            args.deflection_mm,
            scale_about_origin(reference["pivot_mm"]),
        )
        joints[key] = {
            "type": "revolute",
            "origin_m": json_vector(reference["pivot_mm"] * 0.001),
            "axis": json_vector(reference["axis"]),
            "travel": reference["angle_rad"],
            "endpoint_residual_mm": max(item["residual_mm"] for item in group),
        }

    manifest = args.manifest or output / "KINEMATICS.json"
    manifest.write_text(
        json.dumps(
            {
                "closed_step": str(closed_path),
                "closed_step_sha256": sha256(closed_path),
                "open_step": str(open_path),
                "open_step_sha256": sha256(open_path),
                "units": "metres",
                "mesh_pose": "closed endpoint (q=0)",
                "j6_to_gripper_mount": {
                    "xyz_m": list(MOUNT_FROM_J6_M),
                    "source": "mechanical measurement supplied 2026-08-27",
                },
                "gripper_mount_to_tcp": {
                    "xyz_m": list(TCP_FROM_MOUNT_M),
                    "source": "mechanical measurement supplied 2026-08-27",
                },
                "motion_model": (
                    "closed/open endpoint transforms are exact; intermediate "
                    "URDF mimic motion is an approximation because the real "
                    "screw-driven four-bar linkage is nonlinear"
                ),
                "leaf_counts": {
                    "fixed": len(fixed),
                    "slider": len(slider),
                    "drive_links": len(drives),
                    "finger_parts": len(fingers),
                },
                "joints": joints,
                "exports": exports,
                "records": [
                    {
                        "index": item["index"],
                        "name": item["closed"]["name"],
                        "path": item["closed"]["path"],
                        "kind": item["kind"],
                        "angle_rad": item["angle_rad"],
                        "translation_mm": json_vector(item["translation_mm"]),
                        "axis": json_vector(item["axis"]),
                        "pivot_mm": json_vector(item["pivot_mm"]),
                        "residual_mm": item["residual_mm"],
                    }
                    for item in items
                ],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Built {len(exports)} gripper meshes in {output}")
    print(f"Kinematic manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

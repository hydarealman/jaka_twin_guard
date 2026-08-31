#!/usr/bin/env python3
"""Validated sorting-bin geometry shared by MoveIt and RViz.

The public YAML contract is deliberately physical and measurable:

* ``center.x/y/z`` is the geometric centre of the bin's outer envelope;
* ``size.x/y/z`` are the *outer* length, width and height;
* ``wall_thickness`` is the nominal wall/bottom thickness.

Keeping this conversion in one place prevents RViz markers and MoveIt
collision bodies from quietly using different dimensions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BinBox:
    """One axis-aligned box forming part of a five-sided sorting bin."""

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


def normalized_bin_config(config: dict) -> dict:
    """Return validated numeric bin geometry or raise ``ValueError``."""

    try:
        center = config["center"]
        size = config["size"]
        cx = float(center["x"])
        cy = float(center["y"])
        cz = float(center["z"])
        sx = float(size["x"])
        sy = float(size["y"])
        sz = float(size["z"])
        thickness = float(config.get("wall_thickness", 0.01))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "bin requires center.x/y/z, size.x/y/z and wall_thickness"
        ) from exc

    values = (cx, cy, cz, sx, sy, sz, thickness)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("bin geometry must contain only finite values")
    if min(sx, sy, sz, thickness) <= 0.0:
        raise ValueError("bin size and wall_thickness must be positive")
    if sx <= 2.0 * thickness or sy <= 2.0 * thickness:
        raise ValueError("bin opening disappears: size.x/y must exceed two walls")
    if sz <= thickness:
        raise ValueError("bin height must exceed its bottom thickness")

    return {
        "center": {"x": cx, "y": cy, "z": cz},
        "top_z": cz + sz * 0.5,
        "size": {"x": sx, "y": sy, "z": sz},
        "wall_thickness": thickness,
        "inner_size": {"x": sx - 2.0 * thickness, "y": sy - 2.0 * thickness},
        "bottom_z": cz - sz * 0.5,
    }


def bin_wall_boxes(config: dict) -> list[BinBox]:
    """Convert one measured outer bin into bottom plus four wall boxes."""

    cfg = normalized_bin_config(config)
    cx, cy, cz = (
        cfg["center"]["x"],
        cfg["center"]["y"],
        cfg["center"]["z"],
    )
    top_z = cfg["top_z"]
    sx, sy, sz = cfg["size"]["x"], cfg["size"]["y"], cfg["size"]["z"]
    thickness = cfg["wall_thickness"]
    bottom_z = cfg["bottom_z"]

    # Wall centres are inset by half a wall thickness.  Consequently the
    # outside faces land exactly on the measured outer dimensions.
    wall_center_z = cz
    x_wall = (sx - thickness) * 0.5
    y_wall = (sy - thickness) * 0.5
    return [
        BinBox(
            "bottom",
            (cx, cy, bottom_z + thickness * 0.5),
            (sx, sy, thickness),
        ),
        BinBox("minus_x", (cx - x_wall, cy, wall_center_z), (thickness, sy, sz)),
        BinBox("plus_x", (cx + x_wall, cy, wall_center_z), (thickness, sy, sz)),
        # X walls span the corners, so shorten Y walls to avoid double-thick
        # corner geometry while preserving the exact outer envelope.
        BinBox(
            "minus_y",
            (cx, cy - y_wall, wall_center_z),
            (sx - 2.0 * thickness, thickness, sz),
        ),
        BinBox(
            "plus_y",
            (cx, cy + y_wall, wall_center_z),
            (sx - 2.0 * thickness, thickness, sz),
        ),
    ]


def bin_drop_candidates(
    config: dict,
    fruit_radius: float,
    wall_clearance: float = 0.010,
) -> list[tuple[float, float]]:
    """Return deterministic fruit-centre candidates inside the bin opening.

    The surveyed bin centre remains the geometry reference.  Candidate points
    account for wall thickness, fruit radius and an extra clearance.  After
    the centre, points favour the side nearest the robot base (``y=0``) and
    then progressively larger public ``+X`` for better real-arm reachability.
    """

    cfg = normalized_bin_config(config)
    radius = float(fruit_radius)
    clearance = float(wall_clearance)
    if not math.isfinite(radius) or not math.isfinite(clearance):
        raise ValueError("fruit radius and wall clearance must be finite")
    if radius < 0.0 or clearance < 0.0:
        raise ValueError("fruit radius and wall clearance must be non-negative")

    cx = cfg["center"]["x"]
    cy = cfg["center"]["y"]
    sx = cfg["size"]["x"]
    sy = cfg["size"]["y"]
    inset = cfg["wall_thickness"] + radius + clearance
    min_x, max_x = cx - sx * 0.5 + inset, cx + sx * 0.5 - inset
    min_y, max_y = cy - sy * 0.5 + inset, cy + sy * 0.5 - inset
    if min_x > max_x or min_y > max_y:
        raise ValueError(
            "fruit plus clearance does not fit inside the measured bin opening"
        )

    near_y = min(max(0.0, min_y), max_y)
    raw = [
        (cx, cy),
        (cx, near_y),
        (0.5 * (cx + max_x), near_y),
        (max_x, near_y),
        (0.5 * (cx + min_x), near_y),
        (min_x, near_y),
    ]
    candidates: list[tuple[float, float]] = []
    for point in raw:
        if not any(
            math.hypot(point[0] - old[0], point[1] - old[1]) < 1.0e-9
            for old in candidates
        ):
            candidates.append(point)
    return candidates

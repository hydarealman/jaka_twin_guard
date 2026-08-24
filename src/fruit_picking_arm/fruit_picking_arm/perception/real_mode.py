"""Guards that keep physical bring-up on measured hardware data only."""

from __future__ import annotations


def validate_real_perception_config(
    *,
    camera_type: str,
    allow_scene_fallback: bool,
    force_table_center_z: bool,
    enable_table_z_fallback: bool,
) -> None:
    """Reject simulation-only perception options in a physical run."""

    errors: list[str] = []
    if str(camera_type).strip().lower() != "realsense":
        errors.append(
            f"camera_type must be 'realsense', got '{camera_type}'"
        )
    if bool(allow_scene_fallback):
        errors.append("allow_scene_fallback must be false")
    if bool(force_table_center_z):
        errors.append("force_table_center_z must be false")
    if bool(enable_table_z_fallback):
        errors.append("enable_table_z_fallback must be false")
    if errors:
        raise ValueError(
            "physical mode rejects simulation perception settings: "
            + "; ".join(errors)
        )


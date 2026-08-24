# D455 6D pose package

This package is intentionally independent from `fruit_picking_arm`. It neither
imports that package nor uses `/perception/fruit_*` topics. The only shared
resource is the already-running D455 driver.

Current backends:

- `bottle`: opaque colour segmentation plus aligned-depth PCA. It returns the
  bottle centre and axis and explicitly reports axial-rotation ambiguity.
- `energy_unit`: four-corner planar PnP plus depth consistency. It is blocked
  by `configuration_complete: false` until the real target definition is
  supplied. A future keypoint ONNX detector can replace the contour front end
  without changing ROS output topics.

The launch files default to `start_camera:=false`; they never create a second
RealSense process unless explicitly requested. Future commands are:

```bash
ros2 launch d455_6d_pose bottle_pose.launch.py start_camera:=false
ros2 launch d455_6d_pose energy_unit_pose.launch.py start_camera:=false
```

Inputs:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Outputs are confined to `/d455_6d_pose/*`. Invalid, stale, unaligned,
low-confidence or TF-unavailable frames produce diagnostics but no pose.

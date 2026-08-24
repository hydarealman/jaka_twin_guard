# D455 fruit-perception tuning record

This record applies to D455 serial `261822300719`, firmware `5.15.1.55`,
RealSense ROS `4.58.2` and librealsense `2.58.2/2.58.3`.

## Selected baseline

- USB: SuperSpeed / USB 3.2. Do not accept a USB 2 connection for RGB-D use.
- RGB and depth: `848x480x30`. This preserves the D455 depth field of view and
  is a practical starting point for table-scale fruit localisation.
- RGB exposure and white balance: automatic during development. Mains flicker
  is fixed to `50 Hz`; automatic flicker selection is unnecessary in China.
- Depth preset: `Default` (`1`). In the measured mixed indoor scene it retained
  more valid pixels than Hand, High Accuracy, High Density or Medium Density.
- Emitter: enabled; laser power `150` (device default). Do not raise it before
  testing fruit material, range, interference and eye-safety constraints.
- Alignment and point cloud: enabled because the classifier uses RGB while 3D
  localisation consumes colour-aligned depth/points.
- Spatial and temporal filters: enabled. On the current scene, median temporal
  depth deviation fell from `6.5 mm` to `3.5 mm`, and the valid-pixel median
  rose from `82.31%` to `83.87%` in the sampled ROI.
- Hole filling: disabled. Fabricated depth can create unsafe grasp targets at
  fruit silhouettes; downstream code must reject invalid depth instead.
- IMU/infrared streams: disabled because this eye-to-hand fruit stack does not
  consume them. This also avoids unnecessary USB and ROS traffic.
- `initial_reset`: disabled. Under WSL, a device reset breaks the USBIP attach.

Windows-native validation captured 120 aligned RGB-D frames at a measured
`26.38 fps` including startup/warm-up overhead; the post-warm-up 95th-percentile
frame interval was `33.7 ms`. Depth-only capture reported a `1 mm` depth unit
and `82.09%` median valid pixels in the unprepared room scene.

## WSL limitation found during testing

WSL/USBIP transported D455 colour at the requested `15 Hz`, but depth timed
out even at `640x480x15`. Kernel logs contained
`vhci_get_frame_number: Not yet implemented`. Windows-native librealsense read
depth correctly, so this is a USBIP isochronous-transfer limitation, not a
camera fault or a ROS profile error.

Use native Ubuntu on the deployment computer for RGB-D acceptance. WSL remains
useful for colour-only model integration:

```bash
# In an Administrator PowerShell, only once per USB topology:
usbipd bind --busid 1-19

# In ordinary PowerShell after each reboot/replug:
usbipd attach --wsl --busid 1-19

# In Ubuntu/WSL: colour-only development (no 3D target output):
ros2 launch fruit_picking_arm d455_fruit_debug.launch.py enable_depth:=false
```

On native Ubuntu, run full RGB-D without the robot or serial stack:

```bash
ros2 launch fruit_picking_arm d455_fruit_debug.launch.py enable_depth:=true
```

For either control architecture, start this driver once, then launch the real
stack with `start_camera:=false` to avoid two processes owning the D455.

## Retired in-house model result from the first real scene

The former in-house YOLO model classified a dark door/window frame as a
`Healthy` apple. Scores moved from roughly `0.32` to `0.59` as exposure and
framing changed, despite there being no fruit in view. Raising the detector
threshold from `0.25` to `0.50` did not remove the false positive reliably.

The former `0.60` threshold was therefore not a valid solution. The in-house
weights and their whole-frame detector were subsequently removed.

## Real healthy-apple check and replacement (2026-08-12)

One known-healthy red apple was placed in the real D455 scene.  At the tested
camera position its apparent diameter was only about `40 px` in an
`848x480` frame.

- Across 478 frames, the apple itself was not detected at the normal full-frame
  scale.  A large box over the left-side background was repeatedly labelled
  `Healthy` instead.  At threshold `0.25` this false box appeared in `97.7%`
  of processed frames (median score `0.431`, maximum `0.518`).
- At threshold `0.60`, neither the background nor the apple was detected.
  Therefore lowering the threshold creates false targets, while keeping the
  current conservative threshold misses this fruit.
- An ROI scale probe made the same apple occupy approximately `60-175 px`.
  The boxes covering the apple were labelled `Unhealthy`, including a score of
  `0.612` at the approximately `60 px` scale.  This crop-based probe is a
  diagnostic, not a substitute for a physical close-range acceptance run.
- Windows-native inference averaged `37.8 ms/frame` for the sampled run.

This known-positive sample therefore **failed the retired YOLO recognition
acceptance check**: it is either missed or assigned the wrong health class, and
the scene also produces a stable background false positive.  No production
confidence threshold can be selected from the present model.  Collect D455
images of this apple at the final working distance, with varied pose and
lighting, and include the current background as negative training data before
retraining and held-out validation.

The selected open MobileNetV3-Large classifier was then tested on the tight
ROI of the same D455 apple and returned `fresh apple` at `75.5%`.  The runtime
was changed to depth/shape localisation followed by ROI-only classification;
whole-frame quality inference is no longer used.  This is an improvement on a
single known-positive sample, not final validation: rotten-apple acceptance
still requires real rotten fruit and the final camera geometry.

## Work still blocked until later hardware is available

- Eye-to-hand calibration and base-frame target accuracy need the calibration
  board and final rigid camera mount.
- Detection ROI, exposure ROI, useful depth range and laser power must be
  rechecked after the camera, table and lighting positions are fixed.
- No automatic pick is permitted from WSL colour-only testing or from the
  current development model.

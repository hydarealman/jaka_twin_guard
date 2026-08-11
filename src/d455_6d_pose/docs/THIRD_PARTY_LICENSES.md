# Third-party delivery record

The package currently contains no copied third-party source, datasets, model
weights or CAD assets. It links to system packages:

| Dependency | Intended use | License |
|---|---|---|
| ROS 2 Humble client/messages | Runtime communication | Apache-2.0 / package-specific |
| OpenCV | Colour segmentation, PnP and projection | Apache-2.0 |
| Intel RealSense ROS wrapper | D455 image publication, not vendored here | Apache-2.0 |

FoundationPose is not included. Its official source license limits use to
non-commercial research/evaluation and is therefore unsuitable as the default
backend for this paid delivery. Future ONNX weights, training code, data and
CAD require a separate hash/provenance/license entry before release.

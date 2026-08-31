# Fruit-quality model and data license audit

This project is being prepared for a paid, closed-source enterprise delivery.
The checked-in weights are therefore **development-only** until the items below
are approved in writing by the customer/legal owner.

## Decision summary

| Asset | What was verified | Delivery decision |
|---|---|---|
| Retired in-house `best.onnx`, `best.pt` | The project-trained YOLOv8 weights performed poorly on the real D455 scene and carried Ultralytics AGPL metadata. | **Removed** from source and runtime; never ship these obsolete weights. |
| `models/d455_apple_detector_v1.pt`, `d455_apple_detector_v2.pt` | D455 tabletop adaptations used for apple localization; hashes and engineering results are recorded in `models/README.md`. | **Runtime engineering models**. v1 is pinned by the human-gated fruit RViz script; general real wrappers select v2. Dataset/retraining provenance and held-out production acceptance remain required. |
| `models/fruit_quality_mobilenet_v3.onnx` | Exported from `NararyaPutra/Freshness_Banana_Orange_Apple_Image-Classification`, upstream commit `31534dd9b3c74a0e6599867297327297c861999b`; model card declares Apache-2.0. | **Selected development classifier**. It only classifies detected ROIs. Training data is described only as a custom dataset, so written provenance remains required for enterprise release. |
| `FruitNet` | Evaluated as an alternative, but requires TensorFlow and has no non-fruit rejection class. | **Removed**; not part of the runtime or deliverable. |
| FruitVision dataset | Mendeley record declares CC BY-NC-ND 4.0. | **Do not use** for commercial training, fine-tuning, or redistribution. |

Authoritative references:

- [Fruit-quality MobileNetV3 candidate](https://huggingface.co/NararyaPutra/Freshness_Banana_Orange_Apple_Image-Classification)
- [FruitVision dataset and CC BY-NC-ND terms](https://data.mendeley.com/datasets/xkbjx8959c/2)

## Required release evidence

Before setting `model_license_approved:=true` in a real launch, attach:

1. The model file hash, model card, training code, and all transitive package
   licenses.
2. Written permission for every training/validation image source, including
   the exact dataset version and required attribution text.
3. A held-out test report made from the customer's camera, fruit varieties,
   lighting, background, and camera-to-fruit distance. Report per-class
   precision/recall, false-sort rate, unknown/reject rate, and latency.
4. Evidence that the model never emits a pick target for an unknown, weak, or
   conflicting quality result. The ROS target tracker and serial bridge are
   fail-closed for this purpose.

The raw real launch files default to `model_license_approved:=false` and refuse
to start the detector in production mode. The project real-operation shell
scripts currently pass `model_license_approved:=true`; that is an explicit
project configuration choice, not evidence that legal/customer acceptance has
been completed. Simulation/development mode can use the checked-in weights for
integration testing.

## Current external-video smoke test (not a benchmark)

The reusable evaluator is
`test/evaluate_fruit_video.py`. It sampled two Pexels videos solely as an
out-of-domain smoke test; the videos are not copied into the deliverable:

- [Person Holding a Green Apple](https://www.pexels.com/video/person-holding-a-green-apple-7889704/)
- [Flies Eating Rotten Fruit](https://www.pexels.com/video/flies-eating-rotten-fruit-5243421/)

With the checked-in model, 0/33 healthy-scene frames were correctly classified
and 2/23 rotten-scene frames were correctly classified under the scene-level
label protocol. The scenes contain hands, outdoor backgrounds, and fruit types
not represented by the current two-class training set, so these numbers are a
robustness warning—not a customer acceptance score. They are sufficient to
reject the current model as production-ready.

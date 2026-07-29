# Fruit-quality model and data license audit

This project is being prepared for a paid, closed-source enterprise delivery.
The checked-in weights are therefore **development-only** until the items below
are approved in writing by the customer/legal owner.

## Decision summary

| Asset | What was verified | Delivery decision |
|---|---|---|
| `models/best.onnx`, `models/best.pt` | ONNX metadata identifies an Ultralytics YOLOv8 model and `AGPL-3.0 License`. | **Blocked** for a closed-source delivery unless an Ultralytics Enterprise License is purchased and retained with the release record. |
| Roboflow source in `models/data.yaml` | Dataset URL is shown and the dataset declares CC BY 4.0. | Attribution and the exact dataset/version must be retained; this does not remove the Ultralytics model-license requirement. |
| `NararyaPutra/Freshness_Banana_Orange_Apple_Image-Classification` | Model card declares Apache-2.0 and MobileNetV3-Large, but its training-data provenance is not independently established. | Research candidate only; obtain written training-data permission before shipping. |
| `FruitNet` | Repository code is MIT; the referenced Zenodo fresh/rotten dataset is CC BY 4.0. It is classification-only and explicitly has no background-rejection class. | Potential candidate after reproducing evaluation and adding an object crop detector/rejector; preserve MIT/CC BY notices. |
| FruitVision dataset | Mendeley record declares CC BY-NC-ND 4.0. | **Do not use** for commercial training, fine-tuning, or redistribution. |

Authoritative references:

- [Ultralytics licensing and AGPL/Enterprise options](https://github.com/ultralytics/ultralytics)
- [Fruit-quality MobileNetV3 candidate](https://huggingface.co/NararyaPutra/Freshness_Banana_Orange_Apple_Image-Classification)
- [FruitNet repository](https://github.com/OleksandrKlanovets/fruitnet)
- [Fresh/rotten fruit dataset, Zenodo 4788775](https://zenodo.org/records/4788775)
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

The production launch files default to `model_license_approved:=false` and
refuse to start the detector in production mode. Simulation/development mode
can still use the checked-in weights for integration testing.

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

# Runtime fruit-quality model

This directory contains three checked-in runtime/development weights:

- `d455_apple_detector_v1.pt`: field-validated tabletop apple detector used by
  the human-gated fruit RViz script;
- `d455_apple_detector_v2.pt`: motion/light/edge adaptation selected by the
  general real-camera wrappers and automatic real run;
- `fruit_quality_mobilenet_v3.onnx`: tight-ROI fruit type/freshness classifier.

- Upstream: `NararyaPutra/Freshness_Banana_Orange_Apple_Image-Classification`
- Upstream revision: `31534dd9b3c74a0e6599867297327297c861999b`
- Architecture: torchvision MobileNetV3-Large, 224x224 RGB
- Upstream model-card license: Apache-2.0
- Upstream `model.pth` SHA-256:
  `3D81F2EBC6DC71620E89630D70A9FA9E775FD4DF40D25B7310DFB03A464E2973`
- Exported ONNX SHA-256:
  `DCCEF375B2272EB173D306FBE2F6263A091C097DD243FB0ECA013E5EFD16518C`
- Classes, in output order: `freshapples`, `freshbanana`, `freshoranges`,
  `rottenapples`, `rottenbanana`, `rottenoranges`.

The model is a crop classifier, not a detector. The runtime must first isolate
a fruit using the selected measured RGB-D localizer. The current apple project rejects all
non-apple, weak, ambiguous and incomplete-ROI results as `Unknown`.

The upstream card describes the training data only as a custom fruit-quality
dataset. Apache-2.0 metadata for the weight does not establish ownership of
those images. Enterprise production remains blocked until the dataset
provenance is approved in writing and a D455 held-out acceptance set passes.

## Apple detector evaluation candidate

The safe D455 debug wrapper downloads
`Shadyemad/s24-apple-detector` to ignored `artifacts/models/` and verifies
SHA-256 `66309c65f5b44bd5ec70efcc349f295bc74aa09ede0e1dfb20440cf34ebfe642`.
Its model card reports MIT, YOLOv8n, 640 px input and one `apple` detection
class. It is an evaluation candidate, not an accepted production model: its
training domain is synthetic/augmented orchard imagery, so it must pass the
D455 tabletop static/motion/empty-background acceptance set first.

## D455 tabletop hard-negative adaptation

The safe real-camera wrappers prefer the versioned runtime weight
`src/fruit_picking_arm/models/d455_apple_detector_v1.pt`, SHA-256
`A23975D92FA960E0674A37023AB1A2DCEB732AB1B68D635F2EFFEBC7C3244F7B`.
It is the candidate above fine-tuned with 189 D455 apple frames and 189 D455
empty-table/stool hard negatives (302 train, 76 validation). Training stopped
at epoch 9 and selected epoch 1. On two separately retained frames, the apple
scored 0.985 on the full image while the previous stool-edge false positive
produced no full-image candidate and only 0.041 maximum confidence across
overlapping crops. These are engineering checks, not final production
acceptance. A subsequent safe D455 run passed static Healthy detection and
stable camera-frame output, withdrew the stable target during removal/motion
blur, and then reported `raw_apple=0` for consecutive empty-background batches.
Rotten-apple acceptance remains outstanding because no rotten physical sample
was available.

## D455 motion/light/edge adaptation v2

The real-camera wrappers now prefer
`src/fruit_picking_arm/models/d455_apple_detector_v2.pt`, SHA-256
`E1917B61F008E996855D89BEA1138FE7420EEB244962D1AA07CBB806028D096E`.
It continues v1 training with deterministic motion blur, exposure/colour,
shadow/noise and combined motion-light variants while leaving validation
images untouched. The retained validation set reports precision 0.9986,
recall 1.0 and mAP50 0.995. On the separately captured difficult live frame
that scored 0.196 with v1, v2 scored 0.262; under synthetic 3--9 pixel motion
blur it retained 0.226--0.258 versus v1's 0.089--0.249. These figures support
runtime promotion but do not replace a conveyor-speed held-out acceptance run.

# Runtime fruit-quality model

This directory intentionally contains one runtime model:

`fruit_quality_mobilenet_v3.onnx`

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
a fruit using D455 point-cloud geometry. The current apple project rejects all
non-apple, weak, ambiguous and incomplete-ROI results as `Unknown`.

The upstream card describes the training data only as a custom fruit-quality
dataset. Apache-2.0 metadata for the weight does not establish ownership of
those images. Enterprise production remains blocked until the dataset
provenance is approved in writing and a D455 held-out acceptance set passes.

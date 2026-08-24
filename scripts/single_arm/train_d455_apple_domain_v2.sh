#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_DATA="${D455_APPLE_SOURCE_DATA:-${ROOT_DIR}/artifacts/datasets/d455_apple_v1/prepared}"
DOMAIN_DATA="${D455_APPLE_DOMAIN_DATA:-${ROOT_DIR}/artifacts/datasets/d455_apple_domain_v2}"
BASE_MODEL="${D455_APPLE_BASE_MODEL:-${ROOT_DIR}/src/fruit_picking_arm/models/d455_apple_detector_v1.pt}"
RUN_NAME="${D455_APPLE_RUN_NAME:-d455_apple_domain_v2}"

if [[ ! -d "${DOMAIN_DATA}" ]]; then
  python3 "${ROOT_DIR}/scripts/single_arm/prepare_d455_domain_augmented_dataset.py" \
    --source "${SOURCE_DATA}" \
    --output "${DOMAIN_DATA}"
fi

python3 - <<PY
from ultralytics import YOLO

model = YOLO(r"${BASE_MODEL}")
model.train(
    data=r"${DOMAIN_DATA}/data.yaml",
    epochs=24,
    patience=7,
    batch=16,
    imgsz=640,
    device=0,
    workers=4,
    cache="ram",
    project=r"${ROOT_DIR}/artifacts/training",
    name=r"${RUN_NAME}",
    exist_ok=False,
    pretrained=True,
    optimizer="AdamW",
    lr0=0.0005,
    lrf=0.05,
    weight_decay=0.0005,
    warmup_epochs=2.0,
    freeze=0,
    hsv_h=0.01,
    hsv_s=0.25,
    hsv_v=0.20,
    translate=0.15,
    scale=0.35,
    fliplr=0.5,
    mosaic=0.5,
    close_mosaic=5,
    mixup=0.0,
    cutmix=0.0,
    erasing=0.15,
    seed=22,
    deterministic=True,
)
PY

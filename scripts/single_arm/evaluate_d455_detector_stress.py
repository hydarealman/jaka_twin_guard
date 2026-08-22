#!/usr/bin/env python3
"""Compare apple detectors on untouched and deterministic stress variants."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from prepare_d455_domain_augmented_dataset import (
    combined,
    exposure_colour,
    motion_blur,
    seed_for,
    shadow_noise,
)


def ground_truth(label_path: Path, width: int, height: int):
    text = label_path.read_text(encoding="utf-8").strip() if label_path.exists() else ""
    if not text:
        return None
    _, cx, cy, box_width, box_height = map(float, text.splitlines()[0].split())
    return np.asarray(
        [
            (cx - box_width * 0.5) * width,
            (cy - box_height * 0.5) * height,
            (cx + box_width * 0.5) * width,
            (cy + box_height * 0.5) * height,
        ],
        dtype=np.float32,
    )


def iou(left: np.ndarray, right: np.ndarray) -> float:
    x1, y1 = np.maximum(left[:2], right[:2])
    x2, y2 = np.minimum(left[2:], right[2:])
    intersection = max(0.0, float(x2 - x1)) * max(0.0, float(y2 - y1))
    left_area = max(0.0, float(left[2] - left[0])) * max(
        0.0, float(left[3] - left[1])
    )
    right_area = max(0.0, float(right[2] - right[0])) * max(
        0.0, float(right[3] - right[1])
    )
    return intersection / max(1.0e-9, left_area + right_area - intersection)


def evaluate(model_path: Path, dataset: Path, device: str) -> None:
    images_dir = dataset / "images" / "val"
    labels_dir = dataset / "labels" / "val"
    paths = sorted(path for path in images_dir.glob("*") if path.is_file())
    transforms = {
        "original": lambda image, rng: image,
        "motion": motion_blur,
        "light": exposure_colour,
        "shadow": shadow_noise,
        "motion_light": combined,
    }
    model = YOLO(str(model_path))
    print(f"model={model_path}")
    for scenario, transform in transforms.items():
        samples = []
        truths = []
        for path in paths:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"cannot read {path}")
            rng = np.random.default_rng(seed_for(path) + 1009)
            samples.append(transform(image.copy(), rng))
            truths.append(
                ground_truth(labels_dir / f"{path.stem}.txt", image.shape[1], image.shape[0])
            )
        results = model.predict(
            samples,
            imgsz=640,
            conf=0.25,
            iou=0.5,
            max_det=1,
            device=device,
            verbose=False,
        )
        positives = false_negatives = negatives = false_positives = 0
        correct_scores = []
        for truth, result in zip(truths, results):
            boxes = result.boxes
            if truth is None:
                negatives += 1
                false_positives += int(boxes is not None and len(boxes) > 0)
                continue
            positives += 1
            if boxes is None or len(boxes) == 0:
                false_negatives += 1
                continue
            predicted = boxes.xyxy[0].detach().cpu().numpy()
            if iou(predicted, truth) < 0.30:
                false_negatives += 1
                continue
            correct_scores.append(float(boxes.conf[0].detach().cpu()))
        recall = 1.0 - false_negatives / max(1, positives)
        false_positive_rate = false_positives / max(1, negatives)
        mean_score = float(np.mean(correct_scores)) if correct_scores else 0.0
        print(
            f"{scenario:12s} recall={recall:.3f} mean_conf={mean_score:.3f} "
            f"empty_fp={false_positive_rate:.3f} "
            f"({false_positives}/{negatives})"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, action="append", required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    for model in args.model:
        evaluate(model.resolve(), args.dataset.resolve(), args.device)


if __name__ == "__main__":
    main()

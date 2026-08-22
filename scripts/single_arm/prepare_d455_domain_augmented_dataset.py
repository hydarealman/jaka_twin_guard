#!/usr/bin/env python3
"""Prepare deterministic motion/light augmentations without touching validation."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

import cv2
import numpy as np


def seed_for(path: Path) -> int:
    return int(hashlib.sha256(path.name.encode("utf-8")).hexdigest()[:8], 16)


def motion_blur(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    length = int(rng.choice([5, 7, 9, 11, 13]))
    angle = float(rng.uniform(0.0, 180.0))
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0
    rotation = cv2.getRotationMatrix2D(
        ((length - 1) * 0.5, (length - 1) * 0.5), angle, 1.0
    )
    kernel = cv2.warpAffine(kernel, rotation, (length, length))
    kernel /= max(1.0e-6, float(kernel.sum()))
    return cv2.filter2D(image, -1, kernel)


def exposure_colour(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    gamma = float(rng.choice([0.55, 0.70, 1.35, 1.65]))
    table = np.clip(
        ((np.arange(256, dtype=np.float32) / 255.0) ** gamma) * 255.0,
        0,
        255,
    ).astype(np.uint8)
    adjusted = cv2.LUT(image, table).astype(np.float32)
    # Independent channel gains approximate warm/cool industrial lighting.
    gains = np.asarray(
        [rng.uniform(0.78, 1.20), rng.uniform(0.88, 1.12), rng.uniform(0.78, 1.20)],
        dtype=np.float32,
    )
    return np.clip(adjusted * gains, 0, 255).astype(np.uint8)


def shadow_noise(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    height, width = image.shape[:2]
    mask = np.ones((height, width), dtype=np.float32)
    start = int(rng.uniform(0.15, 0.65) * width)
    end = min(width, start + int(rng.uniform(0.18, 0.45) * width))
    mask[:, start:end] *= float(rng.uniform(0.45, 0.75))
    softened = cv2.GaussianBlur(mask, (0, 0), sigmaX=max(3.0, width * 0.04))
    result = image.astype(np.float32) * softened[:, :, None]
    noise = rng.normal(0.0, rng.uniform(2.0, 7.0), image.shape)
    return np.clip(result + noise, 0, 255).astype(np.uint8)


def combined(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    return motion_blur(exposure_colour(image, rng), rng)


def copy_label(source: Path, destination: Path) -> None:
    if source.exists():
        shutil.copy2(source, destination)
    else:
        destination.touch()


def prepare(source: Path, destination: Path) -> None:
    if destination.exists():
        raise FileExistsError(
            f"destination already exists: {destination}; remove it explicitly first"
        )
    for split in ("train", "val"):
        (destination / "images" / split).mkdir(parents=True, exist_ok=True)
        (destination / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Validation remains visually untouched so augmentation cannot inflate it.
    for image_path in sorted((source / "images" / "val").glob("*")):
        if not image_path.is_file():
            continue
        shutil.copy2(image_path, destination / "images" / "val" / image_path.name)
        copy_label(
            source / "labels" / "val" / f"{image_path.stem}.txt",
            destination / "labels" / "val" / f"{image_path.stem}.txt",
        )

    transforms = {
        "motion": motion_blur,
        "light": exposure_colour,
        "shadow": shadow_noise,
        "motion_light": combined,
    }
    source_images = sorted((source / "images" / "train").glob("*"))
    for image_path in source_images:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"cannot read image: {image_path}")
        label_path = source / "labels" / "train" / f"{image_path.stem}.txt"
        original_name = image_path.name
        shutil.copy2(image_path, destination / "images" / "train" / original_name)
        copy_label(
            label_path,
            destination / "labels" / "train" / f"{image_path.stem}.txt",
        )
        rng = np.random.default_rng(seed_for(image_path))
        for suffix, transform in transforms.items():
            output_stem = f"{image_path.stem}__{suffix}"
            output_image = transform(image.copy(), rng)
            output_path = destination / "images" / "train" / f"{output_stem}.jpg"
            if not cv2.imwrite(str(output_path), output_image, [cv2.IMWRITE_JPEG_QUALITY, 94]):
                raise OSError(f"cannot write image: {output_path}")
            copy_label(
                label_path,
                destination / "labels" / "train" / f"{output_stem}.txt",
            )

    data_yaml = destination / "data.yaml"
    data_yaml.write_text(
        "path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: apple\n"
        % destination.as_posix(),
        encoding="utf-8",
    )
    print(
        "prepared train=%d val=%d at %s"
        % (
            len(list((destination / "images" / "train").glob("*"))),
            len(list((destination / "images" / "val").glob("*"))),
            destination,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()

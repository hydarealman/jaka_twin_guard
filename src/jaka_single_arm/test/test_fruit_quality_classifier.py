from pathlib import Path

import cv2
import numpy as np

from jaka_single_arm.perception.fruit_quality_classifier import (
    FruitQualityClassifier,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "fruit_quality_mobilenet_v3.onnx"


def test_model_contract_and_preprocess():
    classifier = FruitQualityClassifier(str(MODEL))
    roi = np.zeros((80, 60, 3), dtype=np.uint8)
    blob = classifier.preprocess(roi)
    assert blob.shape == (1, 3, 224, 224)
    assert blob.dtype == np.float32
    result = classifier.classify(roi)
    assert result is not None
    assert result.fruit_type in {"apple", "banana", "orange"}
    assert result.health in {"Healthy", "Unhealthy"}
    assert 0.0 <= result.confidence <= 1.0
    assert 0.0 <= result.margin <= 1.0


def test_real_d455_healthy_apple_roi_when_available():
    image_path = ROOT.parent.parent / ".codex_runtime" / "good_fruit_raw.jpg"
    if not image_path.is_file():
        return
    image = cv2.imread(str(image_path))
    assert image is not None
    roi = image[360:417, 345:397]
    result = FruitQualityClassifier(str(MODEL)).classify(roi)
    assert result is not None
    assert result.fruit_type == "apple"
    assert result.health == "Healthy"
    assert result.confidence >= 0.70

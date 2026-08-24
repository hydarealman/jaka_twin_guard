#!/usr/bin/env python3
"""Fruit-type and freshness classification for an already isolated fruit ROI.

This module deliberately does not scan a complete camera frame.  Fruit
localisation belongs to the depth/shape pipeline; this classifier only decides
the type and quality of a tightly cropped, geometrically validated fruit.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


LABELS = (
    "freshapples",
    "freshbanana",
    "freshoranges",
    "rottenapples",
    "rottenbanana",
    "rottenoranges",
)


@dataclass(frozen=True)
class FruitQuality:
    fruit_type: str
    health: str
    confidence: float
    margin: float
    raw_label: str
    probabilities: tuple[float, ...]


class FruitQualityClassifier:
    """ONNX Runtime wrapper for the Apache-2.0 MobileNetV3 candidate."""

    def __init__(self, model_path: str):
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(f"Fruit-quality model does not exist: {path}")
        import onnxruntime as ort

        # The debug node shares a small WSL VM with RealSense USB/IP, RViz and
        # DDS. ORT's default all-core thread pool can starve image callbacks
        # and make a healthy camera look disconnected. One inference thread is
        # sufficient at the 1-2 Hz annotation rate and keeps acquisition live.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self._session = ort.InferenceSession(
            str(path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        self._input_name = self._session.get_inputs()[0].name


    """
    接收裁剪好的水果图像 -> 预处理 -> 送入ONNX模型推理 -> 解码输出 -> 返回结构化的分类结果
    """
    def classify(self, roi_bgr: np.ndarray) -> FruitQuality | None:
        # 输入校验
        if roi_bgr is None or roi_bgr.size == 0:
            return None
        # 预处理(图像->模型输入张量)
        blob = self.preprocess(roi_bgr)
        # ONNX推理
        logits = np.asarray(
            self._session.run(None, {self._input_name: blob})[0],
            dtype=np.float32,
        ).reshape(-1)
        # Logits健康检查
        if logits.size != len(LABELS) or not np.isfinite(logits).all():
            return None
        # Softmax与排序
        probabilities = self.softmax(logits)
        winner = int(np.argmax(probabilities))
        runner = float(np.partition(probabilities, -2)[-2])
        # 标签解码与结果封装
        label = LABELS[winner]
        fruit_type = self.fruit_type(label)
        health = "Healthy" if label.startswith("fresh") else "Unhealthy"
        return FruitQuality(
            fruit_type=fruit_type,
            health=health,
            confidence=float(probabilities[winner]),
            margin=float(probabilities[winner] - runner),
            raw_label=label,
            probabilities=tuple(float(value) for value in probabilities),
        )


    @staticmethod
    def preprocess(roi_bgr: np.ndarray) -> np.ndarray:
        # Follow the model card exactly: RGB, resize 224x224, ImageNet stats.
        rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_LINEAR)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = (tensor - np.array([0.485, 0.456, 0.406], np.float32)) / np.array(
            [0.229, 0.224, 0.225], np.float32
        )
        return np.transpose(tensor, (2, 0, 1))[None, ...]

    @staticmethod
    def softmax(logits: np.ndarray) -> np.ndarray:
        shifted = logits - float(np.max(logits))
        exponents = np.exp(shifted)
        return exponents / float(np.sum(exponents))

    @staticmethod
    def fruit_type(label: str) -> str:
        if "apple" in label:
            return "apple"
        if "banana" in label:
            return "banana"
        if "orange" in label:
            return "orange"
        return "unknown"

#!/usr/bin/env python3
"""YOLOv8 苹果好坏识别 —— 纯推理引擎（与 ROS 解耦，便于单元测试）。

本模块把队友 `ros2IdentifyApple/src/rm_identify/src/identify.cpp` 中的
`YOLOV8::preprocess_img / postprocess_img` 算法用 numpy/opencv 忠实重写为
原生 Python，去除 OpenVINO / ROS1 / 串口等依赖。模型本身（best.onnx / best.pt）
原样沿用，不重新训练。

两个可切换后端：
  - onnxruntime : 加载 best.onnx，手写 letterbox 预处理 + YOLOv8 解码 + NMS
  - ultralytics : 加载 best.pt，调用 ultralytics.YOLO 推理

类别（来自 model/data.yaml）：
  0 = Healthy   (好苹果)
  1 = Unhealthy (坏苹果)
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

try:  # opencv 仅用于 resize / 画框，非核心算法
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


DEFAULT_CLASS_NAMES = ["Healthy", "Unhealthy"]


@dataclass
class Detection:
    """单个检测结果（图像像素坐标，与原始输入图同尺度）。"""
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def xywh(self) -> tuple[float, float, float, float]:
        return (self.x1, self.y1, self.x2 - self.x1, self.y2 - self.y1)

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def is_healthy(self) -> bool:
        return self.class_id == 0


def load_class_names(data_yaml: str | None) -> list[str]:
    """从 data.yaml 读取类别名，失败时回退默认。"""
    if data_yaml and os.path.isfile(data_yaml):
        try:
            import yaml
            with open(data_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            names = data.get("names")
            if isinstance(names, dict):  # {0: 'Healthy', 1: 'Unhealthy'}
                names = [names[k] for k in sorted(names)]
            if names:
                return list(names)
        except Exception:
            pass
    return list(DEFAULT_CLASS_NAMES)


class YoloDetector:
    """YOLOv8 双后端推理器。

    用法::
        det = YoloDetector(backend="onnxruntime",
                           onnx_path=".../best.onnx",
                           data_yaml=".../data.yaml")
        dets = det.infer(bgr_image)   # -> list[Detection]
    """

    def __init__(
        self,
        backend: str = "onnxruntime",
        onnx_path: str | None = None,
        pt_path: str | None = None,
        data_yaml: str | None = None,
        conf_threshold: float = 0.25,
        nms_threshold: float = 0.45,
        input_size: int = 640,
        logger=None,
    ):
        self._logger = logger
        self._conf = float(conf_threshold)
        self._nms = float(nms_threshold)
        self._size = int(input_size)
        self._onnx_path = onnx_path
        self._pt_path = pt_path
        self.class_names = load_class_names(data_yaml)

        self._session = None       # onnxruntime.InferenceSession
        self._input_name = None
        self._yolo = None          # ultralytics.YOLO
        self.backend = self._init_backend(backend)

    # ── 后端初始化（含自动回退）─────────────────────────────

    def _log(self, msg: str, level: str = "info") -> None:
        if self._logger is not None:
            getattr(self._logger, level, self._logger.info)(msg)
        else:
            print(f"[YoloDetector] {msg}")

    def _init_backend(self, backend: str) -> str:
        backend = (backend or "onnxruntime").lower()
        order = [backend] + [b for b in ("onnxruntime", "ultralytics") if b != backend]
        for b in order:
            try:
                if b == "onnxruntime":
                    self._init_onnx()
                    self._log(f"推理后端: onnxruntime ({self._onnx_path})")
                    return "onnxruntime"
                elif b == "ultralytics":
                    self._init_ultralytics()
                    self._log(f"推理后端: ultralytics ({self._pt_path})")
                    return "ultralytics"
            except Exception as e:
                self._log(f"后端 '{b}' 初始化失败: {e}", "warning")
        raise RuntimeError(
            "无可用推理后端。请安装 onnxruntime 或 ultralytics："
            "pip install onnxruntime opencv-python  (可选 ultralytics)"
        )

    def _init_onnx(self) -> None:
        if not self._onnx_path or not os.path.isfile(self._onnx_path):
            raise FileNotFoundError(f"ONNX 模型不存在: {self._onnx_path}")
        import onnxruntime as ort
        providers = ["CPUExecutionProvider"]
        self._session = ort.InferenceSession(self._onnx_path, providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        ishape = self._session.get_inputs()[0].shape
        # 尝试从模型输入形状推断方形输入尺寸（NCHW）
        try:
            h = int(ishape[2]); w = int(ishape[3])
            if h > 0 and w > 0:
                self._size = max(h, w)
        except Exception:
            pass

    def _init_ultralytics(self) -> None:
        if not self._pt_path or not os.path.isfile(self._pt_path):
            raise FileNotFoundError(f"PT 模型不存在: {self._pt_path}")
        from ultralytics import YOLO
        self._yolo = YOLO(self._pt_path)
        # ultralytics 自带类别名
        names = getattr(self._yolo, "names", None)
        if names:
            if isinstance(names, dict):
                names = [names[k] for k in sorted(names)]
            self.class_names = list(names)

    # ── 对外推理接口 ───────────────────────────────────────

    def infer(self, image_bgr: np.ndarray) -> list[Detection]:
        """对一张 BGR 图做检测，返回像素坐标下的检测列表。"""
        if image_bgr is None or image_bgr.size == 0:
            return []
        if self.backend == "onnxruntime":
            return self._infer_onnx(image_bgr)
        return self._infer_ultralytics(image_bgr)

    # ── onnxruntime 路径（忠实移植 identify.cpp）───────────

    def _letterbox(self, image_bgr: np.ndarray):
        """等比缩放 + 右/下补边到方形（对应 C++ preprocess_img）。

        返回 (padded_bgr, r)，其中 r = 缩放比例；框可用 1/r 还原到原图。
        """
        h, w = image_bgr.shape[:2]
        r = self._size / max(w, h)
        new_w, new_h = int(round(w * r)), int(round(h * r))
        interp = cv2.INTER_LINEAR
        resized = cv2.resize(image_bgr, (new_w, new_h), interpolation=interp)
        dw, dh = self._size - new_w, self._size - new_h
        padded = cv2.copyMakeBorder(
            resized, 0, dh, 0, dw, cv2.BORDER_CONSTANT, value=(100, 100, 100)
        )
        return padded, r

    def _infer_onnx(self, image_bgr: np.ndarray) -> list[Detection]:
        h0, w0 = image_bgr.shape[:2]
        padded, r = self._letterbox(image_bgr)

        # BGR→RGB, /255, HWC→CHW, 加 batch 维（ultralytics 导出 ONNX 的标准输入）
        blob = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]

        outputs = self._session.run(None, {self._input_name: blob})
        preds = outputs[0]  # 期望 [1, 4+nc, N] 或 [1, N, 4+nc]

        dets = self._decode_yolov8(preds, r, w0, h0)
        return dets

    def _decode_yolov8(self, preds: np.ndarray, r: float,
                       w0: int, h0: int) -> list[Detection]:
        nc = len(self.class_names)
        pred = np.squeeze(preds, axis=0) if preds.ndim == 3 else preds
        # 归一到 [N, 4+nc]
        if pred.shape[0] == 4 + nc and pred.shape[1] != 4 + nc:
            pred = pred.T  # [4+nc, N] → [N, 4+nc]
        elif pred.shape[1] != 4 + nc and pred.shape[0] == 4 + nc:
            pred = pred.T

        boxes = pred[:, :4]
        scores_all = pred[:, 4:4 + nc]
        class_ids = np.argmax(scores_all, axis=1)
        confidences = scores_all[np.arange(scores_all.shape[0]), class_ids]

        keep = confidences > self._conf
        if not np.any(keep):
            return []
        boxes = boxes[keep]
        class_ids = class_ids[keep]
        confidences = confidences[keep]

        # cx,cy,w,h → x1,y1,x2,y2（letterbox 坐标）
        cx, cy, ww, hh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = cx - ww / 2.0
        y1 = cy - hh / 2.0
        x2 = cx + ww / 2.0
        y2 = cy + hh / 2.0
        xyxy = np.stack([x1, y1, x2, y2], axis=1)

        idxs = self._nms_boxes(xyxy, confidences, self._nms)

        dets: list[Detection] = []
        for i in idxs:
            # 还原到原图尺度（padding 在右/下，原点不变，直接除以 r）
            bx1, by1, bx2, by2 = xyxy[i] / r
            bx1 = float(np.clip(bx1, 0, w0 - 1))
            by1 = float(np.clip(by1, 0, h0 - 1))
            bx2 = float(np.clip(bx2, 0, w0 - 1))
            by2 = float(np.clip(by2, 0, h0 - 1))
            cid = int(class_ids[i])
            name = self.class_names[cid] if 0 <= cid < len(self.class_names) else str(cid)
            dets.append(Detection(cid, name, float(confidences[i]), bx1, by1, bx2, by2))
        return dets

    @staticmethod
    def _nms_boxes(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> list[int]:
        """纯 numpy 全类别 NMS（对应 cv::dnn::NMSBoxes）。"""
        if len(boxes) == 0:
            return []
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
            order = order[1:][iou <= iou_thr]
        return keep

    # ── ultralytics 路径 ──────────────────────────────────

    def _infer_ultralytics(self, image_bgr: np.ndarray) -> list[Detection]:
        results = self._yolo.predict(
            source=image_bgr, conf=self._conf, iou=self._nms,
            imgsz=self._size, verbose=False,
        )
        dets: list[Detection] = []
        if not results:
            return dets
        boxes = results[0].boxes
        if boxes is None:
            return dets
        for b in boxes:
            cid = int(b.cls[0])
            conf = float(b.conf[0])
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            name = self.class_names[cid] if 0 <= cid < len(self.class_names) else str(cid)
            dets.append(Detection(cid, name, conf, x1, y1, x2, y2))
        return dets

    # ── 可视化 ─────────────────────────────────────────────

    def draw(self, image_bgr: np.ndarray, dets: list[Detection]) -> np.ndarray:
        """在图上画检测框（好=绿，坏=红），返回标注后的 BGR 图。"""
        if cv2 is None:
            return image_bgr
        out = image_bgr.copy()
        good = bad = 0
        for d in dets:
            color = (0, 200, 0) if d.is_healthy else (0, 0, 255)
            if d.is_healthy:
                good += 1
            else:
                bad += 1
            p1 = (int(d.x1), int(d.y1))
            p2 = (int(d.x2), int(d.y2))
            cv2.rectangle(out, p1, p2, color, 2)
            label = f"{d.class_name} {d.confidence * 100:.1f}%"
            ty = max(int(d.y1) - 8, 20)
            cv2.putText(out, label, (int(d.x1), ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cx, cy = d.center
            cv2.circle(out, (int(cx), int(cy)), 4, color, -1)
        summary = f"Healthy: {good}  Unhealthy: {bad}"
        cv2.putText(out, summary, (15, out.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return out

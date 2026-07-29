#!/usr/bin/env python3
"""Evaluate the bundled apple quality detector on downloaded test videos.

This is an offline, frame-level smoke test rather than a substitute for a
labelled benchmark: the source clips have a scene-level label, not boxes for
every frame.  The script records empty/wrong/mixed detections and saves a
small set of annotated frames for manual review.

The input videos are intentionally kept outside the package models directory.
They are third-party test material and should not be redistributed with the
enterprise deliverable.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2


def _repo_src() -> Path:
    # test/evaluate_fruit_video.py -> jaka_single_arm/ -> src/jaka_single_arm
    return Path(__file__).resolve().parents[1]


def _classify_frame(detections, expected: str) -> str:
    names = [str(d.class_name) for d in detections]
    has_expected = expected in names
    opposite = "Unhealthy" if expected == "Healthy" else "Healthy"
    has_opposite = opposite in names
    if not names:
        return "empty"
    if has_expected and has_opposite:
        return "mixed"
    if has_expected:
        return "hit"
    return "wrong"


def evaluate_video(detector, path: Path, expected: str, output_dir: Path,
                   sample_seconds: float, max_frames: int) -> dict:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open video: {path}")

    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    duration = frame_count / fps if fps > 0 else None
    stride = max(1, int(round((fps or 25.0) * sample_seconds)))
    stats = {
        "video": str(path),
        "expected": expected,
        "fps": fps,
        "frame_count": frame_count,
        "duration_seconds": duration,
        "sample_seconds": sample_seconds,
        "sampled_frames": 0,
        "hit_frames": 0,
        "wrong_frames": 0,
        "empty_frames": 0,
        "mixed_frames": 0,
        "detections": 0,
        "expected_detections": 0,
        "opposite_detections": 0,
        "duplicate_frames": 0,
        "confidence_sum": 0.0,
        "latency_ms_sum": 0.0,
        "samples": [],
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    index = 0
    saved = 0
    while saved < max_frames:
        ok, frame = capture.read()
        if not ok:
            break
        if index % stride:
            index += 1
            continue
        index += 1
        start = time.perf_counter()
        detections = detector.infer(frame)
        latency_ms = (time.perf_counter() - start) * 1000.0
        state = _classify_frame(detections, expected)
        stats["sampled_frames"] += 1
        stats[f"{state}_frames"] += 1
        stats["detections"] += len(detections)
        stats["expected_detections"] += sum(
            1 for detection in detections if detection.class_name == expected
        )
        stats["opposite_detections"] += sum(
            1 for detection in detections if detection.class_name != expected
        )
        stats["confidence_sum"] += sum(float(d.confidence) for d in detections)
        stats["latency_ms_sum"] += latency_ms
        if len(detections) > 1:
            stats["duplicate_frames"] += 1

        if saved < 6:
            annotated = detector.draw(frame, detections)
            output_path = output_dir / f"{path.stem}_{saved:02d}_{state}.jpg"
            cv2.imwrite(str(output_path), annotated)
            stats["samples"].append({
                "frame_index": index - 1,
                "state": state,
                "detections": [
                    {
                        "class": d.class_name,
                        "confidence": round(float(d.confidence), 5),
                        "xyxy": [round(float(v), 1) for v in
                                  (d.x1, d.y1, d.x2, d.y2)],
                    }
                    for d in detections
                ],
                "annotated": str(output_path),
            })
        saved += 1

    capture.release()
    n = stats["sampled_frames"] or 1
    stats["hit_rate"] = stats["hit_frames"] / n
    stats["wrong_rate"] = stats["wrong_frames"] / n
    stats["empty_rate"] = stats["empty_frames"] / n
    stats["mixed_rate"] = stats["mixed_frames"] / n
    stats["mean_latency_ms"] = stats["latency_ms_sum"] / n
    stats["mean_confidence"] = (
        stats["confidence_sum"] / stats["detections"]
        if stats["detections"] else 0.0
    )
    return stats


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", action="append", required=True,
                        help="video path; may be repeated")
    parser.add_argument("--expected", action="append", required=True,
                        choices=["Healthy", "Unhealthy"],
                        help="scene label corresponding to each --video")
    parser.add_argument("--output-dir", default="artifacts/web_eval/results")
    parser.add_argument("--sample-seconds", type=float, default=0.5)
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()
    if len(args.video) != len(args.expected):
        parser.error("--video and --expected must have the same count")

    sys.path.insert(0, str(_repo_src()))
    from jaka_single_arm.perception.yolo_infer import YoloDetector

    models = _repo_src() / "models"
    detector = YoloDetector(
        backend="onnxruntime",
        onnx_path=str(models / "best.onnx"),
        data_yaml=str(models / "data.yaml"),
        conf_threshold=args.conf,
    )
    output_dir = Path(args.output_dir)
    results = [evaluate_video(
        detector, Path(video), expected, output_dir,
        args.sample_seconds, args.max_frames,
    ) for video, expected in zip(args.video, args.expected)]
    output_dir.mkdir(parents=True, exist_ok=True)
    report = output_dir / "video_eval.json"
    report.write_text(json.dumps(results, ensure_ascii=False, indent=2),
                      encoding="utf-8")
    for result in results:
        print(
            f"{Path(result['video']).name}: expected={result['expected']} "
            f"frames={result['sampled_frames']} hit={result['hit_rate']:.1%} "
            f"wrong={result['wrong_rate']:.1%} empty={result['empty_rate']:.1%} "
            f"mixed={result['mixed_rate']:.1%} "
            f"detections={result['detections']} "
            f"mean_conf={result['mean_confidence']:.3f} "
            f"latency={result['mean_latency_ms']:.1f}ms"
        )
    print(f"VIDEO_EVAL_REPORT: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

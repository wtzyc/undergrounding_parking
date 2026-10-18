#!/usr/bin/env python3
"""Run local Qwen2.5-VL parking-slot detection on a directory of images.

The module is intentionally independent of ROS. It asks the VLM for parking-slot
bounding boxes, parses JSON or Qwen grounding tokens, and writes annotated images
plus machine-readable results.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any, Iterable


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
PROMPT_TEMPLATE = """你是停车场 BEV 图像标注器。请只检测图中清晰可见的停车位区域，不要检测汽车、柱子、道路或阴影。

图像原始尺寸是 {width} x {height} 像素。坐标原点在左上角，x 向右，y 向下。
请给出每个停车位的轴对齐矩形框，格式为 [x1, y1, x2, y2]，坐标必须是原图像素坐标。
如果一个停车位被遮挡、边界不清楚或不能确定，请不要输出它。

只返回下面这种 JSON，不要添加 Markdown、解释文字或代码围栏：
{{
  "image_width": {width},
  "image_height": {height},
  "slots": [
    {{"box": [x1, y1, x2, y2], "label": "parking_slot", "confidence": 0.0}}
  ]
}}
没有确定的停车位时返回 {{"image_width": {width}, "image_height": {height}, "slots": []}}。"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch parking-slot detection with a local VLM")
    parser.add_argument("--input-dir", type=Path, default=Path("images"))
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument("--model-path", required=True, help="Local model directory or Hugging Face model id")
    parser.add_argument("--max-new-tokens", type=int, default=512)
    parser.add_argument("--coordinate-scale", choices=("auto", "pixels", "1000"), default="auto")
    parser.add_argument("--limit", type=int, default=0, help="Process at most N images; 0 means all")
    return parser.parse_args()


def import_vlm_dependencies():
    try:
        import cv2
        import torch
        from PIL import Image
        from qwen_vl_utils import process_vision_info
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
    except ImportError as exc:
        raise RuntimeError(
            "导入 VLM 依赖失败："
            f"{exc}。请在本目录执行 python -m pip install -r requirements.txt；"
            "如果错误提到 NumPy/SciPy 版本不匹配，请升级虚拟环境里的 SciPy。"
        ) from exc
    return cv2, torch, Image, process_vision_info, AutoProcessor, Qwen2_5_VLForConditionalGeneration


def load_vlm(model_path: str, AutoProcessor: Any, ModelClass: Any):
    print(f"[INFO] loading model: {model_path}", flush=True)
    try:
        model = ModelClass.from_pretrained(
            model_path,
            torch_dtype="auto",
            device_map="auto",
        )
        processor = AutoProcessor.from_pretrained(
            model_path,
            min_pixels=256 * 28 * 28,
            max_pixels=1280 * 28 * 28,
        )
    except Exception as exc:
        raise RuntimeError(
            f"无法加载模型 {model_path}。请确认模型目录完整，并且显存/依赖满足要求。原始错误：{exc}"
        ) from exc
    model.eval()
    return model, processor


def image_files(input_dir: Path, limit: int) -> list[Path]:
    if not input_dir.is_dir():
        raise FileNotFoundError(f"input directory does not exist: {input_dir}")
    files = sorted(p for p in input_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)
    if limit > 0:
        files = files[:limit]
    return files


def flatten_numbers(value: Any) -> list[float] | None:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return [float(value)]
    if isinstance(value, (list, tuple)):
        output: list[float] = []
        for item in value:
            numbers = flatten_numbers(item)
            if numbers is None:
                return None
            output.extend(numbers)
        return output
    return None


def extract_json_object(text: str) -> dict[str, Any] | None:
    cleaned = text.strip()
    cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned, flags=re.IGNORECASE)
    cleaned = re.sub(r"\s*```$", "", cleaned)
    try:
        value = json.loads(cleaned)
        return value if isinstance(value, dict) else None
    except json.JSONDecodeError:
        pass

    # The model occasionally prefixes or suffixes the JSON. Select the largest
    # object-looking substring and try it without accepting arbitrary text as JSON.
    starts = [m.start() for m in re.finditer(r"\{", cleaned)]
    ends = [m.end() for m in re.finditer(r"\}", cleaned)]
    for start in starts:
        for end in reversed(ends):
            if end <= start:
                continue
            try:
                value = json.loads(cleaned[start:end])
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                return value
    return None


def parse_grounding_tokens(text: str) -> list[list[float]]:
    # Qwen grounding answers may use <|box_start|>(x1,y1),(x2,y2)<|box_end|>.
    pattern = re.compile(
        r"<\|box_start\|>\s*\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?"
        r"\s*,\s*\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?\s*<\|box_end\|>",
        flags=re.IGNORECASE,
    )
    return [[float(group) for group in match.groups()] for match in pattern.finditer(text)]


def raw_box_candidates(text: str) -> list[tuple[list[float], dict[str, Any]]]:
    parsed = extract_json_object(text)
    candidates: list[tuple[list[float], dict[str, Any]]] = []
    if parsed is not None:
        slots = parsed.get("slots", parsed.get("parking_slots", []))
        if isinstance(slots, list):
            for slot in slots:
                if not isinstance(slot, dict):
                    continue
                value = slot.get("box", slot.get("bbox", slot.get("bounding_box")))
                numbers = flatten_numbers(value)
                if numbers is not None and len(numbers) == 4:
                    candidates.append((numbers, slot))

    if candidates:
        return candidates

    return [([*box], {"label": "parking_slot", "confidence": None}) for box in parse_grounding_tokens(text)]


def convert_box(numbers: Iterable[float], width: int, height: int, coordinate_scale: str) -> list[int] | None:
    values = list(numbers)
    if len(values) != 4:
        return None
    x1, y1, x2, y2 = values
    if coordinate_scale == "1000" or (
        coordinate_scale == "auto" and (max(x1, x2) > width or max(y1, y2) > height)
    ):
        x1, x2 = x1 * width / 1000.0, x2 * width / 1000.0
        y1, y2 = y1 * height / 1000.0, y2 * height / 1000.0
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    x1 = max(0, min(width - 1, round(x1)))
    y1 = max(0, min(height - 1, round(y1)))
    x2 = max(0, min(width - 1, round(x2)))
    y2 = max(0, min(height - 1, round(y2)))
    if x2 - x1 < 10 or y2 - y1 < 10:
        return None
    return [x1, y1, x2, y2]


def parse_slots(text: str, width: int, height: int, coordinate_scale: str) -> list[dict[str, Any]]:
    slots: list[dict[str, Any]] = []
    for numbers, metadata in raw_box_candidates(text):
        box = convert_box(numbers, width, height, coordinate_scale)
        if box is None:
            continue
        confidence = metadata.get("confidence") if isinstance(metadata, dict) else None
        try:
            confidence = float(confidence) if confidence is not None else None
        except (TypeError, ValueError):
            confidence = None
        slots.append(
            {
                "box": box,
                "label": str(metadata.get("label", "parking_slot")),
                "confidence": confidence,
            }
        )
    return slots


def ask_model(model: Any, processor: Any, process_vision_info: Any, torch: Any, image_path: Path, width: int, height: int, max_new_tokens: int) -> str:
    prompt = PROMPT_TEMPLATE.format(width=width, height=height)
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": f"file://{image_path.resolve()}"},
                {"type": "text", "text": prompt},
            ],
        }
    ]
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(
        text=[text],
        images=image_inputs,
        videos=video_inputs,
        padding=True,
        return_tensors="pt",
    )
    device = next(model.parameters()).device
    inputs = {key: value.to(device) if hasattr(value, "to") else value for key, value in inputs.items()}
    with torch.inference_mode():
        generated_ids = model.generate(**inputs, max_new_tokens=max_new_tokens)
    generated_ids_trimmed = [
        output_ids[len(input_ids):] for input_ids, output_ids in zip(inputs["input_ids"], generated_ids)
    ]
    return processor.batch_decode(generated_ids_trimmed, skip_special_tokens=False, clean_up_tokenization_spaces=False)[0]


def draw_boxes(cv2: Any, image_path: Path, output_path: Path, slots: list[dict[str, Any]]) -> None:
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"cannot read image: {image_path}")
    for index, slot in enumerate(slots, start=1):
        x1, y1, x2, y2 = slot["box"]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        confidence = slot.get("confidence")
        suffix = f" {confidence:.2f}" if isinstance(confidence, float) else ""
        label = f"slot-{index}{suffix}"
        baseline_y = max(24, y1 - 8)
        cv2.putText(image, label, (x1, baseline_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"cannot write image: {output_path}")


def main() -> int:
    args = parse_args()
    try:
        cv2, torch, Image, process_vision_info, AutoProcessor, ModelClass = import_vlm_dependencies()
        files = image_files(args.input_dir, args.limit)
        if not files:
            raise FileNotFoundError(f"no supported images found in {args.input_dir}")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        model, processor = load_vlm(args.model_path, AutoProcessor, ModelClass)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    all_results: list[dict[str, Any]] = []
    for index, image_path in enumerate(files, start=1):
        print(f"[{index}/{len(files)}] {image_path.name}", flush=True)
        started = time.time()
        result: dict[str, Any] = {
            "image": image_path.name,
            "model": args.model_path,
            "slots": [],
        }
        try:
            with Image.open(image_path) as image:
                width, height = image.size
            raw_text = ask_model(model, processor, process_vision_info, torch, image_path, width, height, args.max_new_tokens)
            slots = parse_slots(raw_text, width, height, args.coordinate_scale)
            result.update({
                "image_width": width,
                "image_height": height,
                "slots": slots,
                "elapsed_seconds": round(time.time() - started, 3),
            })
            if not slots:
                (args.output_dir / f"{image_path.stem}.raw.txt").write_text(raw_text, encoding="utf-8")
            output_image = args.output_dir / f"{image_path.stem}_boxes.jpg"
            draw_boxes(cv2, image_path, output_image, slots)
            print(f"  detected {len(slots)} slot(s); wrote {output_image.name}", flush=True)
        except Exception as exc:
            result["error"] = str(exc)
            print(f"  [ERROR] {exc}", file=sys.stderr, flush=True)
        (args.output_dir / f"{image_path.stem}.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        all_results.append(result)

    summary = {
        "schema_version": "vlm-parking-slot-results-v1",
        "model": args.model_path,
        "images": all_results,
    }
    (args.output_dir / "all_results.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"[INFO] finished: {len(all_results)} image(s), output={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

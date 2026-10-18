#!/usr/bin/env python3
"""Run classical computer-vision parking-slot detection on BEV images."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import cv2

from traditional_parking_detector import (
    DetectorConfig,
    detect_slots,
    draw_debug_image,
)


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect Gazebo parking bays with color, marking-line, and geometry cues"
    )
    parser.add_argument("--input-dir", type=Path, default=Path("images"))
    parser.add_argument("--output-dir", type=Path, default=Path("results/traditional"))
    parser.add_argument("--limit", type=int, default=0, help="Process at most N images; 0 means all")
    return parser.parse_args()


def image_files(input_dir: Path, limit: int) -> list[Path]:
    if not input_dir.is_dir():
        raise FileNotFoundError(f"input directory does not exist: {input_dir}")
    files = sorted(
        path
        for path in input_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )
    return files[:limit] if limit > 0 else files


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    try:
        files = image_files(args.input_dir, args.limit)
        if not files:
            raise FileNotFoundError(f"no supported images found in {args.input_dir}")
        args.output_dir.mkdir(parents=True, exist_ok=True)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    config = DetectorConfig()
    all_results: list[dict[str, Any]] = []
    for index, image_path in enumerate(files, start=1):
        print(f"[{index}/{len(files)}] {image_path.name}", flush=True)
        result: dict[str, Any] = {
            "image": image_path.name,
            "detector": "gazebo_traditional_color_line_geometry",
            "slots": [],
            "rejected_candidates": [],
        }
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            result["error"] = "could not read image"
            print(f"  [ERROR] could not read {image_path}", file=sys.stderr, flush=True)
        else:
            height, width = image.shape[:2]
            try:
                accepted, rejected, slot_mask, line_mask = detect_slots(image, config)
                for slot_id, slot in enumerate(accepted, start=1):
                    slot["slot_id"] = slot_id
                debug = draw_debug_image(image, accepted, rejected)

                files_to_write = {
                    f"{image_path.stem}_traditional.jpg": debug,
                    f"{image_path.stem}_slot_mask.png": slot_mask,
                    f"{image_path.stem}_white_lines.png": line_mask,
                }
                for filename, output_image in files_to_write.items():
                    if not cv2.imwrite(str(args.output_dir / filename), output_image):
                        raise RuntimeError(f"could not write {filename}")

                result.update(
                    {
                        "image_width": width,
                        "image_height": height,
                        "slots": accepted,
                        "rejected_candidates": rejected,
                        "debug_image": f"{image_path.stem}_traditional.jpg",
                        "slot_mask_image": f"{image_path.stem}_slot_mask.png",
                        "white_line_mask_image": f"{image_path.stem}_white_lines.png",
                    }
                )
                print(
                    f"  accepted {len(accepted)} slot(s), "
                    f"rejected {len(rejected)} candidate(s)",
                    flush=True,
                )
            except Exception as exc:
                result["error"] = str(exc)
                print(f"  [ERROR] {exc}", file=sys.stderr, flush=True)

        write_json(args.output_dir / f"{image_path.stem}.json", result)
        all_results.append(result)

    summary = {
        "schema_version": "traditional-parking-slot-results-v1",
        "method": "blue-gray surface components validated by white-line support and rectangle geometry",
        "coordinate_frame": "bev_pixel",
        "images": all_results,
    }
    write_json(args.output_dir / "all_results.json", summary)
    print(f"[INFO] finished: {len(all_results)} image(s), output={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

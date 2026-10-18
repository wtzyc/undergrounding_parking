"""Classical BEV parking-slot detector for the fixed Gazebo scene.

The scene renders parking bays blue-gray, the driving lane green, and the bay
markings white. This module uses those image cues and geometric checks; it does
not load a learned model or depend on ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    """Thresholds for the current Gazebo BEV materials and slot geometry."""

    min_blue_red_difference: int = 8
    min_blue_green_difference: int = 5
    min_blue_value: int = 25
    min_white_value: int = 180
    max_white_chroma: int = 40
    min_component_area_px: int = 5000
    min_rectangularity: float = 0.55
    min_aspect_ratio: float = 1.15
    max_aspect_ratio: float = 3.2
    expected_short_side_px: float = 140.0
    short_side_tolerance_fraction: float = 0.30
    min_long_side_px: float = 150.0
    max_long_side_px: float = 380.0
    line_probe_offsets_px: tuple[int, ...] = (-6, -4, -2, 0, 2, 4, 6)
    min_line_support: float = 0.35
    min_supported_edges: int = 2


def build_masks(
    image_bgr: np.ndarray,
    config: DetectorConfig = DetectorConfig(),
) -> tuple[np.ndarray, np.ndarray]:
    """Return masks for blue-gray parking surfaces and white paint markings."""
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image_bgr must be a three-channel BGR image")
    if image_bgr.dtype != np.uint8:
        raise ValueError("image_bgr must use uint8 pixel values")

    b, g, r = cv2.split(image_bgr.astype(np.int16))
    slot_mask = (
        (b - r > config.min_blue_red_difference)
        & (b - g > config.min_blue_green_difference)
        & (b > config.min_blue_value)
    ).astype(np.uint8) * 255

    max_channel = np.max(image_bgr, axis=2)
    min_channel = np.min(image_bgr, axis=2)
    white_mask = (
        (max_channel >= config.min_white_value)
        & ((max_channel.astype(np.int16) - min_channel.astype(np.int16)) <= config.max_white_chroma)
    ).astype(np.uint8) * 255
    return slot_mask, white_mask


def _order_corners(corners: np.ndarray) -> np.ndarray:
    center = np.mean(corners, axis=0)
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    ordered = corners[np.argsort(angles)]
    first = int(np.lexsort((ordered[:, 0], ordered[:, 1]))[0])
    return np.roll(ordered, -first, axis=0)


def _edge_line_support(
    corners: np.ndarray,
    white_mask: np.ndarray,
    offsets: tuple[int, ...],
) -> list[float]:
    height, width = white_mask.shape
    support: list[float] = []
    for index in range(4):
        start = corners[index]
        end = corners[(index + 1) % 4]
        vector = end - start
        length = float(np.linalg.norm(vector))
        sample_count = max(12, int(round(length)))
        points = np.linspace(start, end, sample_count, endpoint=True)
        normal = np.array([-vector[1], vector[0]], dtype=np.float32) / max(length, 1e-6)
        marked = np.zeros(sample_count, dtype=bool)
        valid_samples = np.zeros(sample_count, dtype=bool)

        for offset in offsets:
            shifted = np.rint(points + normal * offset).astype(np.int32)
            valid = (
                (shifted[:, 0] >= 0)
                & (shifted[:, 0] < width)
                & (shifted[:, 1] >= 0)
                & (shifted[:, 1] < height)
            )
            valid_samples |= valid
            marked[valid] |= white_mask[shifted[valid, 1], shifted[valid, 0]] > 0

        count = int(np.count_nonzero(valid_samples))
        support.append(
            float(np.count_nonzero(marked & valid_samples) / count) if count else 0.0
        )
    return support


def detect_slots(
    image_bgr: np.ndarray,
    config: DetectorConfig = DetectorConfig(),
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], np.ndarray, np.ndarray]:
    """Detect rotated bay polygons and return accepted and rejected candidates.

    Each polygon approximates the visible blue-gray bay surface. Candidate
    acceptance additionally requires a rectangular shape and white-marking
    support on at least two sides.
    """
    slot_mask, white_mask = build_masks(image_bgr, config)
    height, width = slot_mask.shape
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        slot_mask, connectivity=8
    )

    candidates: list[dict[str, Any]] = []
    for label in range(1, component_count):
        x, y, component_width, component_height, area = map(int, stats[label])
        if area < config.min_component_area_px:
            continue

        component = (labels[y : y + component_height, x : x + component_width] == label).astype(
            np.uint8
        ) * 255
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        contour = contour.copy()
        contour[:, 0, 0] += x
        contour[:, 0, 1] += y

        rect = cv2.minAreaRect(contour)
        rect_width, rect_height = rect[1]
        rect_area = float(rect_width * rect_height)
        if rect_area <= 0.0:
            continue

        corners = _order_corners(cv2.boxPoints(rect).astype(np.float32))
        shorter = min(rect_width, rect_height)
        longer = max(rect_width, rect_height)
        aspect_ratio = float(longer / max(shorter, 1e-6))
        rectangularity = float(area / rect_area)
        line_support = _edge_line_support(
            corners, white_mask, config.line_probe_offsets_px
        )
        supported_edges = sum(
            value >= config.min_line_support for value in line_support
        )

        touches_border = (
            x <= 0
            or y <= 0
            or x + component_width >= width
            or y + component_height >= height
        )
        rejection_reasons: list[str] = []
        if aspect_ratio < config.min_aspect_ratio or aspect_ratio > config.max_aspect_ratio:
            rejection_reasons.append("aspect_ratio_out_of_range")
        min_short_side = config.expected_short_side_px * (
            1.0 - config.short_side_tolerance_fraction
        )
        max_short_side = config.expected_short_side_px * (
            1.0 + config.short_side_tolerance_fraction
        )
        if shorter < min_short_side or shorter > max_short_side:
            rejection_reasons.append("slot_width_out_of_range")
        if longer < config.min_long_side_px or longer > config.max_long_side_px:
            rejection_reasons.append("slot_length_out_of_range")
        if rectangularity < config.min_rectangularity:
            rejection_reasons.append("component_not_rectangular_enough")
        if supported_edges < config.min_supported_edges:
            rejection_reasons.append("insufficient_white_line_support")

        candidates.append(
            {
                "corners_px": [
                    [round(float(point[0]), 1), round(float(point[1]), 1)]
                    for point in corners
                ],
                "bbox_xyxy_px": [
                    round(float(np.min(corners[:, 0])), 1),
                    round(float(np.min(corners[:, 1])), 1),
                    round(float(np.max(corners[:, 0])), 1),
                    round(float(np.max(corners[:, 1])), 1),
                ],
                "visible_blue_area_px": area,
                "rectangularity": round(rectangularity, 3),
                "aspect_ratio": round(aspect_ratio, 3),
                "white_line_support_per_edge": [
                    round(value, 3) for value in line_support
                ],
                "supported_edge_count": supported_edges,
                "touches_image_border": touches_border,
                "partial_or_occluded": touches_border or supported_edges < 3,
                "accepted": not rejection_reasons,
                "rejection_reasons": rejection_reasons,
                "_sort_key": [
                    round(float(np.mean(corners[:, 1])), 1),
                    round(float(np.mean(corners[:, 0])), 1),
                ],
            }
        )

    candidates.sort(key=lambda item: item["_sort_key"])
    accepted: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for slot_id, candidate in enumerate(candidates, start=1):
        candidate["candidate_id"] = slot_id
        candidate.pop("_sort_key")
        (accepted if candidate["accepted"] else rejected).append(candidate)
    return accepted, rejected, slot_mask, white_mask


def draw_debug_image(
    image_bgr: np.ndarray,
    accepted: list[dict[str, Any]],
    rejected: list[dict[str, Any]],
) -> np.ndarray:
    """Draw accepted slot polygons and rejected color regions for inspection."""
    output = image_bgr.copy()
    entries = [(item, True) for item in accepted]
    entries.extend((item, False) for item in rejected)
    for candidate, accepted_candidate in entries:
        corners = np.rint(np.asarray(candidate["corners_px"])).astype(np.int32)
        color = (0, 220, 0) if accepted_candidate else (0, 140, 255)
        cv2.polylines(output, [corners], isClosed=True, color=color, thickness=3)
        center = np.rint(np.mean(corners, axis=0)).astype(int)
        label = (
            f"slot-{candidate.get('slot_id', candidate['candidate_id'])}"
            if accepted_candidate
            else f"reject-{candidate['candidate_id']}"
        )
        cv2.putText(
            output,
            label,
            (int(center[0]), int(center[1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        for point in corners:
            cv2.circle(output, tuple(point), 4, (0, 0, 255), -1)
    return output

"""Helpers shared by the rosbag recording launch entry and its tests."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Iterable


DEFAULT_RECORD_TOPICS = (
    "/clock",
    "/scan",
    "/imu",
    "/camera_front/image",
    "/camera_front/camera_info",
    "/model/my_orange_car/odometry",
    "/world/luxury_parking_lot_expanded/pose/info",
)


class RecordingDestinationError(ValueError):
    """Raised when a rosbag destination would overwrite an existing run."""


def manifest_path_for_bag(bag_output_dir: str | Path) -> Path:
    """Return the sidecar manifest path without creating the bag directory."""

    bag_path = Path(bag_output_dir).expanduser()
    return bag_path.parent / f"{bag_path.name}.manifest.json"


def ensure_fresh_recording_destination(bag_output_dir: str | Path) -> Path:
    """Validate that rosbag2 and its sidecar can start a new recording."""

    bag_path = Path(bag_output_dir).expanduser()
    manifest = manifest_path_for_bag(bag_path)
    if bag_path.exists():
        raise RecordingDestinationError(
            f"rosbag output directory already exists: {bag_path}; "
            "choose a new bag_output_dir"
        )
    if manifest.exists():
        raise RecordingDestinationError(
            f"recording manifest already exists: {manifest}; "
            "choose a new bag_output_dir"
        )
    return bag_path


def write_recording_manifest(
    bag_output_dir: str | Path,
    *,
    world_file: str,
    models_path: str,
    topics: Iterable[str] = DEFAULT_RECORD_TOPICS,
    vehicle_model_name: str = "my_orange_car",
    collection_save_interval: str = "0.5",
    recorded_at_utc: str | None = None,
) -> Path:
    """Write recording metadata next to a rosbag output directory.

    The bag directory itself is intentionally not created here.  rosbag2 expects
    to own that path when recording starts, while this sidecar can be created by
    the launch description before the recorder process is started.
    """

    destination = manifest_path_for_bag(bag_output_dir)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "recorded_at_utc": recorded_at_utc
        or datetime.now(timezone.utc).isoformat(),
        "bag_path": str(Path(bag_output_dir).expanduser()),
        "world_file": str(Path(world_file).expanduser()),
        "models_path": str(Path(models_path).expanduser()),
        "vehicle_model_name": vehicle_model_name,
        "collection_save_interval": str(collection_save_interval),
        "topics": list(topics),
    }
    destination.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return destination

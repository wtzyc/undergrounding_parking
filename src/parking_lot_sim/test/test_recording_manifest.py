import json

from parking_lot_sim.recording import (
    DEFAULT_RECORD_TOPICS,
    RecordingDestinationError,
    ensure_fresh_recording_destination,
    manifest_path_for_bag,
    write_recording_manifest,
)


def test_manifest_is_written_next_to_bag_without_creating_bag_directory(tmp_path):
    bag_dir = tmp_path / "bags" / "b_zone_loop"
    manifest = write_recording_manifest(
        bag_dir,
        world_file="/workspace/world/track_world.sdf",
        models_path="/workspace/models",
        recorded_at_utc="2026-09-20T00:00:00+00:00",
    )

    assert manifest == manifest_path_for_bag(bag_dir)
    assert manifest == tmp_path / "bags" / "b_zone_loop.manifest.json"
    assert not bag_dir.exists()

    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["bag_path"].endswith("bags/b_zone_loop")
    assert payload["world_file"].endswith("world/track_world.sdf")
    assert payload["models_path"].endswith("models")
    assert payload["vehicle_model_name"] == "my_orange_car"
    assert payload["topics"] == list(DEFAULT_RECORD_TOPICS)


def test_existing_bag_directory_is_rejected_with_a_clear_message(tmp_path):
    bag_dir = tmp_path / "bags" / "already_recorded"
    bag_dir.mkdir(parents=True)

    try:
        ensure_fresh_recording_destination(bag_dir)
    except RecordingDestinationError as exc:
        assert "already exists" in str(exc)
        assert "new bag_output_dir" in str(exc)
    else:
        raise AssertionError("an existing bag directory must be rejected")

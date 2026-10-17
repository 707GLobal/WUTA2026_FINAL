"""Reuse the existing viewer renderer to resolve effective native/bridge topics."""

import importlib.util
import math
from pathlib import Path
import yaml

DISPLAY_NAMES = {
    "guided": "Camera-guided clusters (fusion)",
    "fused": "Fused targets and sources",
    "map": "Cone map",
    "centerline": "Centerline",
    "path": "Final planned path",
    "trajectory": "Driven trajectory",
    "control": "Control target",
}


def load_config(filename, mode=None):
    path = Path(filename).resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    for key in ("topics_config", "rviz_config"):
        config[key] = str((path.parent / config[key]).resolve())
    config["display_mode"] = mode or config["display_mode"]
    topics = yaml.safe_load(Path(config["topics_config"]).read_text(encoding="utf-8"))
    template = yaml.safe_load(Path(config["rviz_config"]).read_text(encoding="utf-8"))
    renderer = (
        Path(__file__).resolve().parents[2]
        / "src/fsd_visualization/fsd_visualization/configuration.py"
    )
    # The original pure configuration module has no ROS dependency.
    if not renderer.is_file():
        raise ValueError(
            f"keep the full viewer source directory; renderer missing: {renderer}"
        )
    spec = importlib.util.spec_from_file_location("_fsd_viewer_configuration", renderer)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    effective = module.render(template, topics, config["display_mode"])
    displays = effective["Visualization Manager"]["Displays"]
    config["resolved_layers"] = {}
    for key, name in DISPLAY_NAMES.items():
        # Existing bridge renderer gives the fused display a descriptive suffix.
        display = next(
            d
            for d in displays
            if d["Name"] == name
            or (key == "fused" and d["Name"].startswith("Fused targets ("))
        )
        config["resolved_layers"][key] = (
            display["Topic"]["Value"] if display.get("Enabled", False) else ""
        )
    config["fixed_frame"] = topics["fixed_frame"]
    config["pose_topic"] = topics["topics"]["pose"]
    config["target_frame"] = topics["target_frame"]
    config["velocity_topic"] = (
        config.get("velocity_topic") or topics["topics"]["velocity"]
    )
    config["inspection"] = {
        "excluded": {
            "camera": topics["topics"]["camera_image"],
            "raw_lidar": topics["topics"]["raw_lidar"],
        },
        "raw_lidar_template": next(
            d
            for d in template["Visualization Manager"]["Displays"]
            if d["Name"] == "Raw LiDAR (optional)"
        )["Topic"]["Value"],
        "velocity_in_topics_yaml": topics["topics"]["velocity"],
        "effective_marker_topics": config["resolved_layers"],
    }
    validate(config)
    return config


def validate(config):
    def positive(value, name):
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{name} must be finite and positive")

    link = config["link"]
    for name in (
        "bytes_per_second",
        "fragment_bytes",
        "response_timeout",
        "turnaround_seconds",
        "assembly_timeout",
    ):
        positive(link[name], name)
    if link["bytes_per_second"] > 240:
        raise ValueError(
            "2.4kbps profile allows <=240 UART bytes/s aggregate; default is 160"
        )
    if (
        not isinstance(link["fragment_bytes"], int)
        or not 16 <= link["fragment_bytes"] <= 96
    ):
        raise ValueError("fragment_bytes must be an integer from 16 to 96")
    if not isinstance(link["retries"], int) or not 0 <= link["retries"] <= 5:
        raise ValueError("retries must be 0..5")
    for name in ("max_message_bytes", "extension_queue"):
        if not isinstance(link[name], int) or link[name] <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if link["max_message_bytes"] > 4096:
        raise ValueError("max_message_bytes cannot exceed 4096")
    if link["response_timeout"] < 2.0:
        raise ValueError("response_timeout must be >=2 seconds for the 2.4kbps profile")
    positive(config["geometry"]["quantization_m"], "quantization_m")
    positive(config["foxglove"]["stale_seconds"], "stale_seconds")
    for name in ("cache_markers", "text_chars", "snapshot_bytes"):
        if (
            not isinstance(config["geometry"][name], int)
            or config["geometry"][name] < 1
        ):
            raise ValueError(f"geometry.{name} must be a positive integer")
    if config["geometry"]["snapshot_bytes"] > link["max_message_bytes"]:
        raise ValueError("snapshot_bytes exceeds max_message_bytes")
    if config["geometry"]["snapshot_bytes"] > link["fragment_bytes"] * 64:
        raise ValueError("snapshot_bytes exceeds 64 fragment capacity")
    for key, layer in config["layers"].items():
        positive(layer["interval"], f"{key}.interval")
        positive(layer["weight"], f"{key}.weight")
        if key != "vehicle":
            for name in ("markers", "points"):
                if not isinstance(layer[name], int) or not 2 <= layer[name] <= 256:
                    raise ValueError(f"{key}.{name} must be 2..256")

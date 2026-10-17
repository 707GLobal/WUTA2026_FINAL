from pathlib import Path
import pytest
import yaml
from fsd_radio.configuration import load_config, validate


def test_topics_are_resolved_by_existing_renderer(config):
    assert config["serial"]["pc_port"] == "COM3"
    assert config["serial"]["linux_port"] == ""
    assert config["resolved_layers"]["fused"] == "/perception/fused/cones_viz"
    assert config["velocity_topic"] == "/chcnav/velocity"
    assert "/rslidar_points" not in config["resolved_layers"].values()
    bridge = load_config(Path(__file__).resolve().parents[1] / "radio.yaml", "bridge")
    assert bridge["resolved_layers"]["fused"] == "/fsd_visualization/fused_cones"
    assert bridge["resolved_layers"]["path"] == "/fsd_visualization/planned_path"


@pytest.mark.parametrize(
    "name,value",
    [
        ("bytes_per_second", 300),
        ("fragment_bytes", 240),
        ("response_timeout", float("nan")),
        ("retries", -1),
    ],
)
def test_bad_link_settings_rejected(config, name, value):
    config["link"][name] = value
    with pytest.raises(ValueError):
        validate(config)


def test_personal_topics_yaml_outside_source_tree(config, tmp_path):
    topics = yaml.safe_load(Path(config["topics_config"]).read_text(encoding="utf-8"))
    topics["topics"]["path_markers"] = "/personal/path_viz"
    path = tmp_path / "personal_topics.yaml"
    path.write_text(yaml.safe_dump(topics), encoding="utf-8")
    config["topics_config"] = str(path)
    filename = tmp_path / "custom_radio.yaml"
    filename.write_text(yaml.safe_dump(config), encoding="utf-8")
    custom = load_config(filename)
    assert custom["resolved_layers"]["path"] == "/personal/path_viz"

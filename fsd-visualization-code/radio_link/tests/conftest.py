from pathlib import Path
import pytest
from fsd_radio.configuration import load_config


@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parents[1] / "radio.yaml")

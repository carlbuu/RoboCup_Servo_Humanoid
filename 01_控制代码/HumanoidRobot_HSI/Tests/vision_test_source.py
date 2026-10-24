"""Resolve the packaged vision source or an explicitly provided reference."""
import os
from pathlib import Path


def current_vision_source():
    path = Path(__file__).resolve().parents[3] / "02_视觉代码" / "geometry_vision19.py"
    if not path.is_file():
        raise FileNotFoundError(f"Current vision source is missing: {path}")
    return path


def reference_vision_source():
    value = os.environ.get("VISION_REFERENCE_SOURCE")
    if not value:
        raise SystemExit(
            "This suite requires its matching reference vision source, which is not "
            "included. Set VISION_REFERENCE_SOURCE explicitly. For the packaged "
            "version 19, run test_walk_late_camera.py."
        )
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Reference vision source is missing: {path}")
    return path

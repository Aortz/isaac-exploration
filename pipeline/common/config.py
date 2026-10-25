"""Config loading shared by every Python stage (uv env, Blender and Isaac interpreters).

Blender's and Isaac's bundled Pythons have no PyYAML, so the uv stages write a JSON copy of each
config to build/<name>_config.json (config/site.yaml -> build/site_config.json) and those
interpreters read that instead.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
BUILD = PROJECT_ROOT / "build"
CONFIG_JSON = BUILD / "site_config.json"


def _yaml_path(path: str | os.PathLike | None) -> Path:
    return Path(path or os.environ.get("SITE_CONFIG", PROJECT_ROOT / "config" / "site.yaml"))


def json_path_for(yaml_path: Path) -> Path:
    return BUILD / f"{yaml_path.stem}_config.json"


def load_config(path: str | os.PathLike | None = None) -> dict:
    yaml_path = _yaml_path(path)
    try:
        import yaml
    except ImportError:
        cached = json_path_for(yaml_path)
        if not cached.exists() or cached.stat().st_mtime < yaml_path.stat().st_mtime:
            raise RuntimeError(f"{cached} missing or older than {yaml_path}; "
                               f"run `uv run python -m pipeline.common.config {yaml_path}`")
        return json.loads(cached.read_text())
    return yaml.safe_load(yaml_path.read_text())


def project_path(rel: str) -> Path:
    return (PROJECT_ROOT / os.path.expanduser(rel)).resolve()


def export_json(path: str | os.PathLike | None = None) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    out = json_path_for(_yaml_path(path))
    out.write_text(json.dumps(load_config(path), indent=2) + "\n")
    return out


if __name__ == "__main__":
    print(export_json(sys.argv[1] if len(sys.argv) > 1 else None))

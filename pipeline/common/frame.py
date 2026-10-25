"""Site coordinate frame shared by the localization and validation stages.

Frames:
  geographic  WGS84 lon/lat/height (EPSG:4326 + height as recorded in EXIF)
  utm         projected UTM easting/northing/height in metres (EPSG:326xx / 327xx)
  odm         UTM minus the ODM offset from coords.txt (what odm_textured_model_geo.obj uses)
  local       UTM minus the chosen site origin; X east, Y north, Z up, metres (what Blender/Isaac use)

Only a translation separates utm, odm and local. Heights are passed through unchanged from the
reconstruction; their datum is whatever the camera EXIF used and is recorded, not converted.
"""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from pyproj import Transformer


def parse_odm_coords(path: str | Path) -> tuple[int, np.ndarray]:
    """Read ODM coords.txt and return (EPSG code, offset[easting, northing]).

    Line 1 is the projection, for example "WGS84 UTM 17N". Line 2 is the offset subtracted from
    every georeferenced output.
    """
    lines = [ln.strip() for ln in Path(path).read_text().splitlines() if ln.strip()]
    m = re.match(r"WGS84 UTM (\d+)([NS])", lines[0], re.IGNORECASE)
    if not m:
        raise ValueError(f"unsupported projection line in {path}: {lines[0]!r}")
    zone, hemi = int(m.group(1)), m.group(2).upper()
    epsg = (32600 if hemi == "N" else 32700) + zone
    offset = np.array([float(v) for v in lines[1].split()[:2]])
    return epsg, offset


@dataclass
class SiteFrame:
    epsg: int
    odm_offset: list[float]           # [easting, northing] subtracted by ODM
    origin_utm: list[float]           # [easting, northing, height] of local (0, 0, 0)
    vertical_datum: str
    axes: str = "ENU: X east, Y north, Z up, metres"
    notes: list[str] = field(default_factory=list)

    @property
    def _origin(self) -> np.ndarray:
        return np.asarray(self.origin_utm, dtype=np.float64)

    @property
    def odm_to_local_shift(self) -> np.ndarray:
        """Vector subtracted from ODM coordinates to get local coordinates."""
        off = np.array([self.odm_offset[0], self.odm_offset[1], 0.0])
        return self._origin - off

    def utm_to_local(self, p: np.ndarray) -> np.ndarray:
        return np.asarray(p, dtype=np.float64) - self._origin

    def local_to_utm(self, p: np.ndarray) -> np.ndarray:
        return np.asarray(p, dtype=np.float64) + self._origin

    def odm_to_local(self, p: np.ndarray) -> np.ndarray:
        return np.asarray(p, dtype=np.float64) - self.odm_to_local_shift

    def _to_utm(self) -> Transformer:
        return Transformer.from_crs("EPSG:4326", f"EPSG:{self.epsg}", always_xy=True)

    def lonlat_to_local(self, lon, lat, h) -> np.ndarray:
        e, n = self._to_utm().transform(np.asarray(lon), np.asarray(lat))
        return self.utm_to_local(np.stack([e, n, np.asarray(h, dtype=np.float64)], axis=-1))

    def local_to_lonlat(self, p: np.ndarray) -> np.ndarray:
        utm = self.local_to_utm(p)
        lon, lat = self._to_utm().transform(utm[..., 0], utm[..., 1], direction="INVERSE")
        return np.stack([lon, lat, utm[..., 2]], axis=-1)

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "SiteFrame":
        return cls(**json.loads(Path(path).read_text()))

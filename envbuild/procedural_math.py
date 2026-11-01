"""Pure-numpy terrain helpers shared by envbuild (Isaac Python) and testplan (uv Python)."""

from __future__ import annotations

import numpy as np


def height_at(heights, x0, y0, res, x, y) -> np.ndarray:
    """Bilinear terrain height at world (x, y); rows of `heights` run along +y from y0, columns along +x from x0."""
    ny, nx = heights.shape
    fx = np.clip((np.asarray(x) - x0) / res, 0, nx - 1.001)
    fy = np.clip((np.asarray(y) - y0) / res, 0, ny - 1.001)
    ix, iy = fx.astype(int), fy.astype(int)
    tx, ty = fx - ix, fy - iy
    h00, h10 = heights[iy, ix], heights[iy, ix + 1]
    h01, h11 = heights[iy + 1, ix], heights[iy + 1, ix + 1]
    return (h00 * (1 - tx) + h10 * tx) * (1 - ty) + (h01 * (1 - tx) + h11 * tx) * ty

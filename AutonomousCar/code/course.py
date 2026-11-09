"""Closed test course for controller work: smooth centerline, width, fast (s, e) lookup.

The repo's Track.py holds the full-size Laguna Seca map; this is an RC-scale loop that fits the
same parking lot as the sysid passes, with the queries a sampling controller needs:
`lookup` turns thousands of rollout points into (arc length, signed lateral offset) via a
precomputed grid, which is how AutoRally-style MPPI evaluates track costs cheaply.
"""

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

G = 9.81


class Course:
    def __init__(self, control_points, width=3.0, ds=0.05, grid_res=0.1, margin=6.0):
        pts = np.asarray(control_points, dtype=float)
        if np.hypot(*(pts[0] - pts[-1])) > 1e-9:
            pts = np.vstack([pts, pts[:1]])  # close the loop
        chord = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        spline = CubicSpline(chord, pts, bc_type="periodic")
        u = np.linspace(0, chord[-1], int(chord[-1] / ds * 4), endpoint=False)
        xy = spline(u)
        seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
        s_fine = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(s_fine[-1])
        # resample at uniform arc length
        n = int(round(self.length / ds))
        self.s = np.arange(n) * (self.length / n)
        u_s = np.interp(self.s, s_fine, np.concatenate([u, [chord[-1]]]))
        self.xy = spline(u_s)
        d1, d2 = spline(u_s, 1), spline(u_s, 2)
        self.psi = np.arctan2(d1[:, 1], d1[:, 0])
        self.kappa = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.hypot(d1[:, 0], d1[:, 1]) ** 3
        self.tangent = np.column_stack([np.cos(self.psi), np.sin(self.psi)])
        self.normal = np.column_stack([-self.tangent[:, 1], self.tangent[:, 0]])  # left
        self.width = float(width)
        self.ds = self.length / n
        self._tree = cKDTree(self.xy)

        # (s, e) lookup grid
        lo = self.xy.min(axis=0) - margin
        hi = self.xy.max(axis=0) + margin
        self.grid_origin, self.grid_res = lo, grid_res
        gx = np.arange(lo[0], hi[0], grid_res) + grid_res / 2
        gy = np.arange(lo[1], hi[1], grid_res) + grid_res / 2
        GX, GY = np.meshgrid(gx, gy, indexing="ij")
        s_g, e_g = self.project(GX.ravel(), GY.ravel())
        self._s_grid = s_g.reshape(GX.shape).astype(np.float32)
        self._e_grid = e_g.reshape(GX.shape).astype(np.float32)

    # -- queries ---------------------------------------------------------------------------
    def project(self, x, y):
        """Exact-ish projection: (arc length s, signed lateral offset e, +left) of each point."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        p = np.stack([x, y], axis=-1)
        _, i = self._tree.query(p)
        d = p - self.xy[i]
        along = np.einsum("...j,...j->...", d, self.tangent[i])
        e = np.einsum("...j,...j->...", d, self.normal[i])
        return (self.s[i] + along) % self.length, e

    def lookup(self, x, y):
        """Grid lookup (vectorized, ~grid_res accuracy). Points off the grid read as far off track."""
        ix = np.floor((np.asarray(x) - self.grid_origin[0]) / self.grid_res).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.grid_origin[1]) / self.grid_res).astype(np.int64)
        nx, ny = self._s_grid.shape
        off = (ix < 0) | (ix >= nx) | (iy < 0) | (iy >= ny)
        ix, iy = np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1)
        s = self._s_grid[ix, iy].astype(float)
        e = np.where(off, 1e3, self._e_grid[ix, iy])
        return s, e

    def at(self, s):
        """Centerline point, heading, curvature, normal at arc length s (nearest sample)."""
        i = np.round(np.asarray(s) / self.ds).astype(np.int64) % len(self.s)
        return self.xy[i], self.psi[i], self.kappa[i], self.normal[i]

    def wrap_ds(self, ds):
        """Arc-length difference wrapped to (-L/2, L/2] - progress across the start line."""
        return (ds + self.length / 2) % self.length - self.length / 2

    def min_radius(self):
        return float(1.0 / np.max(np.abs(self.kappa)))

    def extent(self):
        """Bounding box of the track edges (m)."""
        half = self.width / 2
        edges = np.vstack([self.xy + half * self.normal, self.xy - half * self.normal])
        return np.ptp(edges[:, 0]), np.ptp(edges[:, 1])

    def speed_profile(self, a_lat=0.6 * G, a_accel=3.0, a_brake=6.0, v_max=20.0):
        """Curvature-limited speed with accel/brake passes around the closed loop (m/s per sample)."""
        v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(self.kappa), 1e-6)))
        n = len(v)
        for _ in range(3):  # passes wrap the loop; converges in two
            for i in range(2 * n):  # forward: acceleration limit
                k, j = i % n, (i - 1) % n
                v[k] = min(v[k], np.sqrt(v[j] ** 2 + 2 * a_accel * self.ds))
            for i in range(2 * n, 0, -1):  # backward: braking limit
                k, j = i % n, (i + 1) % n
                v[k] = min(v[k], np.sqrt(v[j] ** 2 + 2 * a_brake * self.ds))
        return v


def parking_lot_course(width=3.0, **kw):
    """Kidney-shaped loop (~92 m, edges 38 x 22 m) in the same 40 x 25 m lot as the sysid passes:
    a long straight, two ~4-5 m radius corners and a dent that forces a left-right transition."""
    pts = [(7, 3.5), (20, 3), (33, 3.5), (37, 9), (36, 16), (30, 21), (23, 19), (17, 19), (10, 21), (4, 16), (3, 9)]
    return Course(pts, width=width, **kw)

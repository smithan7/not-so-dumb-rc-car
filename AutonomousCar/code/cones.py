"""Cone-marked track: layout, RTK survey, camera detections, and the course built from the survey.

Three versions of every cone exist, and the simulation keeps them apart:
  true      where the cone actually stands. Placement is approximate: the design spacing and
            offsets are only aimed for, and some cones are missing.
  surveyed  the cone map: true position + RTK survey error (per-cone sd). This is all the car knows.
  detected  per camera frame: range/bearing from the L515 RGB stream (colour gives the side),
            with range error growing with distance, an extrinsic yaw error, misses, wrong colour
            calls and the odd false detection.

The planner's track is rebuilt from the survey (`course_from_cones`): the centerline runs midway
between the surveyed left and right cone lines, and the edges are the lines between cones. The
truth edges come from the true cones, so placement error never matters but survey error does.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from course import Course

LEFT, RIGHT = 1, -1
CONE_RADIUS = 0.12  # base radius: the usable edge is this far inside the line through cone centres

# L515 RGB camera + colour detection. Range from depth when it returns, else the ground-plane
# projection of the cone base; sd = range_sd0 + range_sd_per_m * range (meets gate B at 3 / 6 m).
CAMERA = {"rate_hz": 15.0, "lag_s": 0.07, "fov_deg": 70.0, "range_min": 0.5, "range_max": 8.0,
          "p_detect_near": 0.9, "p_detect_far": 0.6, "range_sd0": 0.05, "range_sd_per_m": 0.03,
          "bearing_sd": 0.01, "mount_yaw_err_deg": 0.5, "p_wrong_side": 0.005, "p_false": 0.05,
          "cam_offset": 0.25}
# how the cones go down: aimed at the design, placed by hand, surveyed with the RTK rover pole
PLACEMENT = {"spacing": 3.5, "corner_spacing": 2.0, "corner_radius": 8.0, "placement_sd": 0.25,
             "along_sd": 0.4, "p_missing": 0.1, "survey_sd": 0.05}


@dataclass
class ConeMap:
    xy: np.ndarray    # (n, 2)
    side: np.ndarray  # (n,) LEFT / RIGHT
    sd: np.ndarray    # (n,) position sd per axis (0 for the truth)

    def __len__(self):
        return len(self.side)

    def max_gap(self, course):
        """Largest along-track gap between consecutive cones on either side (m)."""
        gaps = []
        for side in (LEFT, RIGHT):
            s, _ = course.project(*self.xy[self.side == side].T)
            s = np.sort(s)
            gaps.append(np.max(np.diff(np.concatenate([s, [s[0] + course.length]]))))
        return float(max(gaps))


def layout_cones(design, rng, placement=PLACEMENT):
    """Cones as actually placed along the design course's edges: tighter in corners, each off its
    mark by placement_sd (across) and along_sd (along), a fraction p_missing never placed."""
    pl = placement
    xy, side = [], []
    for sgn in (LEFT, RIGHT):
        s = 0.0
        while s < design.length - 0.5 * pl["corner_spacing"]:
            p, _, kappa, n = design.at(s)
            i = int(round(s / design.ds)) % len(design.s)
            t = design.tangent[i]
            if rng.random() >= pl["p_missing"]:
                off = sgn * design.width / 2 + rng.normal(0, pl["placement_sd"])
                xy.append(p + off * n + rng.normal(0, pl["along_sd"]) * t)
                side.append(sgn)
            tight = abs(kappa) > 1.0 / pl["corner_radius"]
            s += pl["corner_spacing"] if tight else pl["spacing"]
    return ConeMap(np.array(xy), np.array(side), np.zeros(len(side)))


def survey(true_map, rng, sd=PLACEMENT["survey_sd"]):
    """RTK rover pole on each cone: the map the car uses."""
    return ConeMap(true_map.xy + rng.normal(0, sd, true_map.xy.shape), true_map.side.copy(),
                   np.full(len(true_map), float(sd)))


def _edge_on(course, cones, side, smooth_m=0.0):
    """Signed offset e(s) of one cone line on `course`, linear between cones, around the loop."""
    s, e = course.project(*cones.xy[cones.side == side].T)
    o = np.argsort(s)
    s, e = s[o], e[o]
    L = course.length
    s_ext = np.concatenate([s[-1:] - L, s, s[:1] + L])
    e_ext = np.concatenate([e[-1:], e, e[:1]])
    out = np.interp(course.s, s_ext, e_ext)
    if smooth_m > 0:
        k = max(int(smooth_m / course.ds), 1)
        out = np.convolve(np.concatenate([out[-k:], out, out[:k]]), np.ones(k) / k, mode="same")[k:-k]
    return out


def course_from_cones(design, cones, edge_sd=None, cp_spacing=2.0, smooth_m=4.0):
    """Planner course from a cone map: centerline midway between the cone lines (smoothed so
    placement scatter does not become curvature), edges = lines between cones."""
    eL, eR = _edge_on(design, cones, LEFT, smooth_m), _edge_on(design, cones, RIGHT, smooth_m)
    mid = design.xy + (0.5 * (eL + eR))[:, None] * design.normal
    step = max(int(cp_spacing / design.ds), 1)
    width = float(np.mean(eL - eR)) - 2 * CONE_RADIUS
    course = Course(mid[::step], width=width)
    set_edges(course, cones, edge_sd=float(np.mean(cones.sd)) if edge_sd is None else edge_sd)
    return course


def set_edges(course, cones, edge_sd=0.0):
    course.e_left = _edge_on(course, cones, LEFT) - CONE_RADIUS
    course.e_right = _edge_on(course, cones, RIGHT) + CONE_RADIUS
    course.edge_sd = edge_sd
    return course


def truth_edges(course, true_map):
    """True edges expressed on the planner's course: (e_left, e_right) per sample."""
    return _edge_on(course, true_map, LEFT) - CONE_RADIUS, _edge_on(course, true_map, RIGHT) + CONE_RADIUS


class ConeCamera:
    """Detections of the true cones from the true pose, as the L515 colour pipeline would give."""

    def __init__(self, true_map, rng, cfg=CAMERA):
        self.cones, self.rng, self.cfg = true_map, rng, cfg
        self.mount_yaw = math.radians(cfg["mount_yaw_err_deg"]) * rng.choice([-1, 1])  # unmodelled
        self._tree = cKDTree(true_map.xy)

    def frame(self, x, y, psi):
        c = self.cfg
        cam = np.array([x + c["cam_offset"] * math.cos(psi), y + c["cam_offset"] * math.sin(psi)])
        out = []
        for i in self._tree.query_ball_point(cam, c["range_max"]):
            d = self.cones.xy[i] - cam
            r = math.hypot(*d)
            b = math.atan2(d[1], d[0]) - psi - self.mount_yaw
            b = (b + math.pi) % (2 * math.pi) - math.pi
            if r < c["range_min"] or abs(b) > math.radians(c["fov_deg"]) / 2:
                continue
            p = c["p_detect_near"] + (c["p_detect_far"] - c["p_detect_near"]) * r / c["range_max"]
            if self.rng.random() > p:
                continue
            side = int(self.cones.side[i])
            if self.rng.random() < c["p_wrong_side"]:
                side = -side
            out.append((r + self.rng.normal(0, c["range_sd0"] + c["range_sd_per_m"] * r),
                        b + self.rng.normal(0, c["bearing_sd"]), side))
        if self.rng.random() < c["p_false"]:
            out.append((self.rng.uniform(1, c["range_max"]),
                        self.rng.uniform(-1, 1) * math.radians(c["fov_deg"]) / 2,
                        int(self.rng.choice([LEFT, RIGHT]))))
        return out

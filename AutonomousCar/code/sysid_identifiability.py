"""Monte Carlo identifiability study: can the planned tests and sensors pin down the model?

    python sysid_identifiability.py            # ~1-2 min
    python sysid_identifiability.py --runs 3   # quick look

Each run draws a random 'true car' (every parameter scaled by U(0.7, 1.3) around the defaults),
simulates what the logger would record for a test plan, fits starting from the defaults, and
scores the parameter errors. Plans: the designed passes vs. the same driving time of
manual-style driving; speed from the driveshaft encoder vs. GPS only.
"""

import argparse
import sys
import warnings

import numpy as np

import sysid_maneuvers as D
import sysid_model as M

PERTURB = [k for k in M.LATERAL_FREE + M.LONGITUDINAL_FREE if k not in ("steer_center_us", "gyro_bias")]


def random_truth(rng):
    p = M.default_params()
    for k in PERTURB:
        p[k] *= rng.uniform(0.7, 1.3)
    p["steer_center_us"] += rng.uniform(-15, 15)
    p["gyro_bias"] = rng.uniform(-0.02, 0.02)
    return p


def run_plan(plan, truth, speed_sensor, rng):
    seed = int(rng.integers(1 << 31))
    if plan == "designed":
        long_logs = [D.synthesize(s, truth, speed_sensor, seed=seed + i) for i, s in enumerate(D.longitudinal_passes())]
        lat_logs = [D.synthesize(s, truth, speed_sensor, seed=seed + 100 + i) for i, s in enumerate(D.lateral_passes())]
    else:
        n_s = sum(D.footprint(s)["duration_s"] for s in D.longitudinal_passes() + D.lateral_passes())
        log = D.synthesize(D.manual_like_design(n_s, seed=seed), truth, speed_sensor, seed=seed)
        long_logs = lat_logs = [log]
    fl = M.fit_longitudinal([D.longitudinal_segment(l) for l in long_logs])
    fa = M.fit_lateral([D.lateral_segment(l) for l in lat_logs])
    return fl, fa


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    warnings.simplefilter("ignore")

    rng = np.random.default_rng(args.seed)
    truths = [random_truth(rng) for _ in range(args.runs)]
    setups = [("designed", "encoder"), ("designed", "gps"), ("manual", "encoder"), ("manual", "gps")]
    free = M.LONGITUDINAL_FREE + M.LATERAL_FREE
    results = {}
    for plan, sensor in setups:
        errs = {k: [] for k in free}
        for truth in truths:
            fl, fa = run_plan(plan, truth, sensor, rng)
            for f in (fl, fa):
                for k in f.free:
                    t = truth[k]
                    # relative error, except offsets where absolute error is what matters
                    errs[k].append(abs(f.params[k] - t) * ABS_SCALE[k] if k in ABS_SCALE else abs(f.params[k] / t - 1))
        results[(plan, sensor)] = errs
        print(f"done: {plan} / {sensor}", file=sys.stderr)

    print(f"\nMedian |error| over {args.runs} random cars (fit starts from defaults; "
          f"offsets in their own units, others relative)\n")
    head = f"{'parameter':>18}" + "".join(f"{p + '/' + s:>18}" for p, s in setups)
    print(head)
    print("-" * len(head))
    for k in free:
        row = f"{k:>18}"
        for key in setups:
            e = np.median(results[key][k])
            unit = PARAM_UNITS.get(k)
            row += f"{e:>11.2g}{unit:<7}" if unit else f"{e:17.1%} "
        print(row)
    return 0


# reported as absolute error in these units instead of relative
ABS_SCALE = {"steer_center_us": 1.0, "gyro_bias": 1.0, "input_delay": 1000.0}
PARAM_UNITS = {"steer_center_us": " us", "gyro_bias": " rad/s", "input_delay": " ms"}

if __name__ == "__main__":
    sys.exit(main())

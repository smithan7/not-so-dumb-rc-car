"""Rehearse the sysid loop on the emulator: fit -> suggest -> drive the suggestions -> refit.

    python sysid_rehearsal.py                      # a random 'true car', up to 5 rounds
    python sysid_rehearsal.py --rounds 3 --seed 4 --max-speed 2.5

Sim only: between maneuvers the car is teleported back to the start (on the real car the operator
repositions it). Each round fits everything logged so far, prints gate A and the plan, then drives
the plan. At the end, the fitted parameters are compared with the emulator's true car.
"""

import argparse
import tempfile
import time

import numpy as np

import ground_station as G
import sysid_design as SD
import sysid_model as M
import sysid_run as R

PERTURB = ["speed_gain", "tau_speed", "deadband_us", "brake_gain", "roll_decel", "drag_coeff",
           "steer_gain", "tau_steer", "v_char", "input_delay"]


def random_car(rng):
    p = M.default_params()
    car = {k: p[k] * rng.uniform(0.6, 1.4) for k in PERTURB}
    car["speed_gain"] = p["speed_gain"] * rng.uniform(0.25, 0.5)  # the real car is unknown: don't let the prior be right
    car["steer_center_us"] = 1500 + rng.uniform(-12, 12)
    return car


def drive(gs, maneuvers):
    emu, cmd = gs.emulator, gs.commander
    for m in maneuvers:
        emu.reset_car()
        time.sleep(0.3)
        cmd.start_pass(m["name"], [tuple(x) for x in m["schedule"]])
        time.sleep(m["duration_s"] + 0.5)
    cmd.hold()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-speed", type=float, default=3.0)
    ap.add_argument("--n", type=int, default=3, help="maneuvers per round")
    args = ap.parse_args(argv)

    car = random_car(np.random.default_rng(args.seed))
    truth = M.default_params(**car)
    limits = SD.Limits(max_speed_mps=args.max_speed)
    logs = tempfile.mkdtemp(prefix="sysid_rehearsal_")
    sessions, driven = [], 0.0
    rep = R.run([], limits, n_suggest=args.n)  # round 0: no data, plan from the prior
    for rnd in range(1, args.rounds + 1):
        print("\n=== round %d: driving %d maneuver(s) ===" % (rnd, len(rep["suggestions"])))
        for s in rep["suggestions"]:
            print("   ", s["name"].replace("suggested: ", ""), "-", s["why"])
        gs = G.GroundStation.simulated(log_dir=logs, emulator_params=car)
        try:
            gs.emulator.set_rc(on=True, ch3=1900)
            time.sleep(0.3)
            gs.commander.hold()
            time.sleep(0.3)
            gs.emulator.set_rc(ch3=1100)
            time.sleep(0.2)
            drive(gs, rep["suggestions"])
            driven += sum(s["duration_s"] for s in rep["suggestions"])
            sessions.append(gs.log.path)
        finally:
            gs.close()
        t0 = time.time()
        rep = R.run(sessions, limits, n_suggest=args.n)
        print("\n--- fit after round %d (%.0f s of maneuvers, fit %.0f s) ---" % (rnd, driven, time.time() - t0))
        print(R.format_report(rep))
        if rep["gate_pass"]:
            print("\nGATE A PASSED after %d round(s)" % rnd)
            break

    print("\n=== fitted vs the emulator's true car ===")
    print("  %-16s %11s %11s %8s %10s" % ("param", "true", "fitted", "error", "1-sigma"))
    for r in rep["gate"]:
        k = r["param"]
        if k == "gyro_bias":
            continue
        err = rep["params"][k] - truth[k]
        rel = "%+.1f%%" % (100 * err / truth[k]) if truth[k] else "%+.3g" % err
        print("  %-16s %11.5g %11.5g %8s %10s" % (k, truth[k], rep["params"][k], rel, r["std_text"]))
    print("  (fitted input_delay includes the ESP32's 50 Hz servo frame, ~10 ms on average)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

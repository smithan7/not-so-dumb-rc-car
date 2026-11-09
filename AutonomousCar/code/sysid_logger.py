"""Record every serial sensor stream for system ID, without falling behind.

    python sysid_logger.py --duration 300 --notes "throttle steps, parking lot"
    python sysid_logger.py --port ola=/dev/tty.usbmodem1101 --port pwm=/dev/tty.usbserial-0001

Design (fixes for the 2023 data_logger_good_place notebook):
- one reader thread per port, so a busy stream (the OLA at ~90 Hz) can't starve the others
- every line is written raw with a PC monotonic timestamp; nothing is parsed or dropped while
  recording (the old DataFrame.loc appends got slower as the log grew, so the logger fell
  behind by up to 3 minutes)
- streams keep their own device clocks in the raw text; sysid_log.load_session() aligns them

Output: <out>/session_YYYYmmdd_HHMMSS/{meta.json, <stream>.csv}; load with sysid_log.load_session().
"""

import argparse
import collections
import csv
import datetime
import json
import os
import platform
import signal
import sys
import threading
import time

from sysid_log import classify_line

DEFAULT_BAUD = 115200


def _open_serial(port, baud, timeout):
    import serial  # imported lazily so parsing/tests don't need pyserial

    return serial.Serial(port, baud, timeout=timeout)


def list_candidate_ports():
    from serial.tools import list_ports

    skip = ("bluetooth", "debug-console")
    return [p.device for p in list_ports.comports() if not any(s in p.device.lower() for s in skip)]


class StreamRecorder(threading.Thread):
    """Reads lines from one serial port and appends ``t_pc_ns,line`` rows to a CSV file."""

    def __init__(self, name, port, baud, path, stop_event, opener=_open_serial, flush_s=0.5):
        super().__init__(name=f"rec-{name}", daemon=True)
        self.stream = name
        self.port = port
        self.baud = baud
        self.path = path
        self.stop_event = stop_event
        self.opener = opener
        self.flush_s = flush_s
        self.count = 0
        self.error = None

    def run(self):
        try:
            ser = self.opener(self.port, self.baud, 0.2)
        except Exception as e:  # report, don't kill the other streams
            self.error = f"could not open {self.port}: {e}"
            return
        try:
            with open(self.path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_pc_ns", "line"])
                last_flush = time.monotonic()
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    t = time.monotonic_ns()  # stamp as soon as the line is complete
                    if raw:
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        w.writerow([t, line])
                        self.count += 1
                    if time.monotonic() - last_flush > self.flush_s:
                        fh.flush()
                        last_flush = time.monotonic()
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
        finally:
            try:
                ser.close()
            except Exception:
                pass


def identify_ports(ports, baud=DEFAULT_BAUD, listen_s=3.0, opener=_open_serial):
    """Listen to each port in parallel and name it by the lines it sends -> {kind: port}.

    Opening an Arduino resets it, so listen long enough to get past its boot output.
    """
    votes = {}

    def listen(port):
        c = collections.Counter()
        try:
            ser = opener(port, baud, 0.2)
        except Exception:
            return
        try:
            end = time.monotonic() + listen_s
            while time.monotonic() < end:
                raw = ser.readline()
                if raw:
                    kind = classify_line(raw.decode("utf-8", errors="replace"))
                    if kind:
                        c[kind] += 1
        except Exception:
            pass
        finally:
            ser.close()
        votes[port] = c

    threads = [threading.Thread(target=listen, args=(p,), daemon=True) for p in ports]
    for th in threads:
        th.start()
    for th in threads:
        th.join(listen_s + 2.0)

    found = {}
    for port, c in votes.items():
        if not c:
            continue
        kind, _ = c.most_common(1)[0]
        if kind in found:
            # two ports claim the same stream: keep both, numbered
            kind = f"{kind}{sum(k.startswith(kind) for k in found) + 1}"
        found[kind] = port
    return found


def record(ports, out_dir, duration_s=0.0, baud=DEFAULT_BAUD, notes="", opener=_open_serial,
           status_every_s=2.0, stop_event=None, printer=print):
    """Record ``ports`` ({stream_name: port}) into a new session directory. Returns its path."""
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = os.path.join(out_dir, f"session_{stamp}")
    os.makedirs(session, exist_ok=False)
    stop_event = stop_event or threading.Event()

    start_ns = time.monotonic_ns()
    meta = {"start_wall": datetime.datetime.now().astimezone().isoformat(),
            "start_monotonic_ns": start_ns,
            "host": platform.node(), "python": sys.version.split()[0],
            "notes": notes, "duration_requested_s": duration_s, "streams": {}}
    recorders = []
    for name, port in ports.items():
        kind = "".join(ch for ch in name if not ch.isdigit())
        fname = f"{name}.csv"
        meta["streams"][name] = {"port": port, "baud": baud, "kind": kind, "file": fname}
        recorders.append(StreamRecorder(name, port, baud, os.path.join(session, fname), stop_event, opener))

    def write_meta():
        with open(os.path.join(session, "meta.json"), "w") as fh:
            json.dump(meta, fh, indent=2)

    write_meta()
    for r in recorders:
        r.start()

    last = {r.stream: 0 for r in recorders}
    t_last = time.monotonic()
    try:
        while not stop_event.is_set():
            elapsed = (time.monotonic_ns() - start_ns) / 1e9
            if duration_s and elapsed >= duration_s:
                break
            time.sleep(0.05)
            if time.monotonic() - t_last >= status_every_s:
                dt = time.monotonic() - t_last
                t_last = time.monotonic()
                parts = []
                for r in recorders:
                    rate = (r.count - last[r.stream]) / dt
                    last[r.stream] = r.count
                    parts.append(f"{r.stream} {rate:5.1f} Hz" + (f" [{r.error}]" if r.error else ""))
                printer(f"[{elapsed:6.1f} s] " + " | ".join(parts))
    finally:
        stop_event.set()
        for r in recorders:
            r.join(2.0)
        meta["duration_s"] = (time.monotonic_ns() - start_ns) / 1e9
        for r in recorders:
            meta["streams"][r.stream]["lines"] = r.count
            if r.error:
                meta["streams"][r.stream]["error"] = r.error
        write_meta()
    return session


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"),
                    help="directory for session folders (default: AutonomousCar/logs)")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to record; 0 = until Ctrl-C")
    ap.add_argument("--port", action="append", default=[], metavar="NAME=PORT",
                    help="stream name and port, e.g. ola=/dev/tty.usbmodem1101 (repeatable); "
                         "skips auto-detection")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--notes", default="", help="free text saved in meta.json (what was tested, where)")
    args = ap.parse_args(argv)

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial is not installed: pip install -r requirements.txt", file=sys.stderr)
        return 1

    if args.port:
        ports = dict(p.split("=", 1) for p in args.port)
    else:
        candidates = list_candidate_ports()
        print("listening on:", ", ".join(candidates) or "(no serial ports found)")
        ports = identify_ports(candidates, args.baud)
    if not ports:
        print("no sensor streams found; pass --port NAME=PORT", file=sys.stderr)
        return 1
    for name, port in ports.items():
        print(f"  {name:>5} <- {port}")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    session = record(ports, os.path.abspath(args.out), args.duration, args.baud, args.notes, stop_event=stop)
    print("saved", session)

    from sysid_log import load_session, summarize

    print(summarize(load_session(session)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

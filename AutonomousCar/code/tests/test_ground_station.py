"""Ground station web server, end to end against the car emulator (no browser needed)."""

import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

pytest.importorskip("serial")
import ground_station as G  # noqa: E402
import esp32_protocol as P  # noqa: E402
import sysid_log  # noqa: E402


@pytest.fixture
def station(tmp_path):
    gs = G.GroundStation.simulated(log_dir=str(tmp_path))
    srv = G.make_server(gs, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    yield gs, base
    srv.shutdown()
    srv.server_close()
    gs.close()


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=3) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def post(base, path, body=None):
    req = urllib.request.Request(base + path, data=json.dumps(body or {}).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def snapshot(base):
    """First Server-Sent Event from /api/stream."""
    with urllib.request.urlopen(base + "/api/stream", timeout=3) as r:
        assert "text/event-stream" in r.headers.get("Content-Type", "")
        for raw in r:
            line = raw.decode().strip()
            if line.startswith("data: "):
                return json.loads(line[6:])


def wait_snapshot(base, pred, seconds=2.0):
    t_end = time.time() + seconds
    while time.time() < t_end:
        s = snapshot(base)
        if pred(s):
            return s
        time.sleep(0.05)
    return None


def test_serves_the_page_and_assets(station):
    _, base = station
    status, ctype, body = get(base, "/")
    assert status == 200 and "text/html" in ctype and b"<title>" in body
    for asset in ("/static/app.js", "/static/style.css"):
        status, _, body = get(base, asset)
        assert status == 200 and len(body) > 100
    with pytest.raises(urllib.error.HTTPError):
        get(base, "/static/../ground_station.py")


def test_config_describes_course_passes_limits(station):
    _, base = station
    cfg = json.loads(get(base, "/api/config")[2])
    assert cfg["sim"] is True
    assert len(cfg["course"]["left"]) > 100 and len(cfg["course"]["right"]) == len(cfg["course"]["left"])
    assert cfg["limits"]["auto_speed_max_us"] == 1600
    assert cfg["drivetrain"]["meters_per_edge"] == pytest.approx(P.meters_per_edge())
    names = [p["name"] for p in cfg["passes"]]
    assert len(names) == len(set(names)) >= 10
    p0 = cfg["passes"][0]
    assert {"name", "kind", "duration_s", "top_speed_mps", "max_throttle_us", "clamped"} <= set(p0)


def test_stream_has_status_link_truth(station):
    _, base = station
    s = wait_snapshot(base, lambda s: s["status"] is not None)
    assert s["link"]["connected"] is True
    assert s["status"]["mode"] == P.MODE_FAILSAFE_NO_RC  # virtual transmitter starts off
    assert {"x", "y", "psi", "v"} <= set(s["truth"])
    assert s["cmd"]["state"] == "off"
    assert s["rc"]["on"] is False


def arm(base):
    assert post(base, "/api/sim/rc", {"on": True, "ch3": 1900})[0] == 200
    assert wait_snapshot(base, lambda s: s["status"] and s["status"]["mode"] == P.MODE_MANUAL)
    assert post(base, "/api/hold")[0] == 200
    # the Jetson must already be streaming when ch3 goes to AUTO, or the firmware fails safe
    # (no fresh Jetson command) and disarms - same on the real car
    assert wait_snapshot(base, lambda s: 0 <= s["status"]["jetson_age_ms"] < 100)
    assert post(base, "/api/sim/rc", {"ch3": 1100})[0] == 200
    assert wait_snapshot(base, lambda s: s["status"]["mode"] == P.MODE_AUTO)


def test_jog_is_a_deadman(station):
    _, base = station
    arm(base)
    assert post(base, "/api/jog", {"speed_us": 1570, "steer_us": 1550})[0] == 200
    s = wait_snapshot(base, lambda s: s["status"]["out_speed_us"] == 1570, seconds=1.0)
    assert s is not None and s["cmd"]["state"] == "jog" and s["status"]["out_steer_us"] == 1550
    # no more jog requests: back to neutral hold within ~0.3 s
    s = wait_snapshot(base, lambda s: s["cmd"]["state"] == "hold" and s["status"]["out_speed_us"] == 1500,
                      seconds=1.0)
    assert s is not None


def test_jog_rejects_out_of_range(station):
    _, base = station
    assert post(base, "/api/jog", {"speed_us": 3000, "steer_us": 1500})[0] == 400
    assert post(base, "/api/jog", {"speed_us": "fast"})[0] == 400


def raw_post(base, path, data, headers):
    req = urllib.request.Request(base + path, data=data, method="POST", headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status
    except urllib.error.HTTPError as e:
        return e.code


def test_cross_site_requests_cannot_drive_the_car(station):
    """A web page elsewhere in the operator's browser must not be able to command the car."""
    _, base = station
    body = json.dumps({"speed_us": 1700, "steer_us": 1500}).encode()
    # a plain cross-site form/fetch POST (no preflight) uses a "simple" content type
    assert raw_post(base, "/api/jog", body, {"Content-Type": "text/plain"}) == 415
    assert raw_post(base, "/api/jog", body, {}) == 415
    # JSON from another origin
    assert raw_post(base, "/api/jog", body, {"Content-Type": "application/json",
                                             "Origin": "http://evil.example"}) == 403
    # DNS rebinding: attacker's name resolving to 127.0.0.1 -> same-origin, but Host is theirs
    assert raw_post(base, "/api/jog", body, {"Content-Type": "application/json",
                                             "Host": "evil.example:8080"}) == 403
    req = urllib.request.Request(base + "/api/config", headers={"Host": "evil.example"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req, timeout=3)
    assert e.value.code == 403
    # the page itself: same origin, JSON
    port = base.rsplit(":", 1)[1]
    assert raw_post(base, "/api/hold", b"{}", {"Content-Type": "application/json; charset=utf-8",
                                               "Origin": "http://127.0.0.1:" + port}) == 200
    assert raw_post(base, "/api/hold", b"{}", {"Content-Type": "application/json",
                                               "Host": "localhost:" + port, "Origin": "http://localhost:" + port}) == 200


def test_run_and_stop_a_pass(station):
    gs, base = station
    arm(base)
    name = min(json.loads(get(base, "/api/config")[2])["passes"], key=lambda p: p["duration_s"])["name"]
    assert post(base, "/api/pass/start", {"name": name})[0] == 200
    s = wait_snapshot(base, lambda s: s["cmd"]["state"] == "pass" and s["cmd"]["pass"]["t"] > 0.2)
    assert s is not None and s["cmd"]["pass"]["name"] == name
    assert post(base, "/api/pass/stop")[0] == 200
    assert wait_snapshot(base, lambda s: s["cmd"]["state"] == "hold")
    assert post(base, "/api/pass/start", {"name": "no such pass"})[0] == 400


def test_custom_step_pass(station):
    _, base = station
    arm(base)
    assert post(base, "/api/pass/start", {"custom": {"throttle_us": 1560, "hold_s": 1.0, "steer_us": 1500}})[0] == 200
    s = wait_snapshot(base, lambda s: s["status"]["out_speed_us"] == 1560, seconds=5.0)
    assert s is not None and s["cmd"]["pass"]["name"].startswith("custom")


def test_release_stops_streaming_and_firmware_fails_safe(station):
    _, base = station
    arm(base)
    assert post(base, "/api/release")[0] == 200
    s = wait_snapshot(base, lambda s: s["status"]["mode"] == P.MODE_FAILSAFE_NO_JETSON, seconds=1.0)
    assert s is not None and s["cmd"]["state"] == "off"


def test_session_log_loads_in_sysid_log(station, tmp_path):
    gs, base = station
    arm(base)
    post(base, "/api/jog", {"speed_us": 1560, "steer_us": 1500})
    time.sleep(0.5)
    session = gs.log.path
    gs.close()  # flushes and finalises meta.json
    sess = sysid_log.load_session(session)
    esp = sess["esp32"]
    assert len(esp) > 20 and {"t", "mode", "out_speed_us", "enc_count"} <= set(esp.columns)
    assert esp["t"].is_monotonic_increasing
    assert (esp["kind"] == "A").any() and (esp["kind"] == "S").any()
    assert len(sess["cmd"]) > 10  # commands sent, raw
    assert sess["meta"]["streams"]["esp32"]["kind"] == "esp32"
    ola = sess["ola"]  # the IMU, logged alongside and time-aligned
    assert len(ola) > 40 and "gz_raw" in ola.columns and ola["t"].is_monotonic_increasing


def test_fit_and_suggest_from_the_page(station):
    gs, base = station
    arm(base)
    assert post(base, "/api/pass/start", {"custom": {"throttle_us": 1570, "hold_s": 2.0, "steer_us": 1500}})[0] == 200
    time.sleep(6.5)
    status, body = post(base, "/api/sysid/run", {"max_speed_mps": 2.0, "n": 2})
    assert status == 200 and body["sysid"]["state"] == "running"
    assert post(base, "/api/sysid/run", {"max_speed_mps": 2.0})[0] == 409  # one at a time
    t_end = time.time() + 300
    while time.time() < t_end:
        sy = json.loads(get(base, "/api/sysid")[2])
        if sy["state"] != "running":
            break
        time.sleep(0.5)
    assert sy["state"] == "done", sy.get("error")
    rep = sy["report"]
    assert rep["pieces"] and {r["param"] for r in rep["gate"]} >= {"speed_gain", "input_delay"}
    assert rep["suggestions"]
    names = [p["name"] for p in json.loads(get(base, "/api/config")[2])["passes"]]
    assert rep["suggestions"][0]["name"] in names  # runnable from the pass list
    assert post(base, "/api/pass/start", {"name": rep["suggestions"][0]["name"]})[0] == 200
    assert post(base, "/api/sysid/run", {"max_speed_mps": 99})[0] == 400


def test_live_yaw_rate_from_imu(station):
    _, base = station
    s = wait_snapshot(base, lambda s: s["imu"] is not None and s["imu"]["yaw_rate"] is not None)
    assert s is not None and abs(s["imu"]["yaw_rate"]) < 0.2  # parked: gyro bias + noise only

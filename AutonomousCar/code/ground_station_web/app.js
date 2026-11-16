// Ground station page: live status over Server-Sent Events, controls over JSON POSTs.
"use strict";

const $ = (id) => document.getElementById(id);
const WINDOW_S = 20;        // chart history shown
const KEEP_S = 60;          // history kept
const TRAIL_POINTS = 4000;  // map trail
const JOG_PERIOD_MS = 100;  // server reverts to neutral 300 ms after the last jog

let cfg = null;
let last = null;            // latest snapshot
let lastMsgAt = 0;
const hist = [];            // {t, ...values}
let trail = [];
let dirty = true;

// ---------------------------------------------------------------- colours (follow the theme)
let C = {};
function readColours() {
  const cs = getComputedStyle(document.documentElement);
  for (const k of ["ink", "muted", "line", "grid", "card", "bg", "s1", "s2", "s3", "s4", "edge", "trail", "car", "accent"])
    C[k] = cs.getPropertyValue("--" + k).trim();
  dirty = true;
}
readColours();
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", readColours);

// ---------------------------------------------------------------- helpers
async function post(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || r.statusText);
    say("");
    return j;
  } catch (e) {
    say(String(e.message || e), true);
    return null;
  }
}
function say(text, err) { const m = $("msg"); m.textContent = text; m.className = "msg" + (err ? " err" : ""); }
const fmt = (v, d = 0) => (v === null || v === undefined || Number.isNaN(v) ? "–" : Number(v).toFixed(d));
const fresh = (age, limit) => age !== null && age !== undefined && age >= 0 && age <= limit;

function sizeCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(cv.clientWidth * dpr)), h = Math.max(1, Math.round(cv.clientHeight * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [ctx, cv.clientWidth, cv.clientHeight];
}

// ---------------------------------------------------------------- charts
const CHARTS = {
  thr: { el: "ch-thr", legend: "lg-thr", range: [1300, 1800], step: 100, series: [
    { key: "rc_speed", label: "receiver", c: "s1" }, { key: "j_speed", label: "Jetson cmd", c: "s2" },
    { key: "out_speed", label: "output", c: "s3", w: 2 }] },
  str: { el: "ch-str", legend: "lg-str", range: [1100, 1900], step: 200, series: [
    { key: "rc_steer", label: "receiver", c: "s1" }, { key: "j_steer", label: "Jetson cmd", c: "s2" },
    { key: "out_steer", label: "output", c: "s3", w: 2 }] },
  spd: { el: "ch-spd", legend: "lg-spd", range: [0, 4], auto: true, series: [
    { key: "speed", label: "encoder m/s", c: "s3", w: 2 }, { key: "truth_v", label: "true (emulator)", c: "s4", dash: [4, 3] },
    { key: "yaw", label: "yaw rate rad/s", c: "s1" }] },
  lnk: { el: "ch-lnk", legend: "lg-lnk", range: [0, 10], auto: true, series: [
    { key: "rtt", label: "round trip", c: "s3", w: 2 }, { key: "j_age", label: "Jetson cmd age", c: "s2", alpha: 0.45, w: 1 },
    { key: "rc_age", label: "receiver age", c: "s1" }] },
};

function buildLegends() {
  for (const ch of Object.values(CHARTS)) {
    const box = $(ch.legend);
    box.replaceChildren(...ch.series.map((s) => {
      const span = document.createElement("span"), sw = document.createElement("i");
      sw.style.background = `var(--${s.c})`;
      span.append(sw, s.label);
      return span;
    }));
  }
}

function niceStep(span) {
  const raw = span / 4, p = Math.pow(10, Math.floor(Math.log10(raw)));
  return [1, 2, 2.5, 5, 10].map((m) => m * p).find((s) => s >= raw) || raw;
}

function drawChart(ch, tNow) {
  const [ctx, W, H] = sizeCanvas($(ch.el));
  const L = 42, R = 8, T = 8, B = 20;
  ctx.clearRect(0, 0, W, H);
  const t0 = tNow - WINDOW_S;
  const pts = hist.filter((h) => h.t >= t0);
  let [lo, hi] = ch.range;
  if (ch.auto) {
    for (const h of pts) for (const s of ch.series) {
      const v = h[s.key];
      if (v === null || v === undefined) continue;
      if (v > hi) hi = v;
      if (v < lo) lo = v;
    }
    hi = Math.ceil(hi * 1.1 * 10) / 10;
    if (lo < 0) lo = Math.floor(lo * 1.1 * 10) / 10;
  }
  const step = ch.step || niceStep(hi - lo);
  const X = (t) => L + ((t - t0) / WINDOW_S) * (W - L - R);
  const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (H - T - B);
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillStyle = C.muted; ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) {
    if (v === lo && lo !== 0) continue;  // keep the corner clear for the time labels
    const y = Math.round(Y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
    ctx.fillText(step < 1 ? v.toFixed(1) : String(Math.round(v)), L - 6, y);
  }
  ctx.textBaseline = "top";
  for (let s = 0; s <= WINDOW_S; s += 5) {
    ctx.textAlign = s === 0 ? "left" : s === WINDOW_S ? "right" : "center";
    ctx.fillText(s === WINDOW_S ? "now" : `-${WINDOW_S - s}s`, X(t0 + s), H - B + 5);
  }
  // widest line (the actual output) first, so equal-valued inputs stay visible on top of it
  for (const s of [...ch.series].sort((a, b) => (b.w || 1.5) - (a.w || 1.5))) {
    ctx.strokeStyle = C[s.c]; ctx.lineWidth = s.w || 1.5; ctx.setLineDash(s.dash || []); ctx.globalAlpha = s.alpha || 1;
    ctx.beginPath();
    let pen = false;
    for (const h of pts) {
      const v = h[s.key];
      if (v === null || v === undefined) { pen = false; continue; }
      const x = X(h.t), y = Y(Math.min(Math.max(v, lo), hi));
      if (pen) ctx.lineTo(x, y); else { ctx.moveTo(x, y); pen = true; }
    }
    ctx.stroke();
  }
  ctx.setLineDash([]); ctx.globalAlpha = 1;
}

// ---------------------------------------------------------------- map
let view = null;  // world bounds
function drawMap() {
  const cv = $("map");
  const [ctx, W, H] = sizeCanvas(cv);
  ctx.clearRect(0, 0, W, H);
  if (!cfg) return;
  const course = cfg.course;
  if (!view) {
    const all = course.left.concat(course.right);
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]);
    view = { x0: Math.min(0, ...xs) - 2, x1: Math.max(40, ...xs) + 2, y0: Math.min(0, ...ys) - 2, y1: Math.max(25, ...ys) + 2 };
  }
  const s = Math.min(W / (view.x1 - view.x0), H / (view.y1 - view.y0));
  const ox = (W - s * (view.x1 - view.x0)) / 2, oy = (H - s * (view.y1 - view.y0)) / 2;
  const X = (x) => ox + (x - view.x0) * s, Y = (y) => H - oy - (y - view.y0) * s;  // y up

  // 5 m grid
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  for (let x = Math.ceil(view.x0 / 5) * 5; x <= view.x1; x += 5) { ctx.beginPath(); ctx.moveTo(X(x), Y(view.y0)); ctx.lineTo(X(x), Y(view.y1)); ctx.stroke(); }
  for (let y = Math.ceil(view.y0 / 5) * 5; y <= view.y1; y += 5) { ctx.beginPath(); ctx.moveTo(X(view.x0), Y(y)); ctx.lineTo(X(view.x1), Y(y)); ctx.stroke(); }
  // course
  const poly = (pts, close) => { ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); if (close) ctx.closePath(); ctx.stroke(); };
  ctx.strokeStyle = C.edge; ctx.lineWidth = 2; poly(course.left, true); poly(course.right, true);
  ctx.setLineDash([3, 5]); ctx.lineWidth = 1; poly(course.center, true); ctx.setLineDash([]);
  // scale bar
  ctx.strokeStyle = C.muted; ctx.fillStyle = C.muted; ctx.lineWidth = 2;
  const sx = X(view.x0) + 12, sy = Y(view.y0) - 12;
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + 5 * s, sy); ctx.stroke();
  ctx.font = "11px system-ui, sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "bottom"; ctx.fillText("5 m", sx, sy - 3);
  // trail + car
  if (trail.length > 1) {
    ctx.strokeStyle = C.trail; ctx.lineWidth = 2; ctx.globalAlpha = 0.8;
    poly(trail, false); ctx.globalAlpha = 1;
  }
  const pose = last && last.truth;
  if (pose) {
    const L = Math.max(18, 0.6 * s), Wd = Math.max(11, 0.3 * s);
    ctx.save(); ctx.translate(X(pose.x), Y(pose.y)); ctx.rotate(-pose.psi);
    ctx.fillStyle = C.car; ctx.strokeStyle = C.card; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(L / 2, 0); ctx.lineTo(-L / 2, Wd / 2); ctx.lineTo(-L / 3, 0); ctx.lineTo(-L / 2, -Wd / 2); ctx.closePath();
    ctx.fill(); ctx.stroke(); ctx.restore();
  }
}

// ---------------------------------------------------------------- status
function modeClass(m) {
  if (m === 0) return "mode-manual";
  if (m === 1) return "mode-auto";
  if (m === -3) return "mode-warn";
  if (m === -1 || m === -2) return "mode-fail";
  return "mode-unknown";
}

function renderStatus() {
  const stale = Date.now() - lastMsgAt > 1500;
  const st = last && last.status, lk = last && last.link;
  const carStale = !st || !lk || lk.status_age_ms === null || lk.status_age_ms > 500;
  const mode = $("mode");
  if (stale) { mode.className = "mode mode-unknown"; $("mode-name").textContent = "no ground station"; }
  else if (carStale) { mode.className = "mode mode-unknown"; $("mode-name").textContent = lk && lk.connected ? "no status from car" : "serial disconnected"; }
  else { mode.className = "mode " + modeClass(st.mode); $("mode-name").textContent = st.mode_name; }
  const ok = !stale && !carStale;
  $("armed").textContent = ok ? (st.armed ? "yes" : "no") : "–";
  $("armed").className = ok ? (st.armed ? "ok" : "") : "";
  $("speed").textContent = ok ? fmt(st.speed_mps, 2) : "–";
  $("rc-age").textContent = ok ? (st.rc_age_ms < 0 ? "never" : fmt(st.rc_age_ms)) : "–";
  $("rc-age").className = ok && fresh(st.rc_age_ms, 100) ? "" : "bad";
  $("j-age").textContent = ok ? (st.jetson_age_ms < 0 ? "never" : fmt(st.jetson_age_ms)) : "–";
  $("j-age").className = ok && fresh(st.jetson_age_ms, 200) ? "" : "warn";
  $("rtt").textContent = lk ? `${fmt(lk.rtt_ms, 1)} / p95 ${fmt(lk.rtt_p95_ms, 1)}` : "–";
  $("link").textContent = lk ? (lk.connected ? "connected" : "down") : "–";
  $("link").className = lk && lk.connected ? "ok" : "bad";
  if (last) {
    const cm = last.cmd;
    $("cmd-state").textContent = { off: "not sending", hold: "sending neutral", jog: "driving (jog)", pass: "running pass" }[cm.state] || cm.state;
    $("cmd-state").className = "chip st-" + cm.state;
    const p = cm.pass;
    $("pass-progress").style.width = p ? `${Math.min(100, (100 * p.t) / p.duration)}%` : "0";
    $("pass-now").textContent = p ? `${p.name}: ${fmt(p.t, 1)} / ${fmt(p.duration, 1)} s` : "";
    if (last.truth) $("pose").textContent = `x ${fmt(last.truth.x, 1)} m, y ${fmt(last.truth.y, 1)} m, heading ${fmt((last.truth.psi * 180) / Math.PI % 360, 0)}°, odometer ${fmt(last.truth.odometer_m, 1)} m (true pose, emulator)`;
    if (last.rc) syncTx(last.rc);
    $("foot-lines").textContent = lk ? `${lk.port} · ${lk.lines} lines · ${lk.unparsed} unparsed${lk.error ? " · " + lk.error : ""}` : "";
  }
  const note = $("map-note");
  if (cfg && !cfg.sim) { note.hidden = false; note.textContent = "No position source yet: the map shows the car once GPS / the UKF is connected."; }
}

function record(s) {
  const st = s.status;
  const h = { t: s.t };
  if (st) {
    const rcOk = fresh(st.rc_age_ms, 100), jOk = fresh(st.jetson_age_ms, 200);
    Object.assign(h, {
      rc_speed: rcOk ? st.rc_speed_us : null, rc_steer: rcOk ? st.rc_steer_us : null,
      j_speed: jOk ? st.jetson_speed_us : null, j_steer: jOk ? st.jetson_steer_us : null,
      out_speed: st.out_speed_us, out_steer: st.out_steer_us, speed: st.speed_mps,
      rc_age: st.rc_age_ms >= 0 ? Math.min(st.rc_age_ms, 1000) : null,
      j_age: st.jetson_age_ms >= 0 ? Math.min(st.jetson_age_ms, 1000) : null,
    });
  }
  h.rtt = s.link.rtt_ms;
  h.truth_v = s.truth ? s.truth.v : null;
  h.yaw = s.imu ? s.imu.yaw_rate : null;
  hist.push(h);
  while (hist.length && hist[0].t < s.t - KEEP_S) hist.shift();
  if (s.truth) {
    const p = trail[trail.length - 1];
    if (!p || Math.hypot(p[0] - s.truth.x, p[1] - s.truth.y) > 0.05) trail.push([s.truth.x, s.truth.y]);
    if (trail.length > TRAIL_POINTS) trail = trail.slice(-TRAIL_POINTS);
  }
}

function frame() {
  if (dirty) {
    dirty = false;
    renderStatus();
    const tNow = hist.length ? hist[hist.length - 1].t : 0;
    for (const ch of Object.values(CHARTS)) drawChart(ch, tNow);
    drawMap();
  }
  requestAnimationFrame(frame);
}
setInterval(() => { dirty = true; }, 500);  // keep "stale" indicators honest when no data arrives
window.addEventListener("resize", () => { dirty = true; });

// ---------------------------------------------------------------- controls
function bindSlider(id) {
  const el = $(id), out = $(id + "-v");
  el.addEventListener("input", () => { out.textContent = el.value; });
  return el;
}
const jogThr = bindSlider("jog-thr"), jogStr = bindSlider("jog-str");
$("btn-hold").onclick = () => post("/api/hold");
$("btn-release").onclick = () => post("/api/release");

// hold-to-drive: requests repeat while held; the server falls back to neutral 300 ms after the last
let jogTimer = null;
function jogStart(e) {
  if (e) e.preventDefault();
  if (jogTimer) return;
  $("btn-jog").classList.add("active");
  const send = () => post("/api/jog", { speed_us: +jogThr.value, steer_us: +jogStr.value });
  send();
  jogTimer = setInterval(send, JOG_PERIOD_MS);
}
function jogStop() {
  if (!jogTimer) return;
  clearInterval(jogTimer); jogTimer = null;
  $("btn-jog").classList.remove("active");
}
const jb = $("btn-jog");
jb.addEventListener("pointerdown", (e) => { jb.setPointerCapture(e.pointerId); jogStart(e); });
for (const ev of ["pointerup", "pointercancel", "lostpointercapture"]) jb.addEventListener(ev, jogStop);
jb.addEventListener("keydown", (e) => { if ((e.key === " " || e.key === "Enter") && !e.repeat) jogStart(e); });
jb.addEventListener("keyup", (e) => { if (e.key === " " || e.key === "Enter") jogStop(); });
jb.addEventListener("blur", jogStop);
window.addEventListener("blur", jogStop);
document.addEventListener("visibilitychange", () => { if (document.hidden) jogStop(); });
jb.addEventListener("contextmenu", (e) => e.preventDefault());

function passInfo() {
  const p = cfg && cfg.passes.find((q) => q.name === $("pass-select").value);
  if (!p) { $("pass-info").textContent = ""; return; }
  $("pass-info").textContent = (p.why ? p.why + ". " : "") + `${fmt(p.duration_s, 1)} s, up to ${p.max_throttle_us} µs` +
    (p.clamped ? ` (firmware clamps to ${cfg.limits.auto_speed_max_us})` : "") +
    `; prior model: top ${fmt(p.top_speed_mps, 1)} m/s, path ${fmt(p.extent_m[0], 0)} × ${fmt(p.extent_m[1], 0)} m.` +
    " The prior's speed gain is a placeholder - start with small custom steps.";
}
$("pass-select").onchange = passInfo;
$("btn-pass").onclick = () => post("/api/pass/start", { name: $("pass-select").value });
$("btn-pass-stop").onclick = () => post("/api/pass/stop");
$("btn-custom").onclick = () => post("/api/pass/start", { custom: {
  throttle_us: +$("cs-thr").value, hold_s: +$("cs-hold").value, steer_us: +$("cs-str").value } });
$("clear-trail").onclick = () => { trail = []; dirty = true; };

// virtual transmitter (emulator)
let txSyncedOnce = false;
function syncTx(rc) {
  const pw = $("tx-power"), c3 = $("tx-ch3");
  pw.setAttribute("aria-pressed", String(rc.on)); pw.textContent = rc.on ? "Transmitter on" : "Transmitter off";
  const manual = rc.ch3 > 1500;
  c3.setAttribute("aria-pressed", String(manual)); c3.textContent = manual ? "ch3: manual" : "ch3: auto";
  if (!txSyncedOnce) {
    txSyncedOnce = true;
    $("tx-thr").value = rc.speed_us; $("tx-thr-v").textContent = rc.speed_us;
    $("tx-str").value = rc.steer_us; $("tx-str-v").textContent = rc.steer_us;
  }
}
const txThr = bindSlider("tx-thr"), txStr = bindSlider("tx-str");
let txPending = null;
function sendSticks() {
  if (txPending) return;
  txPending = setTimeout(() => { txPending = null; post("/api/sim/rc", { speed_us: +txThr.value, steer_us: +txStr.value }); }, 50);
}
txThr.addEventListener("input", sendSticks); txStr.addEventListener("input", sendSticks);
$("tx-power").onclick = () => post("/api/sim/rc", { on: !(last && last.rc && last.rc.on) });
$("tx-ch3").onclick = () => post("/api/sim/rc", { ch3: last && last.rc && last.rc.ch3 > 1500 ? 1100 : 1900 });
$("tx-center").onclick = () => {
  txThr.value = 1500; txStr.value = 1500; $("tx-thr-v").textContent = 1500; $("tx-str-v").textContent = 1500;
  post("/api/sim/rc", { speed_us: 1500, steer_us: 1500 });
};
$("sim-reset").onclick = () => { trail = []; post("/api/sim/reset"); };

// ---------------------------------------------------------------- system ID
let sysidSeen = null;
function el(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text !== undefined) e.textContent = text; return e; }
function fmtVal(v) { return Math.abs(v) >= 1000 ? v.toFixed(1) : Number(v).toPrecision(4); }

async function refreshPasses() {
  cfg = await (await fetch("/api/config")).json();
  const sel = $("pass-select"), keep = sel.value;
  sel.replaceChildren();
  const groups = [["suggested", "Suggested by system ID"], ["longitudinal", "Longitudinal (speed)"], ["lateral", "Lateral (steering)"]];
  for (const [kind, label] of groups) {
    const ps = cfg.passes.filter((q) => q.kind === kind);
    if (!ps.length) continue;
    const g = document.createElement("optgroup"); g.label = label;
    for (const p of ps) { const o = document.createElement("option"); o.value = p.name; o.textContent = p.name; g.appendChild(o); }
    sel.appendChild(g);
  }
  if ([...sel.options].some((o) => o.value === keep)) sel.value = keep;
  passInfo();
}

function renderSysid(sy) {
  const chip = $("sysid-state");
  chip.textContent = { idle: "not run", running: "fitting…", done: "done", error: "error" }[sy.state] || sy.state;
  chip.className = "chip" + (sy.state === "running" ? " st-pass" : "");
  $("btn-sysid").disabled = sy.state === "running";
  if (sy.state === "error") { say("system ID: " + sy.error, true); return; }
  if (sy.state !== "done" || !sy.report) return;
  const r = sy.report;
  $("sysid-out").hidden = false;
  const v = $("sy-verdict");
  v.textContent = r.gate_pass ? "Gate A passed" : "Gate A not yet";
  v.className = "verdict " + (r.gate_pass ? "pass" : "fail");
  $("sy-data").textContent = `${r.pieces.length} piece(s) of driving` +
    (r.rmse.speed !== null ? `, speed fit rmse ${r.rmse.speed.toFixed(3)} m/s` : "") +
    (r.rmse.yaw !== null ? `, yaw rmse ${r.rmse.yaw.toFixed(3)} rad/s` : "") +
    (r.notes.length ? ". " + r.notes.join("; ") : "") + ". * = gating. 1σ assumes white noise (optimistic).";
  const tb = $("sy-gate").tBodies[0];
  tb.replaceChildren(...r.gate.map((g) => {
    const tr = el("tr", g.gating ? "gating" : "");
    tr.append(el("td", "", g.param), el("td", "", fmtVal(g.value)), el("td", "", g.std_text), el("td", "", g.target_text),
              el("td", g.ok ? "okmark" : "nomark", g.ok ? "✓" : "–"));
    return tr;
  }));
  const checks = [];
  for (const [kind, h] of Object.entries(r.holdout)) {
    checks.push(h === null ? `hold-out ${kind}: needs ≥ 2 pieces of this kind`
      : h.skipped ? `hold-out ${kind}: skipped`
      : `hold-out ${kind}: rmse ${h.rmse.toFixed(3)} (limit ${h.limit}) ${h.ok ? "✓" : "–"}`);
  }
  for (const c of r.correlated) checks.push(`${c.a} / ${c.b} correlated ${c.corr.toFixed(3)}: not separable yet`);
  $("sy-checks").replaceChildren(...checks.map((t) => el("li", "", t)));
  $("sy-suggest").replaceChildren(...(r.suggestions.length ? r.suggestions.map((s) => {
    const li = el("li");
    const run = el("button", "small", "Run");
    run.onclick = () => post("/api/pass/start", { name: s.name });
    li.append(el("div", "name", `${s.name.replace("suggested: ", "")} (${Math.round(s.duration_s)} s) `), el("div", "why", s.why || ""), run);
    return li;
  }) : [el("li", "", "Nothing - every target is predicted to be met.")]));
  $("sy-remaining").textContent = r.remaining_after_plan.length
    ? `Still short after this plan: ${r.remaining_after_plan.join(", ")}. Refit after driving; if it stays short, the speed cap may be too low to excite it.` : "";
}

async function pollSysid(state) {
  if (state === sysidSeen && state !== "running") return;
  sysidSeen = state;
  const sy = await (await fetch("/api/sysid")).json();
  renderSysid(sy);
  if (sy.state === "done") await refreshPasses();
}
$("btn-sysid").onclick = async () => {
  const r = await post("/api/sysid/run", { max_speed_mps: +$("sy-speed").value, n: +$("sy-n").value });
  if (r) { sysidSeen = null; pollSysid("running"); }
};

// ---------------------------------------------------------------- start
async function init() {
  buildLegends();
  cfg = await (await fetch("/api/config")).json();
  $("sim-badge").hidden = !cfg.sim;
  $("sim-panel").hidden = !cfg.sim;
  await refreshPasses();
  $("foot-log").textContent = "logging to " + cfg.log_dir;
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    last = JSON.parse(ev.data);
    lastMsgAt = Date.now();
    record(last);
    dirty = true;
    if (last.sysid && (last.sysid.state !== sysidSeen || last.sysid.state === "running")) pollSysid(last.sysid.state);
  };
  requestAnimationFrame(frame);
}
init();

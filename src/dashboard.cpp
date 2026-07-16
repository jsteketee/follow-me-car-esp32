// HTTP + WebSocket dashboard. Serves a single-page UI at port 80 and pushes
// telemetry JSON at ~10 Hz. Config overrides are applied via POST /config, mode via
// POST /mode, and the bench test sliders over the WebSocket ("direct:<t>,<s>,<p>"
// and "setpoint:<mph>" text frames; POST /direct and /setpoint as fallbacks).
#include "dashboard.h"
#include "uwb.h"
#include "rpm.h"
#include "imu.h"
#include "control.h"
#include "serial_hal.h"
#include "runtime_config.h"
#include "config.h"
#include "utils.h"
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include "esp_log.h"

static const char* TAG = "dash";

static AsyncWebServer _server(80);
static AsyncWebSocket _webSocket("/ws");

static const char DASHBOARD_HTML[] = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Follow-Me Car</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#111;color:#ccc;padding:12px;font-size:14px}
h1{color:#4af;margin-bottom:10px;font-size:1.1em;display:flex;justify-content:space-between;align-items:center}
#connStatus{font-size:0.75em;font-weight:normal;padding:2px 10px;border-radius:12px;border:1px solid #333;color:#555}
.row{display:flex;gap:10px;margin-bottom:10px;flex-wrap:wrap}
.card{background:#1a1a1a;border-radius:6px;padding:10px;flex:1;min-width:160px}
.card h2{font-size:0.75em;color:#555;margin-bottom:8px;text-transform:uppercase;letter-spacing:1px}
.stat{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #222}
.stat:last-child{border-bottom:none}
.val{color:#4f4;font-weight:bold}
.stale{color:#fa4;font-weight:bold}
canvas{width:100%;display:block}
.btn{background:#222;border:1px solid #444;color:#888;padding:5px 14px;border-radius:4px;cursor:pointer;font-family:monospace;font-size:0.9em}
.btn.active{border-color:#4af;color:#4af}
.srow{margin-bottom:10px}
.srow label{display:flex;justify-content:space-between;font-size:0.85em;margin-bottom:3px}
.srow label span:last-child{color:#4af}
input[type=range]{width:100%;accent-color:#4af;cursor:pointer}
input[type=range]:disabled{opacity:0.35;cursor:not-allowed}
@media(max-width:600px){.cfg-row .card{flex:1 1 100%}}
#dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;margin-right:7px;vertical-align:middle;transition:background 0.3s}
#dot.on{background:#4f4}
.pill{padding:2px 10px;border-radius:12px;font-size:0.8em;font-weight:bold;border:1px solid #444;color:#555}
.pill.good{color:#4f4;border-color:#4f4;background:#0a200a}
.pill.deg{color:#fa4;border-color:#fa4;background:#1a1000}
.pill.lost{color:#f44;border-color:#f44;background:#1a0808}
</style>
</head>
<body>
<h1><span><span id="dot"></span>Follow-Me Car</span><span id="connStatus">Disconnected</span></h1>
<div style="display:flex;gap:8px;margin-bottom:10px">
  <span class="pill" id="pill-uwb">UWB</span>
</div>
<div class="row">
  <div class="card">
    <h2>Navigation</h2>
    <div class="stat"><span>Distance</span><span id="dist" class="val">--</span></div>
    <div class="stat"><span>Angle</span><span id="angle" class="val">--</span></div>
    <div class="stat"><span>State</span><span id="navState" class="val">--</span></div>
    <div class="stat"><span>Odometry</span><span id="odometry" class="val">--</span></div>
  </div>
  <div class="card">
    <h2>Motion</h2>
    <div class="stat"><span>Hall Speed</span><span id="speed" class="val">--</span></div>
    <div class="stat"><span>Fused Speed</span><span id="fusedSpeed" class="val">--</span></div>
    <div class="stat"><span>Hall Raw</span><span id="hallRaw" class="val">--</span></div>
    <div class="stat"><span>Throttle</span><span id="throttle" class="val">--</span></div>
    <div class="stat"><span>Steering</span><span id="steering" class="val">--</span></div>
    <div class="stat"><span>Cogging</span><span id="cogging" class="val">--</span></div>
    <div class="stat"><span>Sign Changes</span><span id="signChanges" class="val">--</span></div>
  </div>
  <div class="card">
    <h2>IMU</h2>
    <div class="stat"><span>Heading</span><span id="heading" class="val">--</span></div>
    <div class="stat"><span>Cal Rot</span><span id="cal_rot" class="val">--</span></div>
    <div class="stat"><span>Cal Acc</span><span id="cal_acc" class="val">--</span></div>
    <div class="stat"><span>LPS</span><span id="lps" class="val">--</span></div>
  </div>
</div>
<div class="row">
  <div class="card" style="max-width:220px">
    <h2>Heading</h2>
    <canvas id="arrow" width="180" height="180"></canvas>
  </div>
  <div class="card" style="max-width:220px">
    <h2>Position</h2>
    <canvas id="posView"></canvas>
  </div>
</div>
<div class="row">
  <div class="card" style="flex:0 1 340px">
    <h2>Drive Mode</h2>
    <div style="display:flex;gap:8px">
      <button class="btn" id="btn-SETPOINT"       onclick="setMode('SETPOINT')">Setpoint</button>
      <button class="btn" id="btn-DIRECT"       onclick="setMode('DIRECT')">Direct</button>
      <button class="btn" id="btn-STOPPED"      onclick="setMode('STOPPED')">Stopped</button>
    </div>
    <div class="srow" style="margin-top:10px">
      <label><span>Target Speed (Setpoint)</span><span id="v_rSp">0.0 mph</span></label>
      <input type="range" id="setpointSpeedMph" min="0" max="4.0" step="0.1" value="0" disabled>
    </div>
    <div class="srow">
      <label><span>Throttle</span><span id="v_dThr">0%</span></label>
      <input type="range" id="directThrottlePct" min="-100" max="100" step="1" value="0" disabled>
    </div>
    <div class="srow">
      <label><span>Steering Servo</span><span id="v_dSteer">0%</span></label>
      <input type="range" id="directSteerPct" min="-100" max="100" step="1" value="0" disabled>
    </div>
    <div class="srow" style="margin-bottom:0">
      <label><span>Pan Servo</span><span id="v_dPan">0%</span></label>
      <input type="range" id="directPanPct" min="-100" max="100" step="1" value="0" disabled>
    </div>
  </div>
</div>
<div class="row">
  <div class="card">
    <h2 style="display:flex;justify-content:space-between;align-items:center">Speed (mph) + Throttle — last 10s
      <button class="btn" id="btnGraphPause" onclick="toggleGraphPause()">Pause</button></h2>
    <canvas id="speedGraph" height="80"></canvas>
  </div>
</div>
<div class="row cfg-row">
  <div class="card">
    <h2>Throttle</h2>
    <div class="srow">
      <label><span>Min Speed — speed at follow distance</span><span id="v_mnSp">--</span></label>
      <input type="range" id="minSpeedMph" min="0.3" max="4.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>Max Speed — speed at max distance</span><span id="v_mxSp">--</span></label>
      <input type="range" id="maxSpeedMph" min="0.3" max="4.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>Follow Distance — stop below this</span><span id="v_fd">--</span></label>
      <input type="range" id="followDistanceCm" min="50" max="400" step="10">
    </div>
    <div class="srow">
      <label><span>Max Distance — full speed beyond this</span><span id="v_md">--</span></label>
      <input type="range" id="maxDistanceCm" min="100" max="800" step="10">
    </div>
    <div class="srow">
      <label><span>Throttle Scale — max PWM output cap</span><span id="v_ts">--</span></label>
      <input type="range" id="throttleScale" min="0.045" max="0.36" step="0.005">
    </div>
    <div class="srow">
      <label><span>KP — speed PID proportional gain</span><span id="v_kp">--</span></label>
      <input type="range" id="kp" min="0.0" max="8.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>KI — speed PID integral gain</span><span id="v_ki">--</span></label>
      <input type="range" id="ki" min="0.0" max="1.0" step="0.025">
    </div>
    <div class="srow">
      <label><span>KD — speed PID derivative gain</span><span id="v_kd">--</span></label>
      <input type="range" id="kd" min="0.0" max="1.0" step="0.01">
    </div>
  </div>
  <div class="card">
    <h2>Steering</h2>
    <div class="srow">
      <label><span>Trim — mechanical bias correction (+ = right)</span><span id="v_sTr">--</span></label>
      <input type="range" id="steeringTrim" min="-0.3" max="0.3" step="0.01">
    </div>
    <div class="srow">
      <label><span>KP — angle PID proportional gain</span><span id="v_sKp">--</span></label>
      <input type="range" id="steeringKp" min="0.0025" max="0.02" step="0.0005">
    </div>
    <div class="srow">
      <label><span>KI — angle PID integral gain</span><span id="v_sKi">--</span></label>
      <input type="range" id="steeringKi" min="0.001" max="0.008" step="0.0002">
    </div>
    <div class="srow">
      <label><span>Max — servo deflection cap (0–1)</span><span id="v_sMax">--</span></label>
      <input type="range" id="steeringMax" min="0.16" max="1.0" step="0.01">
    </div>
  </div>
</div>
<div class="row cfg-row">
  <div class="card">
    <h2>UWB Filtering</h2>
    <div class="srow">
      <label><span>Outlier Reject — discard jumps larger than this</span><span id="v_uOr">--</span></label>
      <input type="range" id="uwbOutlierRejectCm" min="8.0" max="60.0" step="1.0">
    </div>
  </div>
  <div class="card">
    <h2>Fused Speed KF</h2>
    <div class="srow">
      <label><span>Enc R — encoder noise below the ramp</span><span id="v_fER">--</span></label>
      <input type="range" id="fusedEncR" min="-5" max="0" step="0.05">
    </div>
    <div class="srow">
      <label><span>Hall R — hall noise, higher = trust encoder more</span><span id="v_fHR">--</span></label>
      <input type="range" id="fusedHallR" min="-4" max="1" step="0.05">
    </div>
    <div class="srow">
      <label><span>Ramp Start — encoder fade begins</span><span id="v_fRs">--</span></label>
      <input type="range" id="fusedRampStartMph" min="0.5" max="4.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>Ramp End — encoder cut off above this</span><span id="v_fRe">--</span></label>
      <input type="range" id="fusedRampEndMph" min="1.0" max="5.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>2D Q Speed — process noise beyond what accel explains</span><span id="v_f2Qs">--</span></label>
      <input type="range" id="fused2QSpeed" min="-6" max="-1" step="0.05">
    </div>
    <div class="srow">
      <label><span>2D Q Bias — accel-bias drift rate</span><span id="v_f2Qb">--</span></label>
      <input type="range" id="fused2QBias" min="-8" max="-3" step="0.05">
    </div>
  </div>
</div>
<div class="row">
  <div class="card">
    <h2 style="display:flex;justify-content:space-between;align-items:center">Loop Timing (µs)<span>
      <button class="btn active" id="btnPerfAvg" onclick="setPerfMode('avg')">Avg</button>
      <button class="btn"        id="btnPerfMax" onclick="setPerfMode('max')">Max</button>
    </span></h2>
    <canvas id="perfGraph" height="70"></canvas>
  </div>
</div>
<script>
const ws = new WebSocket('ws://' + location.host + '/ws');
let _staleTimer = null;
function _connStatus(text, color) { const s = document.getElementById('connStatus'); s.textContent = text; s.style.color = color; s.style.borderColor = color; }
ws.onopen  = () => { document.getElementById('dot').className = 'on';  _connStatus('Connected', '#4f4'); };
ws.onclose = () => { document.getElementById('dot').className = '';    clearTimeout(_staleTimer); _connStatus('Disconnected', '#555'); };
ws.onmessage = e => {
  clearTimeout(_staleTimer);
  _staleTimer = setTimeout(() => _connStatus('No Data', '#fa4'), 2500);
  try { render(JSON.parse(e.data)); } catch(err) { console.warn('bad frame', err); }
};

function set(id, v) { const el = document.getElementById(id); if (el) el.textContent = v; }
function cls(id, c) { const el = document.getElementById(id); if (el) el.className = c; }
function setPill(id, q) { const el = document.getElementById(id); if (el) el.className = 'pill ' + ['good','deg','lost'][q]; }

const HISTORY = 500;  // 10s window at the 50 Hz control tick rate (samples arrive batched per WS push)
const speedBuf    = new Array(HISTORY).fill(0);
const encBuf      = new Array(HISTORY).fill(0);
const fused2Buf   = new Array(HISTORY).fill(0);
const targetBuf   = new Array(HISTORY).fill(0);
const throttleBuf = new Array(HISTORY).fill(0);
let _maxSpeedMph = 0;
const seriesVisible = { speed: true, enc: true, fused2: true, target: true, throttle: true };
let _graphPaused = false;
function toggleGraphPause() {
  _graphPaused = !_graphPaused;
  const b = document.getElementById('btnGraphPause');
  b.textContent = _graphPaused ? 'Resume' : 'Pause';
  b.className = 'btn' + (_graphPaused ? ' active' : '');
}
const legendBounds  = [];

function render(d) {
  const valid = d.uwbQ < 2;  // UWB fix age drives staleness styling of the UWB-fed readouts
  ['SETPOINT','DIRECT','STOPPED'].forEach(m => {
    const b = document.getElementById('btn-' + m);
    if (b) b.className = 'btn' + (d.navState === m ? ' active' : '');
  });
  directEnable(d.navState === 'DIRECT');
  setpointEnable(d.navState === 'SETPOINT');
  set('dist',     d.dist.toFixed(0) + ' cm');
  set('angle',    d.angle.toFixed(1) + '°');
  set('navState', d.navState);
  cls('dist',     valid ? 'val' : 'stale');
  cls('angle',    valid ? 'val' : 'stale');
  cls('navState', 'val');
  set('odometry', (d.odometry / 100).toFixed(1) + ' m');
  set('speed',    d.speed.toFixed(2) + ' mph');
  set('fusedSpeed', d.fSp.toFixed(2) + ' mph');
  set('hallRaw',  d.hRaw.toFixed(2) + ' mph');
  set('throttle', (d.throttle * 100).toFixed(0) + '%');
  set('steering', d.steering.toFixed(2));
  set('cogging',     d.cogging ? 'YES' : 'no');
  set('signChanges', d.signChanges);
  set('heading',  d.heading.toFixed(1) + '°');
  set('cal_rot',  d.cal_rot + '/3');
  set('cal_acc',  d.cal_acc + '/3');
  set('lps',      d.lps.toFixed(0));
  if (d.cfg) {
    slide('minSpeedMph',         d.cfg.mnSp,'v_mnSp', v => v.toFixed(1) + ' mph');
    slide('maxSpeedMph',         d.cfg.mxSp,'v_mxSp', v => v.toFixed(1) + ' mph');
    slide('followDistanceCm',    d.cfg.fd,  'v_fd',   v => v.toFixed(0) + ' cm');
    slide('maxDistanceCm',       d.cfg.md,  'v_md',   v => v.toFixed(0) + ' cm');
    slide('throttleScale',       d.cfg.ts,  'v_ts',   v => v.toFixed(3));
    slide('kp',                  d.cfg.kp,  'v_kp',   v => v.toFixed(2));
    slide('ki',                  d.cfg.ki,  'v_ki',   v => v.toFixed(3));
    slide('kd',                  d.cfg.kd,  'v_kd',   v => v.toFixed(3));
    slide('steeringTrim',        d.cfg.sTr, 'v_sTr',  v => (v >= 0 ? '+' : '') + v.toFixed(2));
    slide('steeringKp',          d.cfg.sKp, 'v_sKp',  v => v.toFixed(4));
    slide('steeringKi',          d.cfg.sKi, 'v_sKi',  v => v.toFixed(4));
    slide('steeringMax',         d.cfg.sMax,'v_sMax',  v => v.toFixed(2));
    slide('uwbOutlierRejectCm',  d.cfg.uOr, 'v_uOr',  v => v.toFixed(0) + ' cm');
    slide('fusedEncR',           d.cfg.fER, 'v_fER',  v => v.toExponential(2));
    slide('fusedHallR',          d.cfg.fHR, 'v_fHR',  v => v.toExponential(2));
    slide('fusedRampStartMph',   d.cfg.fRs, 'v_fRs',  v => v.toFixed(1) + ' mph');
    slide('fusedRampEndMph',     d.cfg.fRe, 'v_fRe',  v => v.toFixed(1) + ' mph');
    slide('fused2QSpeed',        d.cfg.f2Qs,'v_f2Qs', v => v.toExponential(2));
    slide('fused2QBias',         d.cfg.f2Qb,'v_f2Qb', v => v.toExponential(2));
    _maxSpeedMph = d.cfg.mxSp;
  }
  // Unpack the batched 50Hz control-tick arrays — one entry per PID tick since the
  // last push, so the graph sees every tick rather than 10Hz snapshots. Paused =
  // freeze the window: samples arriving while paused are dropped, not queued.
  if (!_graphPaused && d.g && d.g.sp) {
    d.g.sp.forEach((v, i) => {
      speedBuf.push(v);            speedBuf.shift();
      encBuf.push(d.g.en[i]);      encBuf.shift();
      fused2Buf.push(d.g.f2[i]);   fused2Buf.shift();
      targetBuf.push(d.g.tsp[i]);  targetBuf.shift();
      throttleBuf.push(d.g.th[i]); throttleBuf.shift();
    });
  }
  if (!_graphPaused) drawSpeedGraph();
  // Target arrow: heading error against control's held SETPOINT target (same value the
  // steering PID chases; wrap matches the firmware's ±180 convention).
  let err = d.heading - d.setpointHeading;
  while (err >  180) err -= 360;
  while (err < -180) err += 360;
  drawArrow(err, -d.steering * 90, valid, d.navState === 'SETPOINT', 'Target');
  drawPosView(d.dist, d.angle, d.cfg ? d.cfg.fd : 170, d.uwbQ);
  if (d.perf) { _perfData = d.perf; drawPerfGraph(); }
  setPill('pill-uwb', d.uwbQ);
}

function slide(id, val, labelId, fmt) {
  const el = document.getElementById(id);
  if (document.activeElement !== el) el.value = logSliders[id] ? Math.log10(val) : val;
  set(labelId, fmt(val));
}

function setMode(m) {
  fetch('/mode?mode=' + m, {method:'POST'});
}

// DIRECT effort sliders (negative = brake/reverse + left), one per actuator, fully
// isolated — each adjusts its axis alone. While any value is nonzero the browser
// re-sends the triple every 150ms so the firmware's 300ms DIRECT cmd-timeout
// failsafe stays fed — losing the page cuts throttle like a Pi comms loss.
const throttleSlider = document.getElementById('directThrottlePct');
const steerSlider    = document.getElementById('directSteerPct');
const panSlider      = document.getElementById('directPanPct');
const directSliders  = [throttleSlider, steerSlider, panSlider];
let _directTimer = null, _lastDirectSend = 0;
function sendDirect() {
  _lastDirectSend = Date.now();
  const msg = throttleSlider.value + ',' + steerSlider.value + ',' + panSlider.value;
  // The open WebSocket is the primary path — per-request HTTP jitter can starve
  // the 300ms cmd timeout and cause throttle dropouts; POST is the fallback.
  if (ws.readyState === WebSocket.OPEN) ws.send('direct:' + msg);
  else fetch('/direct?t=' + throttleSlider.value + '&s=' + steerSlider.value +
             '&p=' + panSlider.value, {method:'POST'});
}
function directLabels() {
  set('v_dThr',   throttleSlider.value + '%');
  set('v_dSteer', steerSlider.value + '%');
  set('v_dPan',   panSlider.value + '%');
}
function directHeartbeat(on) {
  if (on && !_directTimer) _directTimer = setInterval(sendDirect, 150);
  if (!on && _directTimer) { clearInterval(_directTimer); _directTimer = null; }
}
// After any slider input: refresh labels, send (rate-limited while dragging; an
// all-zero state always sends immediately so the actuators neutralize without
// waiting for the timeout), and keep the heartbeat matched to activity.
function directTouched() {
  directLabels();
  const active = directSliders.some(el => +el.value !== 0);
  if (!active || Date.now() - _lastDirectSend > 100) sendDirect();
  directHeartbeat(active);
}
// Enables/disables the sliders with the mode; leaving DIRECT snaps all back to zero.
function directEnable(on) {
  directSliders.forEach(el => el.disabled = !on);
  if (!on && (_directTimer || directSliders.some(el => +el.value !== 0))) {
    directHeartbeat(false);
    directSliders.forEach(el => el.value = 0);
    directLabels();
  }
}
directSliders.forEach(el => {
  el.addEventListener('input', directTouched);
  el.addEventListener('change', sendDirect);
});

// SETPOINT speed slider: bench setpoint for the throttle PID (firmware re-sends the
// held heading with it, so course is unaffected). Same heartbeat/failsafe contract
// as the DIRECT sliders: re-sent every 150ms while nonzero, cut by the firmware's
// 300ms cmd timeout when the page goes away.
const speedSlider = document.getElementById('setpointSpeedMph');
let _setpointTimer = null, _lastSetpointSend = 0;
function sendSetpoint() {
  _lastSetpointSend = Date.now();
  if (ws.readyState === WebSocket.OPEN) ws.send('setpoint:' + speedSlider.value);
  else fetch('/setpoint?mph=' + speedSlider.value, {method:'POST'});
}
function setpointHeartbeat(on) {
  if (on && !_setpointTimer) _setpointTimer = setInterval(sendSetpoint, 150);
  if (!on && _setpointTimer) { clearInterval(_setpointTimer); _setpointTimer = null; }
}
// Enables/disables the slider with the mode; leaving SETPOINT snaps it back to zero.
function setpointEnable(on) {
  speedSlider.disabled = !on;
  if (!on && (_setpointTimer || +speedSlider.value !== 0)) {
    setpointHeartbeat(false);
    speedSlider.value = 0;
    set('v_rSp', '0.0 mph');
  }
}
speedSlider.addEventListener('input', () => {
  const v = +speedSlider.value;
  set('v_rSp', v.toFixed(1) + ' mph');
  if (v === 0 || Date.now() - _lastSetpointSend > 100) sendSetpoint();
  setpointHeartbeat(v !== 0);
});
speedSlider.addEventListener('change', sendSetpoint);

// Log-scale sliders: the range element carries log10(value) so one drag spans orders
// of magnitude with uniform relative resolution; the actual value is what's sent to
// /config and shown in the label. slide() applies the inverse when syncing from telemetry.
const logSliders = { fusedEncR: true, fusedHallR: true, fused2QSpeed: true, fused2QBias: true };

['throttleScale','minSpeedMph','maxSpeedMph','followDistanceCm','maxDistanceCm','kp','ki','kd',
 'steeringTrim','steeringKp','steeringKi','steeringMax',
 'uwbOutlierRejectCm',
 'fusedEncR','fusedHallR','fusedRampStartMph','fusedRampEndMph',
 'fused2QSpeed','fused2QBias'].forEach(id => {
  document.getElementById(id).addEventListener('change', () => {
    const raw = document.getElementById(id).value;
    const val = logSliders[id] ? Math.pow(10, +raw) : raw;
    fetch('/config?key=' + id + '&value=' + val, {method:'POST'});
  });
});

const graphCanvas = document.getElementById('speedGraph');
graphCanvas.addEventListener('click', e => {
  const rect = graphCanvas.getBoundingClientRect();
  const sx = graphCanvas.width / rect.width;
  const sy = graphCanvas.height / rect.height;
  const mx = (e.clientX - rect.left) * sx;
  const my = (e.clientY - rect.top)  * sy;
  legendBounds.forEach(b => {
    if (mx >= b.x && mx <= b.x + b.w && my >= b.y && my <= b.y + b.h) {
      seriesVisible[b.key] = !seriesVisible[b.key];
      drawSpeedGraph();
    }
  });
});
const gctx = graphCanvas.getContext('2d');
function drawSpeedGraph() {
  const w = graphCanvas.offsetWidth || 400;
  const h = graphCanvas.offsetHeight || 80;
  graphCanvas.width = w; graphCanvas.height = h;
  const pad = 4;
  gctx.clearRect(0, 0, w, h);
  const maxVal = Math.max(5, _maxSpeedMph, ...speedBuf, ...encBuf);
  gctx.strokeStyle = '#333';
  gctx.lineWidth = 1;
  gctx.beginPath(); gctx.moveTo(pad, pad); gctx.lineTo(pad, h-pad); gctx.lineTo(w-pad, h-pad); gctx.stroke();
  gctx.fillStyle = '#555';
  gctx.font = '11px monospace';
  gctx.fillText(maxVal.toFixed(1), pad+2, pad+11);
  gctx.fillText('0', pad+2, h-pad-2);
  const sl = pad + 14, sw = w - sl - pad;
  // Faint 0.5 mph gridlines with matching y-axis ticks, recomputed from maxVal every
  // frame so they rescale with the graph. Integer-mph lines are brighter with longer
  // ticks, and get a value label in the gutter when there's room between them.
  const mphPx = (h - pad * 2) / maxVal;
  gctx.lineWidth = 1;
  for (let i = 1; i * 0.5 < maxVal; i++) {
    const v = i * 0.5, major = (i % 2 === 0);
    const y = h - pad - v * mphPx;
    gctx.strokeStyle = major ? '#2c2c2c' : '#232323';
    gctx.beginPath(); gctx.moveTo(sl, y); gctx.lineTo(w - pad, y); gctx.stroke();
    gctx.strokeStyle = '#555';
    gctx.beginPath(); gctx.moveTo(pad, y); gctx.lineTo(pad + (major ? 5 : 3), y); gctx.stroke();
    if (major && mphPx >= 14) {
      gctx.fillStyle = '#555';
      gctx.font = '9px monospace';
      gctx.fillText(v.toFixed(0), pad + 2, y - 2);
    }
  }
  // Quarter-second ticks (10s window → 40 intervals); every 4th tick marks a full second
  gctx.lineWidth = 1;
  for (let i = 0; i <= 40; i++) {
    const x = sl + (i / 40) * sw;
    const major = (i % 4 === 0);
    gctx.strokeStyle = major ? '#555' : '#2d2d2d';
    gctx.beginPath(); gctx.moveTo(x, h-pad); gctx.lineTo(x, h-pad-(major ? 6 : 3)); gctx.stroke();
  }
  // Target speed line — drawn as a polyline so it tracks the interpolated setpoint
  if (seriesVisible.target) {
    gctx.strokeStyle = '#666';
    gctx.lineWidth = 1;
    gctx.setLineDash([4, 4]);
    gctx.beginPath();
    targetBuf.forEach((v, i) => {
      const x = sl + (i / (HISTORY - 1)) * sw;
      const y = h - pad - (v / maxVal) * (h - pad * 2);
      i === 0 ? gctx.moveTo(x, y) : gctx.lineTo(x, y);
    });
    gctx.stroke();
    gctx.setLineDash([]);
  }
  // Legend — top centre (click to toggle)
  legendBounds.length = 0;
  gctx.font = '11px monospace';
  const items = [
    { key: 'speed',    color: '#4af', label: 'Hall'     },
    { key: 'enc',      color: '#a4f', label: 'Enc'      },
    { key: 'fused2',   color: '#4f4', label: 'Fused'    },
    { key: 'target',   color: '#666', label: 'Target'   },
    { key: 'throttle', color: '#f66', label: 'Throttle' },
  ];
  const totalW = items.reduce((acc, it) => acc + 12 + gctx.measureText(it.label).width + 10, 0) - 10;
  let lx = sl + (sw - totalW) / 2;
  const ly = pad + 11;
  items.forEach(({key, color, label}) => {
    const itemW = 12 + gctx.measureText(label).width;
    legendBounds.push({ x: lx, y: ly - 11, w: itemW + 8, h: 14, key });
    gctx.globalAlpha = seriesVisible[key] ? 1.0 : 0.3;
    gctx.fillStyle = color;
    gctx.fillRect(lx, ly - 9, 9, 9);
    gctx.fillStyle = '#ccc';
    gctx.fillText(label, lx + 12, ly);
    gctx.globalAlpha = 1.0;
    lx += itemW + 10;
  });
  // 10s error — centred below the legend, clear of the y-axis max label
  const avgSpeed  = speedBuf.reduce((a, b) => a + b, 0) / speedBuf.length;
  const avgTarget = targetBuf.reduce((a, b) => a + b, 0) / targetBuf.length;
  const avgError  = avgSpeed - avgTarget;
  gctx.fillStyle = '#fa4';
  gctx.font = 'bold 12px monospace';
  gctx.textAlign = 'center';
  gctx.fillText('10s err: ' + (avgError >= 0 ? '+' : '') + avgError.toFixed(2), sl + sw / 2, pad + 26);
  gctx.textAlign = 'left';
  // Right axis: throttle scale (0–1)
  gctx.fillStyle = '#f66';
  gctx.font = '11px monospace';
  gctx.textAlign = 'right';
  gctx.fillText('100%', w - pad, pad + 11);
  gctx.fillText('0%',   w - pad, h - pad - 2);
  gctx.textAlign = 'left';
  const step = sw / (HISTORY - 1);
  // Throttle line (red, 0–1 mapped to full graph height)
  if (seriesVisible.throttle) {
    gctx.beginPath();
    throttleBuf.forEach((v, i) => {
      const x = sl + i * step;
      const y = h - pad - v * (h - pad * 2);
      i === 0 ? gctx.moveTo(x, y) : gctx.lineTo(x, y);
    });
    gctx.strokeStyle = '#f66';
    gctx.lineWidth = 1;
    gctx.stroke();
  }
  // Fused speed line (green) — the 2-state [speed, accelBias] filter, the PID's feedback
  if (seriesVisible.fused2) {
    gctx.beginPath();
    fused2Buf.forEach((v, i) => {
      const x = sl + i * step;
      const y = h - pad - (v / maxVal) * (h - pad * 2);
      i === 0 ? gctx.moveTo(x, y) : gctx.lineTo(x, y);
    });
    gctx.strokeStyle = '#4f4';
    gctx.lineWidth = 1;
    gctx.stroke();
  }
  // Encoder speed line (purple) — signed, so backward jitter dips below the axis and clips
  if (seriesVisible.enc) {
    gctx.beginPath();
    encBuf.forEach((v, i) => {
      const x = sl + i * step;
      const y = h - pad - (v / maxVal) * (h - pad * 2);
      i === 0 ? gctx.moveTo(x, y) : gctx.lineTo(x, y);
    });
    gctx.strokeStyle = '#a4f';
    gctx.lineWidth = 1;
    gctx.stroke();
  }
  // Speed line
  if (seriesVisible.speed) {
    gctx.beginPath();
    speedBuf.forEach((v, i) => {
      const x = sl + i * step;
      const y = h - pad - (v / maxVal) * (h - pad*2);
      i === 0 ? gctx.moveTo(x, y) : gctx.lineTo(x, y);
    });
    gctx.strokeStyle = '#4af';
    gctx.lineWidth = 1.5;
    gctx.stroke();
    gctx.lineTo(sl + (HISTORY-1)*step, h-pad);
    gctx.lineTo(sl, h-pad);
    gctx.closePath();
    gctx.fillStyle = 'rgba(68,170,255,0.15)';
    gctx.fill();
  }
}

const canvas = document.getElementById('arrow');
const ctx = canvas.getContext('2d');
function drawArrow(targetDeg, steeringDeg, valid, targetActive, thirdLabel) {
  const w = canvas.width, h = canvas.height;
  const cx = w/2, cy = h/2, r = Math.min(w,h) * 0.38;
  ctx.clearRect(0, 0, w, h);
  // Circle
  ctx.strokeStyle = valid ? '#4af' : '#333';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2); ctx.stroke();
  // Steering arrow (blue, shorter)
  const sRad = (steeringDeg - 90) * Math.PI / 180;
  const sTip = {x: cx + r*0.68*Math.cos(sRad), y: cy + r*0.68*Math.sin(sRad)};
  ctx.strokeStyle = '#4af';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(sTip.x, sTip.y); ctx.stroke();
  const sa = sRad + Math.PI;
  ctx.fillStyle = '#4af';
  ctx.beginPath();
  ctx.moveTo(sTip.x, sTip.y);
  ctx.lineTo(sTip.x + 6*Math.cos(sa+0.45), sTip.y + 6*Math.sin(sa+0.45));
  ctx.lineTo(sTip.x + 9*Math.cos(sa),      sTip.y + 9*Math.sin(sa));
  ctx.lineTo(sTip.x + 6*Math.cos(sa-0.45), sTip.y + 6*Math.sin(sa-0.45));
  ctx.closePath(); ctx.fill();
  // Target arrow: green when tracking tag (NORMAL), amber when holding heading, hidden when stopped
  const targetColor = valid ? '#4f4' : (targetActive ? '#fa4' : '#333');
  const rad = (targetDeg - 90) * Math.PI / 180;
  const tip = {x: cx + r*0.85*Math.cos(rad), y: cy + r*0.85*Math.sin(rad)};
  ctx.strokeStyle = targetColor;
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tip.x, tip.y); ctx.stroke();
  const a = rad + Math.PI;
  ctx.fillStyle = targetColor;
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x + 8*Math.cos(a+0.45), tip.y + 8*Math.sin(a+0.45));
  ctx.lineTo(tip.x + 12*Math.cos(a),     tip.y + 12*Math.sin(a));
  ctx.lineTo(tip.x + 8*Math.cos(a-0.45), tip.y + 8*Math.sin(a-0.45));
  ctx.closePath(); ctx.fill();
  // Legend
  ctx.font = '10px monospace';
  ctx.textAlign = 'center';
  ctx.fillStyle = targetColor; ctx.fillText('▲ ' + thirdLabel, cx - 28, h - 4);
  ctx.fillStyle = '#4af';      ctx.fillText('▲ Steer',          cx + 28, h - 4);
  ctx.textAlign = 'left';
}

const posCanvas = document.getElementById('posView');
const posCtx    = posCanvas.getContext('2d');
function drawPosView(dist, angleDeg, followDistCm, uwbQ) {
  const w = posCanvas.offsetWidth || 200;
  posCanvas.width = w; posCanvas.height = w;
  const cx = w / 2, cy = w / 2;
  const LEGEND_H = 18;
  posCtx.clearRect(0, 0, w, w);

  // Auto-scale: fit current dist or 1.5× follow distance, whichever is larger
  const viewR = Math.max(dist > 0 ? dist * 1.3 : followDistCm * 2, followDistCm * 1.5, 150);
  const scale = (w * 0.42) / viewR;

  // Nice ring step: aim for ~4 rings
  const mag  = Math.pow(10, Math.floor(Math.log10(viewR / 4)));
  const norm = (viewR / 4) / mag;
  const step = (norm < 1.5 ? 1 : norm < 3.5 ? 2 : 5) * mag;

  // Range rings + labels
  posCtx.lineWidth = 1;
  for (let r = step; r < viewR * 1.1; r += step) {
    posCtx.strokeStyle = '#252525';
    posCtx.beginPath(); posCtx.arc(cx, cy, r * scale, 0, Math.PI * 2); posCtx.stroke();
    posCtx.fillStyle = '#333'; posCtx.font = '9px monospace'; posCtx.textAlign = 'right';
    posCtx.fillText((r / 100).toFixed(r < 100 ? 1 : 0) + 'm', cx - 3, cy - r * scale + 9);
  }

  // Stop-distance ring (dashed — car holds position inside this)
  posCtx.strokeStyle = '#1c3020'; posCtx.lineWidth = 1; posCtx.setLineDash([4, 4]);
  posCtx.beginPath(); posCtx.arc(cx, cy, followDistCm * scale, 0, Math.PI * 2); posCtx.stroke();
  posCtx.setLineDash([]);

  // Forward axis
  posCtx.strokeStyle = '#1e281e'; posCtx.lineWidth = 1;
  posCtx.beginPath(); posCtx.moveTo(cx, cy - w*0.46); posCtx.lineTo(cx, cy + w*0.46); posCtx.stroke();

  // "FWD" label
  posCtx.fillStyle = '#333'; posCtx.font = '9px monospace'; posCtx.textAlign = 'center';
  posCtx.fillText('FWD', cx, 10);

  // Car icon — small triangle pointing up
  posCtx.fillStyle = '#4af';
  posCtx.beginPath(); posCtx.moveTo(cx, cy-9); posCtx.lineTo(cx-5, cy+5); posCtx.lineTo(cx+5, cy+5); posCtx.closePath(); posCtx.fill();

  function plotDot(angleDeg, dotDist, radius, color, alpha) {
    const rad = angleDeg * Math.PI / 180;
    const tx  = cx + dotDist * scale * Math.sin(rad);
    const ty  = cy - dotDist * scale * Math.cos(rad);
    posCtx.strokeStyle = color; posCtx.globalAlpha = 0.25; posCtx.lineWidth = 1;
    posCtx.beginPath(); posCtx.moveTo(cx, cy); posCtx.lineTo(tx, ty); posCtx.stroke();
    posCtx.globalAlpha = alpha;
    posCtx.fillStyle = color;
    posCtx.beginPath(); posCtx.arc(tx, ty, radius, 0, Math.PI * 2); posCtx.fill();
    posCtx.globalAlpha = 1.0;
    return {tx, ty};
  }

  // Dot color tracks UWB fix freshness (green/amber/red by uwbQ).
  const dotCol = uwbQ === 0 ? '#4f4' : uwbQ === 1 ? '#fa4' : '#f44';

  if (dist > 0) {
    plotDot(angleDeg, dist, 6, dotCol, 0.85);

    // Distance + angle readout (top-left)
    posCtx.fillStyle = dotCol; posCtx.font = 'bold 10px monospace'; posCtx.textAlign = 'left';
    posCtx.fillText(dist.toFixed(0) + ' cm', 4, 12);
    posCtx.fillText((angleDeg >= 0 ? '+' : '') + angleDeg.toFixed(1) + '°', 4, 23);
  }

  // Legend — dot color tracks fix freshness
  posCtx.font = '10px monospace'; posCtx.textAlign = 'left';
  const ly = w - 4;
  posCtx.fillStyle = dotCol; posCtx.fillRect(4,   ly-9, 8, 8);
  posCtx.fillStyle = '#aaa'; posCtx.fillText('UWB', 15, ly);
  posCtx.textAlign = 'left';
}

let _perfData = null;
let _perfMode = 'avg';
function setPerfMode(m) {
  _perfMode = m;
  document.getElementById('btnPerfAvg').className = 'btn' + (m === 'avg' ? ' active' : '');
  document.getElementById('btnPerfMax').className = 'btn' + (m === 'max' ? ' active' : '');
  drawPerfGraph();
}
const perfCanvas = document.getElementById('perfGraph');
const pctx = perfCanvas.getContext('2d');
const PERF_LABELS  = ['IMU','UWB','Ctrl','OLED','WiFi'];
const PERF_COLORS  = ['#4f4','#4af','#f66','#a4f','#4fa'];
function drawPerfGraph() {
  if (!_perfData) return;
  const w = perfCanvas.offsetWidth || 400;
  const h = perfCanvas.offsetHeight || 70;
  perfCanvas.width = w; perfCanvas.height = h;
  const pad = 4, labelH = 16, valH = 12;
  pctx.clearRect(0, 0, w, h);
  const vals = _perfMode === 'avg'
    ? [_perfData.ia, _perfData.ua, _perfData.ca, _perfData.oa, _perfData.wa]
    : [_perfData.im, _perfData.um, _perfData.cm, _perfData.om, _perfData.wm];
  const maxVal = Math.max(1, ...vals);
  const n = vals.length;
  const barW = (w - pad * 2) / n;
  const graphH = h - pad - labelH - valH;
  vals.forEach((v, i) => {
    const x   = pad + i * barW;
    const bh  = (v / maxVal) * graphH;
    const by  = pad + valH + (graphH - bh);
    pctx.fillStyle = PERF_COLORS[i];
    pctx.fillRect(x + 2, by, barW - 4, bh);
    // Value — fixed in reserved strip above bars, always readable
    pctx.font = '10px monospace';
    pctx.textAlign = 'center';
    pctx.fillStyle = PERF_COLORS[i];
    pctx.fillText(v + 'µs', x + barW / 2, pad + valH - 2);
    // Label below bar
    pctx.fillStyle = '#555';
    pctx.fillText(PERF_LABELS[i], x + barW / 2, h - pad - 2);
  });
  pctx.textAlign = 'left';
}
</script>
</body>
</html>
)HTML";

// Maps the DIRECT bench sliders' per-axis percentages [-100, 100] onto the three
// actuators (negative = throttle brake/reverse, steering/pan left) and injects them
// as a synthetic DIRECT frame, so the serial cmd-timeout failsafe governs them — the
// page must keep re-sending while any effort is nonzero.
static void direct_effort(float throttlePct, float steeringPct, float panPct) {
    if (!isfinite(throttlePct) || !isfinite(steeringPct) || !isfinite(panPct)) return;
    serial_hal_inject_direct(constrain(throttlePct, -100.0f, 100.0f) / 100.0f,
                             constrain(steeringPct, -100.0f, 100.0f) / 100.0f,
                             constrain(panPct,      -100.0f, 100.0f) / 100.0f * PAN_MAX_DEG);
}

// SETPOINT bench slider: injects a target-speed setpoint while re-sending the currently
// held target heading, so the speed PID is exercised without changing course. Falls
// back to live yaw before the first heading is seeded; drops the frame if neither is
// valid (a setpoint frame can't carry speed alone).
static void setpoint_speed(float mph) {
    if (!isfinite(mph)) return;
    float heading = control_setpoint_heading_deg();
    if (isnan(heading)) heading = imu_get().yaw;
    if (isnan(heading)) return;
    serial_hal_inject_setpoint(mph, heading);
}

void dashboard_init() {
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", DASHBOARD_HTML);
    });

    _server.on("/config", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("key") && req->hasParam("value")) {
            String key = req->getParam("key")->value();
            float  val = req->getParam("value")->value().toFloat();
            if      (key == "throttleScale")          rtConfig.throttleScale          = val;
            else if (key == "smoothAlpha")            rtConfig.smoothAlpha            = val;
            else if (key == "followDistanceCm")       rtConfig.followDistanceCm       = val;
            else if (key == "maxDistanceCm")          rtConfig.maxDistanceCm          = val;
            else if (key == "minSpeedMph")            rtConfig.minSpeedMph            = val;
            else if (key == "maxSpeedMph")            rtConfig.maxSpeedMph            = val;
            else if (key == "kp")                     rtConfig.kp                     = val;
            else if (key == "ki")                     rtConfig.ki                     = val;
            else if (key == "kd")                     rtConfig.kd                     = val;
            else if (key == "steeringTrim")           rtConfig.steeringTrim           = val;
            else if (key == "steeringKp")             rtConfig.steeringKp             = val;
            else if (key == "steeringKi")             rtConfig.steeringKi             = val;
            else if (key == "steeringMax")            rtConfig.steeringMax            = val;
            else if (key == "uwbOutlierRejectCm")     rtConfig.uwbOutlierRejectCm     = val;
            else if (key == "fusedEncR")              rtConfig.fusedEncR              = val;
            else if (key == "fusedHallR")             rtConfig.fusedHallR             = val;
            else if (key == "fusedRampStartMph")      rtConfig.fusedRampStartMph      = val;
            else if (key == "fusedRampEndMph")        rtConfig.fusedRampEndMph        = val;
            else if (key == "fused2QSpeed")           rtConfig.fused2QSpeed           = val;
            else if (key == "fused2QBias")            rtConfig.fused2QBias            = val;
            ESP_LOGI(TAG, "config %s = %.3f", key.c_str(), val);
        }
        req->send(200);
    });

    // DIRECT bench slider fallback path — the primary is the WebSocket heartbeat in
    // the onEvent handler below. Also handy for curl-driven bench scripts:
    // t = throttle %, s = steering %, p = pan % (each [-100, 100]).
    _server.on("/direct", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("t") && req->hasParam("s") && req->hasParam("p"))
            direct_effort(req->getParam("t")->value().toFloat(),
                          req->getParam("s")->value().toFloat(),
                          req->getParam("p")->value().toFloat());
        req->send(200);
    });

    // SETPOINT bench speed-setpoint fallback path (primary is the WebSocket
    // "setpoint:<mph>" heartbeat).
    _server.on("/setpoint", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("mph")) setpoint_speed(req->getParam("mph")->value().toFloat());
        req->send(200);
    });

    _server.on("/mode", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("mode")) {
            String m = req->getParam("mode")->value();
            if      (m == "SETPOINT")  control_set_mode(ControlMode::SETPOINT);
            else if (m == "DIRECT")  control_set_mode(ControlMode::DIRECT);
            else if (m == "STOPPED") control_set_mode(ControlMode::STOPPED);
            ESP_LOGI(TAG, "mode → %s", m.c_str());
        }
        req->send(200);
    });

    _webSocket.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
        if (type == WS_EVT_CONNECT)    ESP_LOGI(TAG, "WS client connected id=%u", client->id());
        if (type == WS_EVT_DISCONNECT) { ESP_LOGI(TAG, "WS client disconnected"); _webSocket.cleanupClients(); }
        if (type == WS_EVT_DATA) {
            // Slider heartbeat: "direct:<t>,<s>,<p>" (throttle/steering/pan %) as a
            // single-frame text message. The payload is a few bytes, so multi-frame
            // reassembly is deliberately unsupported — anything fragmented or
            // oversized is dropped, as is a triple that doesn't parse whole.
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len &&
                info->opcode == WS_TEXT && len < 32) {
                char msg[32];
                memcpy(msg, data, len);
                msg[len] = '\0';
                float t, s, p;
                if (sscanf(msg, "direct:%f,%f,%f", &t, &s, &p) == 3) direct_effort(t, s, p);
                else if (sscanf(msg, "setpoint:%f", &t) == 1)          setpoint_speed(t);
            }
        }
    });
    _server.addHandler(&_webSocket);
    _server.begin();
    Serial.printf("[%s] ✅ Dashboard at http://192.168.4.1/\n", TAG);
}

static float    safeF(float v) { return isnan(v) || isinf(v) ? 0.0f : v; }
static PerfData _perf = {};
static RateGate _gate{ DASHBOARD_UPDATE_INTERVAL_MS };
void dashboard_set_perf(const PerfData& p) { _perf = p; }

// 50 Hz control-tick samples batched into the 10 Hz WS push, so the speed graph gets
// full PID-rate fidelity without raising the push rate (which would cost canvas
// redraws and connection robustness). Written and drained on the loop task — no locking.
struct CtrlSample { float hallSpeedMph, encSpeedMph, fusedSpeedMph, targetSpeedMph, throttle; };
static const uint8_t CTRL_RING_CAP = 12;   // 2× the nominal 5-6 samples per 100ms push
static CtrlSample _ctrlRing[CTRL_RING_CAP];
static uint8_t    _ctrlRingCount = 0;

// Buffers one graph sample; called by main.cpp on each control PID tick. Drops the
// newest sample if a stalled push has filled the ring — the graph shows a brief gap
// rather than growing unbounded.
void dashboard_sample_ctrl() {
    if (_ctrlRingCount >= CTRL_RING_CAP) return;
    const ControlOutput& ctrl = control_get();
    const RPMData&       rpm  = rpm_get();
    _ctrlRing[_ctrlRingCount++] = { rpm.hallSpeedMph, rpm.encSpeedMph, rpm.fusedSpeedMph, ctrl.targetSpeedMph, ctrl.throttle };
}

void dashboard_update(float lps) {
    const ControlMode    mode  = control_mode();
    const UWBReading&    uwb   = uwb_get();
    const RPMData&       rpm   = rpm_get();
    const ImuData&       imu   = imu_get();
    const ControlOutput& ctrl  = control_get();

    uint32_t nowMs   = millis();
    uint32_t uwbAge  = nowMs - uwb.timestamp;
    int uwbQ = uwbAge < 500 ? 0 : uwbAge < 1500 ? 1 : 2;
    float dt;
    if (!_gate.tick(dt)) return;

    _webSocket.cleanupClients();

    // Drain the control-tick ring into "g" arrays (one entry per 20ms PID tick since
    // the last push). The scalar speed/throttle/tSp fields stay in the frame as the
    // instantaneous values for the stat readouts.
    char gBuf[896];
    int  gLen = snprintf(gBuf, sizeof(gBuf), "\"g\":{\"sp\":[");
    for (uint8_t i = 0; i < _ctrlRingCount; i++)
        gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "%s%.3f", i ? "," : "", safeF(_ctrlRing[i].hallSpeedMph));
    gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "],\"en\":[");
    for (uint8_t i = 0; i < _ctrlRingCount; i++)
        gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "%s%.3f", i ? "," : "", safeF(_ctrlRing[i].encSpeedMph));
    gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "],\"f2\":[");
    for (uint8_t i = 0; i < _ctrlRingCount; i++)
        gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "%s%.3f", i ? "," : "", safeF(_ctrlRing[i].fusedSpeedMph));
    gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "],\"tsp\":[");
    for (uint8_t i = 0; i < _ctrlRingCount; i++)
        gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "%s%.3f", i ? "," : "", safeF(_ctrlRing[i].targetSpeedMph));
    gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "],\"th\":[");
    for (uint8_t i = 0; i < _ctrlRingCount; i++)
        gLen += snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "%s%.3f", i ? "," : "", safeF(_ctrlRing[i].throttle));
    snprintf(gBuf + gLen, sizeof(gBuf) - gLen, "]}");
    _ctrlRingCount = 0;

    char buf[2048];
    snprintf(buf, sizeof(buf),
        "{\"dist\":%.1f,\"angle\":%.1f,\"setpointHeading\":%.1f,\"navState\":\"%s\",\"odometry\":%.0f,"
        "\"uwbQ\":%d,"
        "\"speed\":%.3f,\"fSp\":%.3f,\"hRaw\":%.3f,\"cogging\":%d,\"signChanges\":%d,"
        "\"heading\":%.1f,\"cal_rot\":%u,\"cal_acc\":%u,"
        "\"throttle\":%.3f,\"steering\":%.3f,\"tSp\":%.3f,\"lps\":%.0f,"
        "%s,"
        "\"cfg\":{\"ts\":%.3f,\"sa\":%.3f,\"fd\":%.0f,\"md\":%.0f,\"mnSp\":%.2f,\"mxSp\":%.2f,"
        "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,"
        "\"sTr\":%.3f,\"sKp\":%.4f,\"sKi\":%.4f,\"sMax\":%.3f,"
        "\"uOr\":%.1f,"
        "\"fER\":%.4g,\"fHR\":%.4g,\"fRs\":%.2f,\"fRe\":%.2f,"
        "\"f2Qs\":%.4g,\"f2Qb\":%.4g},"
        "\"perf\":{\"ia\":%u,\"im\":%u,\"ua\":%u,\"um\":%u,"
        "\"ca\":%u,\"cm\":%u,\"oa\":%u,\"om\":%u,\"wa\":%u,\"wm\":%u}}",
        safeF(uwb.distCm), safeF(uwb.angleDeg), safeF(control_setpoint_heading_deg()),
        mode == ControlMode::SETPOINT  ? "SETPOINT"  :
        mode == ControlMode::DIRECT  ? "DIRECT"  :
        mode == ControlMode::STOPPED ? "STOPPED" : "UNKNOWN",
        safeF(rpm.odometryCm),
        uwbQ,
        safeF(rpm.hallSpeedMph), safeF(rpm.fusedSpeedMph), safeF(rpm.hallRawMph), (int)rpm.cogging, rpm.signChanges,
        safeF(imu.yaw), imu.cal_rot, imu.cal_accel,
        safeF(ctrl.throttle), safeF(ctrl.steering), safeF(ctrl.targetSpeedMph), safeF(lps),
        gBuf,
        rtConfig.throttleScale, rtConfig.smoothAlpha,
        rtConfig.followDistanceCm, rtConfig.maxDistanceCm,
        rtConfig.minSpeedMph, rtConfig.maxSpeedMph,
        rtConfig.kp, rtConfig.ki, rtConfig.kd,
        rtConfig.steeringTrim, rtConfig.steeringKp, rtConfig.steeringKi, rtConfig.steeringMax,
        rtConfig.uwbOutlierRejectCm,
        rtConfig.fusedEncR, rtConfig.fusedHallR,
        rtConfig.fusedRampStartMph, rtConfig.fusedRampEndMph,
        rtConfig.fused2QSpeed, rtConfig.fused2QBias,
        _perf.imuAvg,  _perf.imuMax,
        _perf.uwbAvg,  _perf.uwbMax,
        _perf.ctrlAvg, _perf.ctrlMax,
        _perf.oledAvg, _perf.oledMax,
        _perf.wifiAvg, _perf.wifiMax);

    _webSocket.textAll(buf);
}

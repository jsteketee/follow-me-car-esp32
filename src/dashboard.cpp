// HTTP + WebSocket dashboard. Serves a single-page UI at port 80 and pushes
// telemetry JSON at ~10 Hz. Config overrides are applied via POST /config.
#include "dashboard.h"
#include "nav.h"
#include "fusion.h"
#include "rpm.h"
#include "imu.h"
#include "control.h"
#include "runtime_config.h"
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
@media(max-width:600px){.cfg-row .card{flex:1 1 100%}}
#dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;margin-right:7px;vertical-align:middle;transition:background 0.3s}
#dot.on{background:#4f4}
</style>
</head>
<body>
<h1><span><span id="dot"></span>Follow-Me Car</span><span id="connStatus">Disconnected</span></h1>
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
    <div class="stat"><span>Speed</span><span id="speed" class="val">--</span></div>

    <div class="stat"><span>RPM</span><span id="rpm" class="val">--</span></div>
    <div class="stat"><span>Throttle</span><span id="throttle" class="val">--</span></div>
    <div class="stat"><span>Steering</span><span id="steering" class="val">--</span></div>
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
</div>
<div class="row">
  <div class="card" style="flex:0 0 auto">
    <h2>Drive Mode</h2>
    <div style="display:flex;gap:8px">
      <button class="btn" id="btn-FOLLOW_ME" onclick="setMode('FOLLOW_ME')">Follow Me</button>
      <button class="btn" id="btn-TEST"      onclick="setMode('TEST')">Test</button>
      <button class="btn" id="btn-STOPPED"   onclick="setMode('STOPPED')">Stopped</button>
    </div>
  </div>
</div>
<div class="row">
  <div class="card">
    <h2>Speed (mph) + Throttle — last 10s</h2>
    <canvas id="speedGraph" height="80"></canvas>
  </div>
</div>
<div class="row cfg-row">
  <div class="card">
    <h2>Throttle</h2>
    <div class="srow">
      <label><span>Target Speed — speed setpoint while following</span><span id="v_sp">--</span></label>
      <input type="range" id="targetSpeedMph" min="0.5" max="4.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>Follow Distance — stop below this</span><span id="v_fd">--</span></label>
      <input type="range" id="followDistanceCm" min="50" max="400" step="10">
    </div>
    <div class="srow">
      <label><span>Throttle Scale — max PWM output cap</span><span id="v_ts">--</span></label>
      <input type="range" id="throttleScale" min="0.045" max="0.36" step="0.005">
    </div>
    <div class="srow">
      <label><span>KP — speed PID proportional gain</span><span id="v_kp">--</span></label>
      <input type="range" id="kp" min="1.0" max="8.0" step="0.1">
    </div>
    <div class="srow">
      <label><span>KI — speed PID integral gain</span><span id="v_ki">--</span></label>
      <input type="range" id="ki" min="0.12" max="1.0" step="0.025">
    </div>
  </div>
  <div class="card">
    <h2>Steering</h2>
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
      <label><span>Kalman Q — process noise, higher = faster tracking</span><span id="v_uQ">--</span></label>
      <input type="range" id="uwbKalmanQ" min="2.0" max="16.0" step="0.5">
    </div>
    <div class="srow">
      <label><span>Kalman R — sensor noise, lower = more trust</span><span id="v_uR">--</span></label>
      <input type="range" id="uwbKalmanR" min="19.0" max="150.0" step="2.5">
    </div>
    <div class="srow">
      <label><span>Outlier Reject — discard jumps larger than this</span><span id="v_uOr">--</span></label>
      <input type="range" id="uwbOutlierRejectCm" min="8.0" max="60.0" step="1.0">
    </div>
  </div>
  <div class="card">
    <h2>Fusion</h2>
    <div class="srow">
      <label><span>Bearing Q/sec — bearing drift rate between fixes</span><span id="v_fQ">--</span></label>
      <input type="range" id="fusionQBearingPerSec" min="6.0" max="50.0" step="1.0">
    </div>
    <div class="srow">
      <label><span>Bearing R (UWB) — lower = trust each fix more</span><span id="v_fR">--</span></label>
      <input type="range" id="fusionRUwb" min="25.0" max="200.0" step="5.0">
    </div>
    <div class="srow">
      <label><span>Stale Threshold — freeze steering/throttle above</span><span id="v_fSU">--</span></label>
      <input type="range" id="fusionStaleUncertainty" min="50.0" max="400.0" step="10.0">
    </div>
    <div class="srow">
      <label><span>Innov Alpha — interference spike sensitivity</span><span id="v_fEa">--</span></label>
      <input type="range" id="fusionInnovEwmaAlpha" min="0.025" max="0.2" step="0.005">
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

const HISTORY = 100;
const speedBuf    = new Array(HISTORY).fill(0);
const throttleBuf = new Array(HISTORY).fill(0);
let _targetSpeedMph = 0;
const seriesVisible = { speed: true, target: true, throttle: true };
const legendBounds  = [];

function render(d) {
  const valid = d.navState === 'FOLLOW_ME';
  ['FOLLOW_ME','TEST','STOPPED'].forEach(m => {
    const b = document.getElementById('btn-' + m);
    if (b) b.className = 'btn' + (d.navState === m ? ' active' : '');
  });
  set('dist',     d.dist.toFixed(0) + ' cm');
  set('angle',    d.angle.toFixed(1) + '°');
  set('navState', d.navState);
  cls('dist',     valid ? 'val' : 'stale');
  cls('angle',    valid ? 'val' : 'stale');
  cls('navState', valid ? 'val' : 'stale');
  set('odometry', (d.odometry / 100).toFixed(1) + ' m');
  set('speed',    d.speed.toFixed(2) + ' mph');
  set('rpm',      d.rpm.toFixed(0));
  set('throttle', (d.throttle * 100).toFixed(0) + '%');
  set('steering', d.steering.toFixed(2));
  set('heading',  d.heading.toFixed(1) + '°');
  set('cal_rot',  d.cal_rot + '/3');
  set('cal_acc',  d.cal_acc + '/3');
  set('lps',      d.lps.toFixed(0));
  if (d.cfg) {
    slide('targetSpeedMph',      d.cfg.sp,  'v_sp',   v => v.toFixed(1) + ' mph');
    slide('followDistanceCm',    d.cfg.fd,  'v_fd',   v => v.toFixed(0) + ' cm');
    slide('throttleScale',       d.cfg.ts,  'v_ts',   v => v.toFixed(3));
    slide('kp',                  d.cfg.kp,  'v_kp',   v => v.toFixed(2));
    slide('ki',                  d.cfg.ki,  'v_ki',   v => v.toFixed(3));
    slide('steeringKp',          d.cfg.sKp, 'v_sKp',  v => v.toFixed(4));
    slide('steeringKi',          d.cfg.sKi, 'v_sKi',  v => v.toFixed(4));
    slide('steeringMax',         d.cfg.sMax,'v_sMax',  v => v.toFixed(2));
    slide('uwbKalmanQ',          d.cfg.uQ,  'v_uQ',   v => v.toFixed(1));
    slide('uwbKalmanR',          d.cfg.uR,  'v_uR',   v => v.toFixed(1));
    slide('uwbOutlierRejectCm',  d.cfg.uOr, 'v_uOr',  v => v.toFixed(0) + ' cm');
    slide('fusionQBearingPerSec',d.cfg.fQ,  'v_fQ',   v => v.toFixed(1));
    slide('fusionRUwb',          d.cfg.fR,  'v_fR',   v => v.toFixed(0));
    slide('fusionStaleUncertainty',d.cfg.fSU,'v_fSU', v => v.toFixed(0));
    slide('fusionInnovEwmaAlpha',d.cfg.fEa, 'v_fEa',  v => v.toFixed(3));
    _targetSpeedMph = d.cfg.sp;
  }
  speedBuf.push(d.speed);
  speedBuf.shift();
  throttleBuf.push(d.cfg.ts > 0 ? d.throttle / d.cfg.ts : d.throttle);
  throttleBuf.shift();
  drawSpeedGraph();
  let thirdDeg, thirdLabel;
  if (d.navState === 'FOLLOW_ME') {
    thirdDeg   = valid ? d.angle : 0;
    thirdLabel = 'Target';
  } else {
    let err = d.headingHold - d.heading;
    while (err >  180) err -= 360;
    while (err < -180) err += 360;
    thirdDeg   = -err;
    thirdLabel = 'Target';
  }
  drawArrow(thirdDeg, -d.steering * 90, valid, d.navState !== 'STOPPED', thirdLabel);
  if (d.perf) { _perfData = d.perf; drawPerfGraph(); }
}

function slide(id, val, labelId, fmt) {
  const el = document.getElementById(id);
  if (document.activeElement !== el) el.value = val;
  set(labelId, fmt(val));
}

function setMode(m) {
  fetch('/mode?mode=' + m, {method:'POST'});
}

['throttleScale','followDistanceCm','targetSpeedMph','kp','ki',
 'steeringKp','steeringKi','steeringMax',
 'uwbKalmanQ','uwbKalmanR','uwbOutlierRejectCm',
 'fusionQBearingPerSec','fusionRUwb','fusionStaleUncertainty','fusionInnovEwmaAlpha'].forEach(id => {
  document.getElementById(id).addEventListener('change', () => {
    fetch('/config?key=' + id + '&value=' + document.getElementById(id).value, {method:'POST'});
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
  const maxVal = Math.max(5, _targetSpeedMph, ...speedBuf);
  gctx.strokeStyle = '#333';
  gctx.lineWidth = 1;
  gctx.beginPath(); gctx.moveTo(pad, pad); gctx.lineTo(pad, h-pad); gctx.lineTo(w-pad, h-pad); gctx.stroke();
  gctx.fillStyle = '#555';
  gctx.font = '11px monospace';
  gctx.fillText(maxVal.toFixed(1), pad+2, pad+11);
  gctx.fillText('0', pad+2, h-pad-2);
  const sl = pad + 14, sw = w - sl - pad;
  // Quarter-second ticks (10s window → 40 intervals); every 4th tick marks a full second
  gctx.lineWidth = 1;
  for (let i = 0; i <= 40; i++) {
    const x = sl + (i / 40) * sw;
    const major = (i % 4 === 0);
    gctx.strokeStyle = major ? '#555' : '#2d2d2d';
    gctx.beginPath(); gctx.moveTo(x, h-pad); gctx.lineTo(x, h-pad-(major ? 6 : 3)); gctx.stroke();
  }
  // Target speed reference line (dashed gray)
  if (_targetSpeedMph > 0 && seriesVisible.target) {
    const ty = h - pad - (_targetSpeedMph / maxVal) * (h - pad * 2);
    gctx.setLineDash([4, 4]);
    gctx.strokeStyle = '#666';
    gctx.lineWidth = 1;
    gctx.beginPath(); gctx.moveTo(sl, ty); gctx.lineTo(sl + sw, ty); gctx.stroke();
  }
  gctx.setLineDash([]);
  // 10s error label
  const avgSpeed = speedBuf.reduce((a, b) => a + b, 0) / speedBuf.length;
  const avgError = avgSpeed - _targetSpeedMph;
  gctx.fillStyle = '#fa4';
  gctx.font = 'bold 14px monospace';
  gctx.fillText('10s err: ' + (avgError >= 0 ? '+' : '') + avgError.toFixed(2), sl + 4, pad + 14);
  // Legend — top centre (click to toggle)
  legendBounds.length = 0;
  gctx.font = '11px monospace';
  const items = [
    { key: 'speed',    color: '#4af', label: 'Speed'    },
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
const PERF_LABELS  = ['IMU','UWB','Nav','Ctrl','OLED','WiFi'];
const PERF_COLORS  = ['#4f4','#4af','#fa4','#f66','#a4f','#4fa'];
function drawPerfGraph() {
  if (!_perfData) return;
  const w = perfCanvas.offsetWidth || 400;
  const h = perfCanvas.offsetHeight || 70;
  perfCanvas.width = w; perfCanvas.height = h;
  const pad = 4, labelH = 16, valH = 12;
  pctx.clearRect(0, 0, w, h);
  const vals = _perfMode === 'avg'
    ? [_perfData.ia, _perfData.ua, _perfData.na, _perfData.ca, _perfData.oa, _perfData.wa]
    : [_perfData.im, _perfData.um, _perfData.nm, _perfData.cm, _perfData.om, _perfData.wm];
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
            else if (key == "throttleFfK")            rtConfig.throttleFfK            = val;
            else if (key == "followDistanceCm")       rtConfig.followDistanceCm       = val;
            else if (key == "maxDistanceCm")          rtConfig.maxDistanceCm          = val;
            else if (key == "targetSpeedMph")         rtConfig.targetSpeedMph         = val;
            else if (key == "kp")                     rtConfig.kp                     = val;
            else if (key == "ki")                     rtConfig.ki                     = val;
            else if (key == "kd")                     rtConfig.kd                     = val;
            else if (key == "steeringKp")             rtConfig.steeringKp             = val;
            else if (key == "steeringKi")             rtConfig.steeringKi             = val;
            else if (key == "steeringMax")            rtConfig.steeringMax            = val;
            else if (key == "uwbKalmanQ")             rtConfig.uwbKalmanQ             = val;
            else if (key == "uwbKalmanR")             rtConfig.uwbKalmanR             = val;
            else if (key == "uwbOutlierRejectCm")     rtConfig.uwbOutlierRejectCm     = val;
            else if (key == "fusionQBearingPerSec")   rtConfig.fusionQBearingPerSec   = val;
            else if (key == "fusionRUwb")             rtConfig.fusionRUwb             = val;
            else if (key == "fusionStaleUncertainty") rtConfig.fusionStaleUncertainty = val;
            else if (key == "fusionInnovEwmaAlpha")   rtConfig.fusionInnovEwmaAlpha   = val;
            ESP_LOGI(TAG, "config %s = %.3f", key.c_str(), val);
        }
        req->send(200);
    });

    _server.on("/mode", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("mode")) {
            String m = req->getParam("mode")->value();
            if      (m == "FOLLOW_ME") nav_set_mode(NavMode::FOLLOW_ME);
            else if (m == "TEST")    nav_set_mode(NavMode::TEST);
            else if (m == "STOPPED") nav_set_mode(NavMode::STOPPED);
            ESP_LOGI(TAG, "mode → %s", m.c_str());
        }
        req->send(200);
    });

    _webSocket.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void*, uint8_t*, size_t) {
        if (type == WS_EVT_CONNECT)    ESP_LOGI(TAG, "WS client connected id=%u", client->id());
        if (type == WS_EVT_DISCONNECT) { ESP_LOGI(TAG, "WS client disconnected"); _webSocket.cleanupClients(); }
    });
    _server.addHandler(&_webSocket);
    _server.begin();
    ESP_LOGI(TAG, "✅ Dashboard at http://192.168.4.1/");
}

static float    safeF(float v) { return isnan(v) || isinf(v) ? 0.0f : v; }
static PerfData _perf = {};
static RateGate _gate{ DASHBOARD_UPDATE_INTERVAL_MS };
void dashboard_set_perf(const PerfData& p) { _perf = p; }

void dashboard_update(float lps) {
    const NavData&       nav   = nav_get();
    const Pose&     fused = fusion_get();
    const RPMData&       rpm   = rpm_get();
    const ImuData&       imu   = imu_get();
    const ControlOutput& ctrl  = control_get();
    float dt;
    if (!_gate.tick(dt)) return;

    _webSocket.cleanupClients();

    char buf[1200];
    snprintf(buf, sizeof(buf),
        "{\"dist\":%.1f,\"angle\":%.1f,\"headingHold\":%.1f,\"navState\":\"%s\",\"odometry\":%.0f,"
        "\"speed\":%.3f,\"rpm\":%.0f,"
        "\"heading\":%.1f,\"cal_rot\":%u,\"cal_acc\":%u,"
        "\"throttle\":%.3f,\"steering\":%.3f,\"lps\":%.0f,"
        "\"cfg\":{\"ts\":%.3f,\"sa\":%.3f,\"tff\":%.3f,\"fd\":%.0f,\"md\":%.0f,\"sp\":%.2f,"
        "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,"
        "\"sKp\":%.4f,\"sKi\":%.4f,\"sMax\":%.3f,"
        "\"uQ\":%.2f,\"uR\":%.2f,\"uOr\":%.1f,"
        "\"fQ\":%.2f,\"fR\":%.1f,\"fSU\":%.1f,\"fEa\":%.4f},"
        "\"perf\":{\"ia\":%u,\"im\":%u,\"ua\":%u,\"um\":%u,\"na\":%u,\"nm\":%u,"
        "\"ca\":%u,\"cm\":%u,\"oa\":%u,\"om\":%u,\"wa\":%u,\"wm\":%u}}",
        safeF(fused.distanceCm), safeF(fused.angle), safeF(nav.headingHold),
        nav.mode == NavMode::FOLLOW_ME ? "FOLLOW_ME" :
        nav.mode == NavMode::TEST      ? "TEST"      :
        nav.mode == NavMode::STOPPED   ? "STOPPED"   : "UNKNOWN",
        safeF(rpm.odometryCm),
        safeF(rpm.speedMph), safeF(rpm.rpm),
        safeF(imu.yaw), imu.cal_rot, imu.cal_accel,
        safeF(ctrl.throttle), safeF(ctrl.steering), safeF(lps),
        rtConfig.throttleScale, rtConfig.smoothAlpha, rtConfig.throttleFfK,
        rtConfig.followDistanceCm, rtConfig.maxDistanceCm,
        rtConfig.targetSpeedMph,
        rtConfig.kp, rtConfig.ki, rtConfig.kd,
        rtConfig.steeringKp, rtConfig.steeringKi, rtConfig.steeringMax,
        rtConfig.uwbKalmanQ, rtConfig.uwbKalmanR, rtConfig.uwbOutlierRejectCm,
        rtConfig.fusionQBearingPerSec, rtConfig.fusionRUwb,
        rtConfig.fusionStaleUncertainty, rtConfig.fusionInnovEwmaAlpha,
        _perf.imuAvg,  _perf.imuMax,
        _perf.uwbAvg,  _perf.uwbMax,
        _perf.navAvg,  _perf.navMax,
        _perf.ctrlAvg, _perf.ctrlMax,
        _perf.oledAvg, _perf.oledMax,
        _perf.wifiAvg, _perf.wifiMax);

    _webSocket.textAll(buf);
}

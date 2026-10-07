// Driver station page. Everything goes through one keyboard focus: the page
// sends what is held at 20 Hz over the WebSocket (the station turns it into
// /model/rover/cmd_vel and stops the rover when the page goes quiet), camera
// moves as small deltas, and draws the telemetry the station sends at 10 Hz.
"use strict";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const TICK = 50;  // [ms] input period, 20 Hz
const ORBIT_RATE = 1.6;  // [rad/s] chase camera with arrows / full stick
const LOOK_RATE = 1.0;  // [rad/s] looking around with arrows, I/K/J/L, D-pad, stick (the head slews at 2)
const ZOOM_RATE = 1.2;  // [e-folds/s] with +/- or the bumpers
const DRAG = 0.006;  // [rad/px] chase orbit
const WHEEL = 0.0015;  // [e-folds per wheel unit]
const LOOK_DEADZONE = 0.15;
const HISTORY = 300;  // rocker samples kept (30 s at 10 Hz)
const BLANK = "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7";

const VIEWS = ["eye", "chase", "rgb", "depth"];
const VIEW_NAMES = {eye: "Rover eye", chase: "Chase camera", rgb: "Onboard camera", depth: "Onboard depth"};
// Views from the rover's camera pivot: the look controls turn them in place
// (and the rover's camera head with them); in the chase view they orbit.
const LOOK_VIEWS = new Set(["eye", "rgb", "depth"]);
// KeyboardEvent.code values the station's drive mapping understands (drive.KEYS).
const DRIVE_KEYS = new Set(["KeyW", "KeyS", "KeyA", "KeyD", "ShiftLeft", "ShiftRight", "Space"]);
const HELD_KEYS = new Set([...DRIVE_KEYS, "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "KeyI", "KeyK",
  "KeyJ", "KeyL", "Equal", "NumpadAdd", "Minus", "NumpadSubtract"]);

const css = getComputedStyle(document.documentElement);
const color = (name) => css.getPropertyValue(name).trim();
const C = {bone: color("--bone"), dust: color("--dust"), ochre: color("--ochre"), sky: color("--sky"),
  signal: color("--signal"), rule: color("--rule"), basalt: color("--basalt"), dusk: color("--dusk")};

const state = {
  info: null,
  tm: null,
  ws: null,
  view: "eye",
  pip: true,
  held: new Set(),
  pending: {yaw: 0, pitch: 0, zoom: 0, pan: 0, tilt: 0},
  padButtons: [],
  lastTick: performance.now(),
  mapLevel: 0,
  mapImage: null,
  trail: [],
  rockers: [],
  viewSince: performance.now(),
};

// --- Connection ------------------------------------------------------------------

function send(msg) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) state.ws.send(JSON.stringify(msg));
}

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  state.ws = ws;
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.t === "telemetry") onTelemetry(msg);
    else if (msg.t === "error") console.warn(msg.text);
  };
  ws.onclose = () => {
    state.ws = null;
    setStatus("Station offline, reconnecting", "warn");
    setTimeout(connect, 1000);
  };
}

async function loadInfo() {
  try {
    const response = await fetch("/api/info");
    state.info = await response.json();
  } catch (error) {
    setTimeout(loadInfo, 1000);
    return;
  }
  const info = state.info;
  $("world").textContent = info.world;
  $("mission").textContent = info.map && info.map.mission ? info.map.mission : "";
  $("deadman-time").textContent = info.deadman;
  for (const el of document.querySelectorAll("[data-speed]")) {
    el.textContent = info.presets[el.dataset.speed][0];
  }
  if (info.map) {
    const image = new Image();
    image.onload = () => { state.mapImage = image; };
    image.src = "/minimap.png";
    $("map-note").textContent = "";
  } else {
    $("map-note").textContent = "This world has no mission sheet: the map shows your track on a grid.";
  }
  updateMapZoomLabel();
}

// --- Cameras -----------------------------------------------------------------------

function setStream(img, camera) {
  if ((img.dataset.camera || "") === (camera || "")) return;
  img.dataset.camera = camera || "";
  // A new src makes the browser drop the old MJPEG connection, so the station
  // stops encoding (and Gazebo stops rendering) cameras nobody watches.
  img.src = camera ? `/stream/${camera}?t=${Date.now()}` : BLANK;
}

// The small view shows the chase camera, or the rover eye while the chase camera is big.
const smallView = () => (state.view === "chase" ? "eye" : "chase");

function setView(view) {
  state.view = view;
  state.viewSince = performance.now();
  const small = smallView();
  setStream($("main-img"), view);
  setStream($("pip-img"), state.pip ? small : null);
  $("pip").hidden = !state.pip;
  $("pip-name").textContent = VIEW_NAMES[small];
  $("view-hint").textContent = view === "chase" ? "Drag to orbit, scroll to zoom" : "Drag to look around, double-click to centre";
  updateViewName();
}

function updateViewName() {
  let name = VIEW_NAMES[state.view];
  const m = state.tm;
  if (state.view === "chase" && m && m.chase) name += m.chase.mode === "follow" ? ", following" : ", holding the view";
  // The eye turns as commanded at once; the onboard camera as fast as its head does.
  const look = !m ? null : state.view === "eye" ? m.look : LOOK_VIEWS.has(state.view) ? m.head : null;
  if (look) name += `, ${lookText(look)}`;
  $("view-name").textContent = name;
}

function lookText({pan, tilt}) {
  const side = Math.round(Math.abs(pan * DEG));
  const down = Math.round(Math.abs(tilt * DEG));
  return `${side ? `${side}° ${pan > 0 ? "left" : "right"}` : "ahead"}, ${down ? `${down}° ${tilt > 0 ? "down" : "up"}` : "level"}`;
}

// Dragging in a look view turns it by the angle under the pointer (exact at
// the centre). The whole picture shows (object-fit: contain), so its drawn
// width is the smaller of the two fits.
function radiansPerPixel() {
  const img = $("main-img");
  const hfov = state.info ? state.info.hfov[state.view] : 1.5;
  const aspect = img.naturalWidth && img.naturalHeight ? img.naturalWidth / img.naturalHeight : 16 / 9;
  const width = Math.max(Math.min(img.clientWidth, img.clientHeight * aspect), 1);
  return 2 * Math.tan(hfov / 2) / width;
}

function recentre() {
  send(state.view === "chase" ? {t: "chase_mode", mode: "reset"} : {t: "look_center"});
}

for (const img of [$("main-img"), $("pip-img")]) {
  img.addEventListener("error", () => {
    const camera = img.dataset.camera;
    if (!camera) return;
    img.dataset.camera = "";
    setTimeout(() => { if (!img.dataset.camera) setStream(img, camera); }, 1500);
  });
}

// --- Keyboard, mouse, gamepad ----------------------------------------------------------

const TOGGLES = {
  KeyV: () => setView(VIEWS[(VIEWS.indexOf(state.view) + 1) % VIEWS.length]),
  KeyP: () => { state.pip = !state.pip; setView(state.view); },
  KeyF: () => toggleFollow(),
  KeyR: () => send({t: "chase_mode", mode: "reset"}),
  KeyC: () => send({t: "look_center"}),
  KeyH: () => toggleHelp(),
  KeyM: () => cycleMap(),
  Digit1: () => send({t: "preset", n: 1}),
  Digit2: () => send({t: "preset", n: 2}),
  Digit3: () => send({t: "preset", n: 3}),
  Escape: () => toggleHelp(false),
};

function toggleFollow() {
  const chase = state.tm && state.tm.chase;
  send({t: "chase_mode", mode: chase && chase.mode === "follow" ? "orbit" : "follow"});
}

function toggleHelp(show) {
  const help = $("help");
  help.hidden = show === undefined ? !help.hidden : !show;
}

window.addEventListener("keydown", (event) => {
  // macOS browsers send no keyup for a key released while Cmd is down, so a
  // held W would drive on: Cmd lets go of every key.
  if (event.metaKey) state.held.clear();
  if (event.metaKey || event.ctrlKey || event.altKey) return;
  const code = event.code;
  if (HELD_KEYS.has(code)) {
    state.held.add(code);
    event.preventDefault();
  } else if (TOGGLES[code]) {
    event.preventDefault();
    if (!event.repeat) TOGGLES[code]();
  }
});
window.addEventListener("keyup", (event) => state.held.delete(event.code));
window.addEventListener("blur", () => state.held.clear());
// A hidden tab drops its streams, so Gazebo stops rendering cameras nobody sees.
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) setView(state.view);
  else for (const img of [$("main-img"), $("pip-img")]) setStream(img, null);
});

const view = $("view");
let drag = null;
view.addEventListener("pointerdown", (event) => {
  if (event.button !== 0 || event.target.closest(".pip, .drivebar button, .focus-veil")) return;
  drag = {x: event.clientX, y: event.clientY};
  view.setPointerCapture(event.pointerId);
  view.classList.add("dragging");
});
view.addEventListener("pointermove", (event) => {
  if (!drag) return;
  const dx = event.clientX - drag.x;
  const dy = event.clientY - drag.y;
  drag = {x: event.clientX, y: event.clientY};
  // Grab the world: the scene follows the pointer.
  if (state.view === "chase") {
    state.pending.yaw -= dx * DRAG;
    state.pending.pitch += dy * DRAG;
  } else {
    const k = radiansPerPixel();
    state.pending.pan += dx * k;
    state.pending.tilt -= dy * k;
  }
});
const endDrag = () => { drag = null; view.classList.remove("dragging"); };
view.addEventListener("pointerup", endDrag);
view.addEventListener("pointercancel", endDrag);
view.addEventListener("dblclick", (event) => {
  if (!event.target.closest(".pip, .drivebar button, .focus-veil")) recentre();
});
view.addEventListener("wheel", (event) => {
  event.preventDefault();
  const scale = event.deltaMode === 1 ? 30 : 1;  // lines -> pixels
  state.pending.zoom += event.deltaY * scale * WHEEL;
}, {passive: false});

$("pip").addEventListener("click", () => setView(smallView()));
$("focus-veil").addEventListener("click", () => window.focus());
$("control").addEventListener("click", (event) => {
  send({t: "control", on: !(state.tm && state.tm.control)});
  event.currentTarget.blur();  // keep Space for stopping, not for this button
});
$("help-button").addEventListener("click", (event) => { toggleHelp(); event.currentTarget.blur(); });
$("help-close").addEventListener("click", () => toggleHelp(false));
$("help").addEventListener("click", (event) => { if (event.target === $("help")) toggleHelp(false); });
$("map").addEventListener("click", cycleMap);
$("map-zoom").addEventListener("click", (event) => { cycleMap(); event.currentTarget.blur(); });
for (const button of document.querySelectorAll(".preset")) {
  button.addEventListener("click", () => { send({t: "preset", n: Number(button.dataset.preset)}); button.blur(); });
}

function deadzone(value) {
  return Math.abs(value) < LOOK_DEADZONE ? 0 : value;
}

// Standard gamepad mapping: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 7 RT, 12-15 D-pad.
function readPad() {
  const none = {drive: [0, 0], look: [0, 0], zoom: 0, pan: 0, tilt: 0, fast: false, brake: false};
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  if (!pads.length) return none;
  const pad = pads[0];
  const axis = (k) => pad.axes[k] || 0;
  const pressed = (k) => Boolean(pad.buttons[k] && (pad.buttons[k].pressed || pad.buttons[k].value > 0.5));
  const was = state.padButtons;
  const edge = (k) => pressed(k) && !was[k];
  if (edge(2)) TOGGLES.KeyV();
  if (edge(3)) toggleFollow();
  state.padButtons = pad.buttons.map((b, k) => pressed(k));
  return {
    drive: [-axis(1), -axis(0)],  // stick up = forward, left = turn left
    look: [deadzone(axis(2)), deadzone(axis(3))],
    zoom: pressed(4) - pressed(5),
    pan: pressed(14) - pressed(15),
    tilt: pressed(13) - pressed(12),
    fast: pressed(7),
    brake: pressed(1),
  };
}

function hold(plus, minus) {
  return (state.held.has(plus) ? 1 : 0) - (state.held.has(minus) ? 1 : 0);
}

function tick() {
  const now = performance.now();
  const dt = Math.min((now - state.lastTick) / 1000, 0.2);
  state.lastTick = now;
  const active = document.hasFocus() && !document.hidden;
  document.body.classList.toggle("unfocused", !active);
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN || !active) {
    // Silence is the deadman: the station stops the rover after 0.5 s.
    state.held.clear();
    state.pending = {yaw: 0, pitch: 0, zoom: 0, pan: 0, tilt: 0};
    return;
  }
  const pad = readPad();
  const keys = [...state.held].filter((k) => DRIVE_KEYS.has(k));
  if (pad.fast) keys.push("GamepadFast");
  if (pad.brake) keys.push("GamepadBrake");
  send({t: "input", keys, axes: pad.drive});

  const p = state.pending;
  let {yaw, pitch} = p;
  const zoom = p.zoom + (hold("Minus", "Equal") + hold("NumpadSubtract", "NumpadAdd") + pad.zoom) * ZOOM_RATE * dt;
  let pan = p.pan + (hold("KeyJ", "KeyL") + pad.pan) * LOOK_RATE * dt;
  let tilt = p.tilt + (hold("KeyK", "KeyI") + pad.tilt) * LOOK_RATE * dt;
  // The arrows and the right stick move whichever camera fills the big view:
  // the look turns in place (left and down positive), the chase camera orbits.
  const across = hold("ArrowLeft", "ArrowRight") - pad.look[0];
  const up = hold("ArrowUp", "ArrowDown") - pad.look[1];
  if (LOOK_VIEWS.has(state.view)) {
    pan += across * LOOK_RATE * dt;
    tilt -= up * LOOK_RATE * dt;
  } else {
    yaw += across * ORBIT_RATE * dt;
    pitch += up * ORBIT_RATE * dt;
  }
  state.pending = {yaw: 0, pitch: 0, zoom: 0, pan: 0, tilt: 0};
  if (yaw || pitch || zoom) send({t: "chase", yaw, pitch, zoom});
  if (pan || tilt) send({t: "look", pan, tilt});
}

// --- Telemetry --------------------------------------------------------------------

function setStatus(text, tone) {
  const el = $("state");
  el.textContent = text;
  el.dataset.tone = tone || "";
}

function fmtTime(seconds) {
  if (seconds == null) return "–";
  const s = Math.floor(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const ss = String(s % 60).padStart(2, "0");
  return h ? `${h}:${String(m).padStart(2, "0")}:${ss}` : `${m}:${ss}`;
}

function signed(value, digits) {
  const text = Math.abs(value).toFixed(digits);
  return (value < 0 && Number(text) !== 0 ? "−" : value > 0 && Number(text) !== 0 ? "+" : "") + text;
}

const COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

function onTelemetry(m) {
  state.tm = m;
  const {stats, pose} = m;
  $("simtime").textContent = stats ? fmtTime(stats.sim_time) : "–";
  $("rtf").textContent = stats ? (stats.paused ? "paused" : `${Math.round(stats.rtf * 100)} %`) : "–";

  if (!stats) setStatus("Simulation not running: waiting for it", "warn");
  else if (m.others.length) setStatus(`Another driver station is attached (${m.others.join(", ")}): stop one`, "warn");
  else if (!m.control) setStatus("Autonomy has the rover", "calm");
  else if (document.body.classList.contains("unfocused")) setStatus("Holding still: click the page to drive", "warn");
  else if (m.deadman) setStatus("Holding still: no input", "warn");
  else setStatus("You are driving", "");
  const control = $("control");
  control.textContent = m.control ? "Release to autonomy" : "Take control";
  control.dataset.released = String(!m.control);
  $("led").dataset.led = m.led || "off";
  $("led").title = `Status LED: ${m.led || "off"} (URC rule 1.e.ii)`;

  for (const button of document.querySelectorAll(".preset")) {
    button.setAttribute("aria-pressed", String(Number(button.dataset.preset) === m.preset));
  }
  const [vx, wz] = m.cmd;
  $("cmd-speed").textContent = signed(vx, 2);
  $("cmd-turn").textContent = signed(wz * DEG, 0);
  if (pose) {
    $("speed").textContent = signed(pose.speed, 2);
    $("turn").textContent = signed(pose.yaw_rate * DEG, 0);
    const heading = ((90 - pose.yaw * DEG) % 360 + 360) % 360;  // compass, clockwise from north
    $("heading").textContent = `${String(Math.round(heading) % 360).padStart(3, "0")}° ${COMPASS[Math.round(heading / 45) % 8]}`;
    setAngle($("pitch"), -pose.pitch, 20);
    setAngle($("roll"), pose.roll, 20);
    $("alt-z").textContent = `${pose.z.toFixed(2)} m`;
    const last = state.trail[state.trail.length - 1];
    if (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) > 0.3) {
      state.trail.push([pose.x, pose.y]);
      if (state.trail.length > 4000) state.trail.shift();
    }
  }
  if (m.head) {
    $("head").textContent = `${signed(m.head.pan * DEG, 0)}°, ${signed(m.head.tilt * DEG, 0)}°`;
  }
  if (m.rockers) {
    const r = m.rockers;
    const nearStop = state.info ? 0.9 * state.info.rover.rocker_limit * DEG : Infinity;  // warn by the joint limit
    setAngle($("rocker-left"), r.left, nearStop);
    setAngle($("rocker-right"), r.right, nearStop);
    setAngle($("rocker-error"), r.error, 1, 2);
    state.rockers.push([r.left, r.right]);
    if (state.rockers.length > HISTORY) state.rockers.shift();
  }
  if (m.gnss) {
    $("lat").textContent = m.gnss.lat.toFixed(6);
    $("lon").textContent = m.gnss.lon.toFixed(6);
    $("alt").textContent = `${m.gnss.alt.toFixed(1)} m`;
  }
  showReferee(m.score, m.radio);
  showPictureState(m.cameras, Boolean(stats));
  updateViewName();
  draw();
}

function setAngle(el, radians, warnDegrees, digits = 1) {
  el.textContent = `${signed(radians * DEG, digits)}°`;
  el.dataset.tone = Math.abs(radians * DEG) > warnDegrees ? "warn" : "";
}

function showReferee(score, radio) {
  $("referee").hidden = !score && !radio;
  if (score) {
    $("points").textContent = score.points;
    $("max-points").textContent = score.max_points;
    const events = $("events");
    events.replaceChildren(...score.events.slice(-6).reverse().map((text) => {
      const li = document.createElement("li");
      li.textContent = text.trim();
      return li;
    }));
  }
  const el = $("radio");
  el.textContent = radio ? (radio === "los" ? "Radio: line of sight to C2" : "Radio: no line of sight") : "";
  el.dataset.los = String(radio === "los");
}

function showPictureState(cameras, running) {
  const fps = cameras ? cameras[state.view] : 0;
  const waiting = !fps && performance.now() - state.viewSince > 2500;
  $("no-picture").hidden = !waiting;
  if (!waiting) return;
  const camera = {eye: "The rover eye", chase: "The chase camera"}[state.view] || "The onboard camera";
  $("no-picture-text").textContent = !running
    ? "The simulation is not running. When it starts again, the station brings its cameras back."
    : `${camera} has not sent a picture. Cameras render only in worlds with the `
      + "Sensors system, and the first picture of a big world can take a while.";
}

// --- Drawing ------------------------------------------------------------------------

function fit(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const w = Math.round(canvas.clientWidth * ratio);
  const h = Math.round(canvas.clientHeight * ratio);
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  return [ctx, canvas.clientWidth, canvas.clientHeight];
}

function draw() {
  drawStick();
  drawRocker();
  drawHistory();
  drawWheels();
  drawMap();
}

// Command (ochre) and response (sky) on one pad: up is forward, left turns left.
function drawStick() {
  const [ctx, w, h] = fit($("stick"));
  const m = state.tm;
  if (!m || !state.info) return;
  const [maxV, maxW] = state.info.max;
  const r = w / 2 - 6;
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(w / 2, h / 2, r, 0, 2 * Math.PI);
  ctx.moveTo(w / 2 - r, h / 2); ctx.lineTo(w / 2 + r, h / 2);
  ctx.moveTo(w / 2, h / 2 - r); ctx.lineTo(w / 2, h / 2 + r);
  ctx.stroke();
  const point = (v, wz) => [w / 2 - Math.max(-1, Math.min(1, wz / maxW)) * r,
    h / 2 - Math.max(-1, Math.min(1, v / maxV)) * r];
  if (m.pose) {
    const [x, y] = point(m.pose.speed, m.pose.yaw_rate);
    ctx.strokeStyle = C.sky;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, 2 * Math.PI);
    ctx.stroke();
  }
  const [x, y] = point(m.cmd[0], m.cmd[1]);
  ctx.fillStyle = C.ochre;
  ctx.beginPath();
  ctx.arc(x, y, 3.5, 0, 2 * Math.PI);
  ctx.fill();
}

// Side view, front to the right: body pitch and both rocker angles to scale.
function drawRocker() {
  const [ctx, w, h] = fit($("rocker"));
  if (!state.info) return;
  const r = state.info.rover;
  const m = state.tm || {};
  const pitch = m.pose ? m.pose.pitch : 0;
  const q = m.rockers ? [m.rockers.left, m.rockers.right] : [0, 0];
  const length = 2 * (r.wheel_dx + r.wheel_radius);
  const s = Math.min((w - 24) / length, (h - 20) / (r.chassis_z + r.chassis_size[2] / 2 + 0.05));
  const ox = w / 2;
  const oy = h - 10;
  // gz pitch is about +y: positive puts the nose down.
  const toCanvas = (x, z) => [ox + s * (x * Math.cos(pitch) + z * Math.sin(pitch)),
    oy - s * (-x * Math.sin(pitch) + z * Math.cos(pitch))];
  const rocker = (angle, x, z) => [x * Math.cos(angle) + z * Math.sin(angle),
    r.pivot_z - x * Math.sin(angle) + z * Math.cos(angle)];

  ctx.setLineDash([3, 4]);
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(8, oy); ctx.lineTo(w - 8, oy);
  ctx.stroke();
  ctx.setLineDash([]);

  const [lx, , lz] = r.chassis_size;
  const corners = [[-lx / 2, r.chassis_z - lz / 2], [lx / 2, r.chassis_z - lz / 2],
    [lx / 2, r.chassis_z + lz / 2], [-lx / 2, r.chassis_z + lz / 2]].map(([x, z]) => toCanvas(x, z));
  ctx.fillStyle = C.ochre;
  ctx.globalAlpha = 0.85;
  ctx.beginPath();
  corners.forEach(([x, y], k) => (k ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.closePath();
  ctx.fill();
  ctx.globalAlpha = 1;

  // Left rocker (bone) behind, right rocker (sky) in front.
  [[q[0], C.bone, 0.8], [q[1], C.sky, 1]].forEach(([angle, stroke, alpha]) => {
    const front = rocker(angle, r.wheel_dx, r.wheel_dz);
    const rear = rocker(angle, -r.wheel_dx, r.wheel_dz);
    const pivot = toCanvas(0, r.pivot_z);
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 4;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(...toCanvas(...front));
    ctx.lineTo(...pivot);
    ctx.lineTo(...toCanvas(...rear));
    ctx.stroke();
    ctx.lineWidth = 2;
    for (const hub of [front, rear]) {
      ctx.beginPath();
      ctx.arc(...toCanvas(...hub), r.wheel_radius * s, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  });
  ctx.fillStyle = C.dusk;
  ctx.strokeStyle = C.bone;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(...toCanvas(0, r.pivot_z), 4, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = C.dust;
  ctx.font = `12px ${css.getPropertyValue("--text")}`;
  ctx.fillText("front", w - 40, 14);
}

// Both rocker angles over the last 30 s; the scale grows with the motion.
function drawHistory() {
  const [ctx, w, h] = fit($("rocker-history"));
  const data = state.rockers;
  const peak = Math.max(2, ...data.map(([a, b]) => Math.max(Math.abs(a), Math.abs(b)) * DEG * 1.2));
  const y = (radians) => h / 2 - (radians * DEG / peak) * (h / 2 - 6);
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
  ctx.stroke();
  ctx.fillStyle = C.dust;
  ctx.font = `11px ${css.getPropertyValue("--text")}`;
  ctx.fillText(`±${peak.toFixed(1)}°`, 2, 11);
  ctx.fillText("30 s", w - 26, h - 3);
  [[0, C.bone], [1, C.sky]].forEach(([k, stroke]) => {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    data.forEach((sample, i) => {
      const x = w - (data.length - 1 - i) * (w / (HISTORY - 1));
      if (i) ctx.lineTo(x, y(sample[k])); else ctx.moveTo(x, y(sample[k]));
    });
    ctx.stroke();
  });
}

// Plan view, front up: each wheel's ground speed as a bar from its hub.
function drawWheels() {
  const [ctx, w, h] = fit($("wheels"));
  const m = state.tm;
  if (!state.info) return;
  const max = state.info.max[0];
  const spots = {fl: [0.27, 0.27], fr: [0.73, 0.27], rl: [0.27, 0.73], rr: [0.73, 0.73]};
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.strokeRect(w * 0.36, h * 0.18, w * 0.28, h * 0.64);
  ctx.font = `13px ${css.getPropertyValue("--numerals")}`;
  ctx.textAlign = "center";
  for (const [wheel, [fx, fy]] of Object.entries(spots)) {
    const x = fx * w + (fx < 0.5 ? -14 : 14);
    const y = fy * h;
    ctx.strokeStyle = C.dust;
    ctx.strokeRect(x - 5, y - 17, 10, 34);
    const v = m && m.wheels ? m.wheels[wheel] || 0 : 0;
    const bar = Math.max(-1, Math.min(1, v / max)) * 17;
    ctx.fillStyle = C.sky;
    ctx.fillRect(x - 3, Math.min(y, y - bar), 6, Math.abs(bar));
    ctx.fillStyle = C.bone;
    ctx.fillText(signed(v, 2), fx < 0.5 ? x + 1 : x - 1, fy < 0.5 ? y - 22 : y + 30);
  }
  ctx.textAlign = "start";
}

function mapSpans() {
  const map = state.info && state.info.map;
  if (!map) return [40, 12, 150];
  const full = Math.max(...map.size);
  return [full, ...[250, 60].filter((span) => span < full)];
}

function cycleMap() {
  state.mapLevel = (state.mapLevel + 1) % mapSpans().length;
  updateMapZoomLabel();
  drawMap();
}

function updateMapZoomLabel() {
  const span = mapSpans()[state.mapLevel];
  const whole = state.info && state.info.map && state.mapLevel === 0;
  $("map-zoom").textContent = whole ? "Whole terrain" : `${span} m around the rover`;
}

function niceStep(span) {
  const raw = span / 5;
  const power = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((k) => k * power).find((step) => step >= raw);
}

// North up, world x east and y north, origin at the terrain centre.
function drawMap() {
  const canvas = $("map");
  const [ctx, w, h] = fit(canvas);
  const map = state.info && state.info.map;
  const pose = state.tm && state.tm.pose;
  const span = mapSpans()[state.mapLevel];
  const centered = !(map && state.mapLevel === 0) && pose;
  const cx = centered ? pose.x : 0;
  const cy = centered ? pose.y : 0;
  const k = w / span;
  const px = (x) => w / 2 + (x - cx) * k;
  const py = (y) => h / 2 - (y - cy) * k;

  ctx.fillStyle = C.dusk;
  ctx.fillRect(0, 0, w, h);
  if (map && state.mapImage) {
    const [sx, sy] = map.size;
    ctx.drawImage(state.mapImage, px(-sx / 2), py(sy / 2), sx * k, sy * k);
  } else {
    const step = niceStep(span);
    ctx.strokeStyle = C.rule;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil((cx - span) / step) * step; x <= cx + span; x += step) {
      ctx.moveTo(px(x), 0); ctx.lineTo(px(x), h);
    }
    for (let y = Math.ceil((cy - span) / step) * step; y <= cy + span; y += step) {
      ctx.moveTo(0, py(y)); ctx.lineTo(w, py(y));
    }
    ctx.stroke();
    ctx.fillStyle = C.dust;
    ctx.font = `11px ${css.getPropertyValue("--text")}`;
    ctx.fillText(`grid ${step} m`, 6, h - 6);
  }

  if (map) {
    ctx.font = `12px ${css.getPropertyValue("--text")}`;
    for (const place of map.places) {
      const x = px(place.x);
      const y = py(place.y);
      if (x < -40 || x > w + 40 || y < -20 || y > h + 20) continue;
      ctx.fillStyle = place.kind === "c2" ? C.ochre : C.bone;
      ctx.strokeStyle = C.dusk;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y - 5); ctx.lineTo(x + 5, y); ctx.lineTo(x, y + 5); ctx.lineTo(x - 5, y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      const label = place.name.replace(/_/g, " ");
      ctx.lineWidth = 3;
      ctx.strokeText(label, x + 8, y + 4);
      ctx.fillText(label, x + 8, y + 4);
    }
  }

  if (state.trail.length > 1) {
    ctx.strokeStyle = C.sky;
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 0.8;
    ctx.beginPath();
    state.trail.forEach(([x, y], i) => (i ? ctx.lineTo(px(x), py(y)) : ctx.moveTo(px(x), py(y))));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
  if (pose) {
    ctx.save();
    ctx.translate(px(pose.x), py(pose.y));
    ctx.rotate(-pose.yaw);
    ctx.fillStyle = C.ochre;
    ctx.strokeStyle = C.dusk;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(11, 0); ctx.lineTo(-7, 7); ctx.lineTo(-3, 0); ctx.lineTo(-7, -7);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
  ctx.fillStyle = C.dust;
  ctx.font = `600 12px ${css.getPropertyValue("--text")}`;
  ctx.fillText("N", w - 14, 16);
}

// --- Start --------------------------------------------------------------------------

loadInfo();
connect();
setView("eye");
setInterval(tick, TICK);
window.addEventListener("resize", draw);
window.addEventListener("focus", () => document.body.classList.remove("unfocused"));

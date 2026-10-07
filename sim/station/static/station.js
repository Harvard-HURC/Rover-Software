// Driver station page. Everything goes through one keyboard focus: the page
// sends what is held at 20 Hz over the WebSocket (the station turns it into
// /model/rover/cmd_vel and stops the rover when the page goes quiet), camera
// moves as small deltas, and draws the telemetry the station sends at 10 Hz.
// Fly and Map are inspection views only a simulation has: there the keyboard
// flies the fly camera or moves the map, and only a gamepad drives the rover.
"use strict";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const TICK = 50;  // [ms] input period, 20 Hz
const ORBIT_RATE = 1.6;  // [rad/s] chase camera with arrows / full stick
const LOOK_RATE = 1.0;  // [rad/s] looking around with arrows, I/K/J/L, D-pad, stick (the head slews at 2)
const ZOOM_RATE = 1.2;  // [e-folds/s] with +/- or the bumpers; also the fly speed and the map zoom
const DRAG = 0.006;  // [rad/px] chase orbit
const WHEEL = 0.0015;  // [e-folds per wheel unit]
const LOOK_DEADZONE = 0.15;
const HISTORY = 300;  // samples kept for the history plots (30 s at 10 Hz)
const MAP_PAN_RATE = 0.6;  // [map widths/s] with the arrows in the map view
const MAP_SPAN = [10, 4];  // the map view's width: from 10 m to 4 terrain widths
const CLICK_DELAY = 260;  // [ms] a click on the map waits this long for a double-click
const DRAG_SLOP = 4;  // [px] a press that moves less than this is a click, not a drag
const TURNING = 0.05;  // [rad/s] the turn ratio shows only while asked to turn faster than this
const DUG = 1.05;  // dig factor from which a wheel counts as dug in
const BLANK = "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7";

const DRIVE_VIEWS = ["eye", "chase", "rgb", "depth"];
const INSPECT = new Set(["fly", "map"]);
const VIEW_NAMES = {eye: "Rover eye", chase: "Chase camera", rgb: "Onboard camera", depth: "Onboard depth",
  fly: "Fly camera", map: "Map"};
const HINTS = {
  eye: "Drag to look around, double-click to centre",
  rgb: "Drag to look around, double-click to centre",
  depth: "Drag to look around, double-click to centre",
  chase: "Drag to orbit, scroll to zoom",
  fly: "W A S D fly, E Q up and down, drag to look, scroll for speed; R rover, F follow, T top-down, O orthographic",
  map: "Click to fly the fly camera there, double-click to look straight down there; drag and scroll to move",
};
// Views from the rover's camera pivot: the look controls turn them in place
// (and the rover's camera head with them); in the chase view they orbit.
const LOOK_VIEWS = new Set(["eye", "rgb", "depth"]);
// KeyboardEvent.code values the station's mappings understand (drive.KEYS, drive.FLY_KEYS).
const DRIVE_KEYS = new Set(["KeyW", "KeyS", "KeyA", "KeyD", "ShiftLeft", "ShiftRight", "Space"]);
const FLY_KEYS = new Set(["KeyW", "KeyS", "KeyA", "KeyD", "KeyE", "KeyQ", "ArrowLeft", "ArrowRight", "ArrowUp",
  "ArrowDown", "KeyJ", "KeyL", "KeyI", "KeyK", "ShiftLeft", "ShiftRight"]);
const ZOOM_KEYS = ["Equal", "NumpadAdd", "Minus", "NumpadSubtract"];
const HELD = {  // keys that act while held, per view
  drive: new Set([...DRIVE_KEYS, "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "KeyI", "KeyK", "KeyJ", "KeyL",
    ...ZOOM_KEYS]),
  fly: new Set([...FLY_KEYS, ...ZOOM_KEYS]),
  map: new Set(["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "KeyI", "KeyK", "KeyJ", "KeyL", ...ZOOM_KEYS]),
};
const WHEELS = ["fl", "fr", "rl", "rr"];

const css = getComputedStyle(document.documentElement);
const color = (name) => css.getPropertyValue(name).trim();
const C = {bone: color("--bone"), dust: color("--dust"), ochre: color("--ochre"), sky: color("--sky"),
  signal: color("--signal"), sage: color("--sage"), rule: color("--rule"), basalt: color("--basalt"),
  dusk: color("--dusk")};

const NO_PENDING = () => ({yaw: 0, pitch: 0, zoom: 0, pan: 0, tilt: 0, flyYaw: 0, flyPitch: 0, flySpeed: 0});
const state = {
  info: null,
  tm: null,
  ws: null,
  view: "eye",
  lastDrive: "eye",  // the driving view Fly and Map return to
  pip: true,
  held: new Set(),
  pending: NO_PENDING(),
  padButtons: [],
  pad: false,  // a gamepad is connected
  lastTick: performance.now(),
  mapLevel: 0,
  mapLayer: "photo",
  images: {photo: null, relief: null},
  mapView: null,  // the map view's {cx, cy, span [m]}
  mapClick: null,  // timer of a map click waiting for a double-click
  trail: [],
  rockers: [],
  yaws: [],  // [asked, got] turn rates [rad/s]
  viewSince: performance.now(),
};

// --- Connection ------------------------------------------------------------------

function send(msg) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) state.ws.send(JSON.stringify(msg));
}

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  state.ws = ws;
  ws.onopen = () => send({t: "view", view: state.view});
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
  const map = info.map;
  const photo = map && map.photo;
  const load = (url, layer) => {
    const image = new Image();
    image.onload = () => { state.images[layer] = image; showLayer(); draw(); drawMapView(); };
    image.src = url;
  };
  if (map) load("/minimap.png", "relief");
  if (photo && photo.url) load(photo.url, "photo");
  state.mapLayer = photo && photo.url ? "photo" : "relief";
  for (const button of document.querySelectorAll(".layer[data-layer='photo']")) {
    button.textContent = photo && photo.status === "stale" ? "Photo, out of date" : "Photo";
  }
  showLayer();
  const command = `pixi run sim-maps ${info.world}`;
  $("map-note").textContent = !map ? "This world has no mission sheet: the map shows your track on a grid."
    : !photo || photo.status === "missing" ? `No photo map yet (${command}): the map shows the relief.`
      : photo.status === "stale" ? `The photo map is out of date: the world changed since it was rendered (${command}).`
        : "";
  updateMapZoomLabel();
}

// --- Views and cameras ---------------------------------------------------------------

function setStream(img, camera) {
  if ((img.dataset.camera || "") === (camera || "")) return;
  img.dataset.camera = camera || "";
  // A new src makes the browser drop the old MJPEG connection, so the station
  // stops encoding (and Gazebo stops rendering) cameras nobody watches.
  img.src = camera ? `/stream/${camera}?t=${Date.now()}` : BLANK;
}

// The small view shows the chase camera, or the rover eye while the chase
// camera, the fly camera or the map is big.
const smallView = () => (state.view === "chase" || INSPECT.has(state.view) ? "eye" : "chase");

function setView(view) {
  if (view !== state.view) state.held.clear();  // a key held into another view would act there
  if (DRIVE_VIEWS.includes(view)) state.lastDrive = view;
  state.view = view;
  state.viewSince = performance.now();
  const small = smallView();
  const map = view === "map";
  if (map && !state.mapView) state.mapView = wholeMap();
  setStream($("main-img"), map ? null : view);
  $("main-img").hidden = map;
  $("map-view").hidden = !map;
  setStream($("pip-img"), state.pip ? small : null);
  $("pip").hidden = !state.pip;
  $("pip-name").textContent = VIEW_NAMES[small] + (INSPECT.has(view) ? ": click to go back" : "");
  $("view").dataset.view = view;
  $("sim-only").hidden = !INSPECT.has(view);
  $("view-hint").textContent = HINTS[view];
  $("fly-button").setAttribute("aria-pressed", String(view === "fly"));
  $("map-button").setAttribute("aria-pressed", String(map));
  send({t: "view", view});
  updateViewName();
  showLayer();
  drawMapView();
}

const toggleView = (view) => setView(state.view === view ? state.lastDrive : view);

function updateViewName() {
  let name = VIEW_NAMES[state.view];
  const m = state.tm;
  if (state.view === "chase" && m && m.chase) name += m.chase.mode === "follow" ? ", following" : ", holding the view";
  if (state.view === "fly" && m && m.fly) name += `, ${flyText(m.fly)}`;
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

function flyText(f) {
  const parts = [f.mode === "follow" ? "following the rover" : null,
    f.ortho > 0 ? `orthographic ${Math.round(f.ortho)} m wide, no cast shadows`
      : f.pitch > Math.PI / 2 - 0.01 ? "straight down" : null,
    `${Math.round(f.agl)} m above the ground`, `speed ×${Number(f.speed.toFixed(2))}`];
  return parts.filter(Boolean).join(", ");
}

// Dragging in a look view turns it by the angle under the pointer (exact at
// the centre). The whole picture shows (object-fit: contain), so its drawn
// width is the smaller of the two fits.
function pictureRect() {
  const img = $("main-img");
  const aspect = img.naturalWidth && img.naturalHeight ? img.naturalWidth / img.naturalHeight : 16 / 9;
  const width = Math.max(Math.min(img.clientWidth, img.clientHeight * aspect), 1);
  const height = width / aspect;
  return {x: (img.clientWidth - width) / 2, y: (img.clientHeight - height) / 2, width, height};
}

function radiansPerPixel() {
  const hfov = state.info ? state.info.hfov[state.view] : 1.5;
  return 2 * Math.tan(hfov / 2) / pictureRect().width;
}

function recentre(event) {
  if (state.view === "chase") send({t: "chase_mode", mode: "reset"});
  else if (state.view === "fly") {
    // Fly to the spot under the pointer (the station marches its ray against the heightmap).
    const box = $("view").getBoundingClientRect();
    const r = pictureRect();
    const u = (event.clientX - box.left - r.x) / r.width;
    const v = (event.clientY - box.top - r.y) / r.height;
    if (u >= 0 && u <= 1 && v >= 0 && v <= 1) send({t: "fly_goto", kind: "pixel", u, v});
  } else if (state.view !== "map") send({t: "look_center"});
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

const heldKeys = () => (state.view === "fly" ? HELD.fly : state.view === "map" ? HELD.map : HELD.drive);
const fly = () => (state.tm && state.tm.fly) || null;

const TOGGLES = {  // every view
  KeyV: () => setView(INSPECT.has(state.view) ? state.lastDrive
    : DRIVE_VIEWS[(DRIVE_VIEWS.indexOf(state.view) + 1) % DRIVE_VIEWS.length]),
  KeyB: () => toggleView("fly"),
  KeyG: () => toggleView("map"),
  KeyP: () => { state.pip = !state.pip; setView(state.view); },
  KeyH: () => toggleHelp(),
  KeyM: () => cycleMap(),
  Digit1: () => send({t: "preset", n: 1}),
  Digit2: () => send({t: "preset", n: 2}),
  Digit3: () => send({t: "preset", n: 3}),
  Escape: () => toggleHelp(false),
};
const VIEW_TOGGLES = {
  drive: {
    KeyF: () => toggleFollow(),
    KeyR: () => send({t: "chase_mode", mode: "reset"}),
    KeyC: () => send({t: "look_center"}),
  },
  fly: {
    KeyR: () => send({t: "fly_goto", kind: "rover"}),
    KeyF: () => send({t: "fly_mode", mode: fly() && fly().mode === "follow" ? "free" : "follow"}),
    KeyT: () => send({t: "fly_mode", mode: "top"}),
    KeyC: () => send({t: "fly_mode", mode: "level"}),
    KeyO: () => send({t: "fly_mode", mode: fly() && fly().ortho > 0 ? "perspective" : "ortho"}),
    Space: () => send({t: "fly_mode", mode: "stop"}),
  },
  map: {
    KeyC: () => centreMap(),
  },
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
  const toggle = (VIEW_TOGGLES[INSPECT.has(state.view) ? state.view : "drive"] || {})[code] || TOGGLES[code];
  if (heldKeys().has(code)) {
    state.held.add(code);
    event.preventDefault();
  } else if (toggle) {
    event.preventDefault();
    if (!event.repeat) toggle();
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
  if (event.button !== 0 || event.target.closest(".pip, .drivebar button, .focus-veil, .layers")) return;
  drag = {x: event.clientX, y: event.clientY, x0: event.clientX, y0: event.clientY};
  view.setPointerCapture(event.pointerId);
  view.classList.add("dragging");
});
view.addEventListener("pointermove", (event) => {
  if (!drag) return;
  const dx = event.clientX - drag.x;
  const dy = event.clientY - drag.y;
  drag.x = event.clientX;
  drag.y = event.clientY;
  // Grab the world: the scene follows the pointer.
  if (state.view === "map") {
    const k = state.mapView.span / $("map-view").clientWidth;
    state.mapView.cx -= dx * k;
    state.mapView.cy += dy * k;
    drawMapView();
  } else if (state.view === "chase") {
    state.pending.yaw -= dx * DRAG;
    state.pending.pitch += dy * DRAG;
  } else if (state.view === "fly") {
    const k = radiansPerPixel();
    state.pending.flyYaw += dx * k;
    state.pending.flyPitch -= dy * k;
  } else {
    const k = radiansPerPixel();
    state.pending.pan += dx * k;
    state.pending.tilt -= dy * k;
  }
});
const endDrag = (event) => {
  if (drag && event.type === "pointerup" && state.view === "map"
      && Math.hypot(event.clientX - drag.x0, event.clientY - drag.y0) < DRAG_SLOP) {
    const [x, y] = mapWorld(event);
    clearTimeout(state.mapClick);  // a click flies there, unless a double-click follows
    state.mapClick = setTimeout(() => send({t: "fly_goto", kind: "point", x, y, span: state.mapView.span}),
      CLICK_DELAY);
  }
  drag = null;
  view.classList.remove("dragging");
};
view.addEventListener("pointerup", endDrag);
view.addEventListener("pointercancel", endDrag);
view.addEventListener("dblclick", (event) => {
  if (event.target.closest(".pip, .drivebar button, .focus-veil, .layers")) return;
  if (state.view === "map") {
    clearTimeout(state.mapClick);
    const [x, y] = mapWorld(event);
    send({t: "fly_goto", kind: "top", x, y, span: state.mapView.span});
    setView("fly");
  } else recentre(event);
});
view.addEventListener("wheel", (event) => {
  event.preventDefault();
  const amount = event.deltaY * (event.deltaMode === 1 ? 30 : 1) * WHEEL;  // lines -> pixels
  if (state.view === "map") zoomMap(Math.exp(amount), event);
  else if (state.view === "fly") state.pending.flySpeed -= amount;  // up: faster
  else state.pending.zoom += amount;
}, {passive: false});

$("pip").addEventListener("click", () => setView(INSPECT.has(state.view) ? "eye" : smallView()));
$("focus-veil").addEventListener("click", () => window.focus());
$("control").addEventListener("click", (event) => {
  send({t: "control", on: !(state.tm && state.tm.control)});
  event.currentTarget.blur();  // keep Space for stopping, not for this button
});
$("fly-button").addEventListener("click", (event) => { toggleView("fly"); event.currentTarget.blur(); });
$("map-button").addEventListener("click", (event) => { toggleView("map"); event.currentTarget.blur(); });
$("help-button").addEventListener("click", (event) => { toggleHelp(); event.currentTarget.blur(); });
$("help-close").addEventListener("click", () => toggleHelp(false));
$("help").addEventListener("click", (event) => { if (event.target === $("help")) toggleHelp(false); });
$("map").addEventListener("click", () => setView("map"));
$("map-zoom").addEventListener("click", (event) => { cycleMap(); event.currentTarget.blur(); });
for (const button of document.querySelectorAll(".layer")) {
  button.addEventListener("click", (event) => {
    state.mapLayer = button.dataset.layer;
    showLayer();
    draw();
    drawMapView();
    event.currentTarget.blur();
  });
}
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
  state.pad = pads.length > 0;
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

const zoomHeld = () => hold("Minus", "Equal") + hold("NumpadSubtract", "NumpadAdd");  // out +

function tick() {
  const now = performance.now();
  const dt = Math.min((now - state.lastTick) / 1000, 0.2);
  state.lastTick = now;
  const active = document.hasFocus() && !document.hidden;
  document.body.classList.toggle("unfocused", !active);
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN || !active) {
    // Silence is the deadman: the station stops the rover after 0.5 s.
    state.held.clear();
    state.pending = NO_PENDING();
    return;
  }
  const pad = readPad();
  const driving = !INSPECT.has(state.view);
  // In Fly and Map the keyboard does not drive: the rover slows to a stop and
  // holds, the deadman stays satisfied and only a gamepad drives.
  const keys = driving ? [...state.held].filter((k) => DRIVE_KEYS.has(k)) : [];
  if (pad.fast) keys.push("GamepadFast");
  if (pad.brake) keys.push("GamepadBrake");
  send({t: "input", keys, axes: pad.drive});

  const p = state.pending;
  let {yaw, pitch, zoom, pan, tilt} = p;
  // The D-pad always looks with the rover eye, J/L/I/K except in Fly (where they turn the fly camera).
  pan += pad.pan * LOOK_RATE * dt;
  tilt += pad.tilt * LOOK_RATE * dt;
  if (state.view !== "fly") {
    pan += hold("KeyJ", "KeyL") * LOOK_RATE * dt;
    tilt += hold("KeyK", "KeyI") * LOOK_RATE * dt;
  }
  if (state.view === "fly") {
    send({t: "fly", keys: [...state.held].filter((k) => FLY_KEYS.has(k)), axes: pad.look});
    const speed = p.flySpeed - zoomHeld() * ZOOM_RATE * dt;
    if (speed) send({t: "fly_speed", factor: Math.exp(speed)});
    if (p.flyYaw || p.flyPitch) send({t: "fly_look", yaw: p.flyYaw, pitch: p.flyPitch});
  } else if (state.view === "map") {
    const across = hold("ArrowRight", "ArrowLeft");
    const up = hold("ArrowUp", "ArrowDown");
    if (across || up) {
      state.mapView.cx += across * MAP_PAN_RATE * state.mapView.span * dt;
      state.mapView.cy += up * MAP_PAN_RATE * state.mapView.span * dt;
      drawMapView();
    }
    if (zoomHeld()) zoomMap(Math.exp(zoomHeld() * ZOOM_RATE * dt));
  } else {
    zoom += (zoomHeld() + pad.zoom) * ZOOM_RATE * dt;
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
  }
  state.pending = NO_PENDING();
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

function compass(yaw) {
  const heading = ((90 - yaw * DEG) % 360 + 360) % 360;  // clockwise from north
  return `${String(Math.round(heading) % 360).padStart(3, "0")}° ${COMPASS[Math.round(heading / 45) % 8]}`;
}

// The turn the rover is asked for: what the physical drivetrain applies (it
// also sees autonomy's commands, and drops a stale one), else what the station sends.
const askedTurn = (m) => (m.drivetrain ? m.drivetrain.cmd[1] : m.cmd[1]);

function onTelemetry(m) {
  state.tm = m;
  const {stats, pose} = m;
  $("simtime").textContent = stats ? fmtTime(stats.sim_time) : "–";
  $("rtf").textContent = stats ? (stats.paused ? "paused" : `${Math.round(stats.rtf * 100)} %`) : "–";

  if (!stats) setStatus("Simulation not running: waiting for it", "warn");
  else if (m.others.length) setStatus(`Another driver station is attached (${m.others.join(", ")}): stop one`, "warn");
  else if (!m.control) setStatus("Autonomy has the rover", "calm");
  else if (document.body.classList.contains("unfocused")) setStatus("Holding still: click the page to drive", "warn");
  else if (INSPECT.has(state.view)) {
    setStatus(state.pad ? "Inspecting: only the gamepad drives" : "Inspecting: the rover holds still", "calm");
  } else if (m.deadman) setStatus("Holding still: no input", "warn");
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
  const asked = askedTurn(m);
  $("turn-ratio").textContent = pose && Math.abs(asked) > TURNING ? `got ${Math.round(100 * pose.yaw_rate / asked)} %` : "";
  if (pose) {
    $("speed").textContent = signed(pose.speed, 2);
    $("turn").textContent = signed(pose.yaw_rate * DEG, 0);
    const last = state.trail[state.trail.length - 1];
    if (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) > 0.3) {
      state.trail.push([pose.x, pose.y]);
      if (state.trail.length > 4000) state.trail.shift();
    }
    setAngle($("pitch"), -pose.pitch, 20);
    setAngle($("roll"), pose.roll, 20);
    $("alt-z").textContent = `${pose.z.toFixed(2)} m`;
    state.yaws.push([asked, pose.yaw_rate]);
    if (state.yaws.length > HISTORY) state.yaws.shift();
  }
  const headingYaw = state.view === "fly" ? (m.fly ? m.fly.yaw : null) : pose ? pose.yaw : null;
  $("heading").textContent = state.view === "map" || headingYaw == null ? "" : compass(headingYaw);
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
  showDrivetrain(m);
  showReferee(m.score, m.radio);
  showPictureState(m, Boolean(stats));
  updateViewName();
  draw();
  drawMapView();
}

function setAngle(el, radians, warnDegrees, digits = 1) {
  el.textContent = `${signed(radians * DEG, digits)}°`;
  el.dataset.tone = Math.abs(radians * DEG) > warnDegrees ? "warn" : "";
}

// Commanded against achieved turn rate, and per wheel the motor current
// against its limit, the torque, the slip, the dig-in and the ground (design
// spec 9.3). A DiffDrive rover has no motor model, so only the turn shows.
function showDrivetrain(m) {
  const d = m.drivetrain;
  const asked = askedTurn(m);
  const got = m.pose ? m.pose.yaw_rate : null;
  $("yaw-asked").textContent = `${signed(asked * DEG, 0)}°/s`;
  $("yaw-got").textContent = got == null ? "–" : `${signed(got * DEG, 0)}°/s`;
  $("yaw-ratio").textContent = got != null && Math.abs(asked) > TURNING ? `${Math.round(100 * got / asked)} %` : "–";
  $("motor-table").hidden = !d;
  $("motors-figure").hidden = !d;
  $("drivetrain-note").textContent = d ? ""
    : "No motor data: this rover drives with Gazebo's DiffDrive (DriveParams.mode = \"physical\" has the motor model).";
  if (!d) return;
  const limit = state.info ? state.info.drivetrain.current_limit : 20;
  const rows = WHEELS.map((name) => {
    const w = d.wheels[name];
    const tr = document.createElement("tr");
    if (!w) return tr;
    tr.dataset.sat = String(w.sat);
    const cell = (text, cls) => {
      const td = document.createElement("td");
      td.textContent = text;
      if (cls) td.className = cls;
      tr.append(td);
      return td;
    };
    cell(name.toUpperCase());
    const current = cell("", "current");
    current.innerHTML = `<span class="current-cell"><span class="current-bar"><i style="width:${
      Math.min(100, 100 * Math.abs(w.i) / limit).toFixed(0)}%"></i></span>${Math.abs(w.i).toFixed(1)} A</span>`;
    current.title = w.sat ? "At the current limit: the motor gives all it can" : `of ${limit} A`;
    cell(`${signed(w.tau, 1)} N·m`);
    cell(`${Math.abs(w.slip).toFixed(2)} m/s`);
    cell(w.dig.toFixed(2)).dataset.dug = String(w.dig > DUG);
    cell(w.surface ? w.surface.replace(/_/g, " ") : "in the air", "ground");
    return tr;
  });
  $("motor-rows").replaceChildren(...rows);
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

function showPictureState(m, running) {
  const fps = m.cameras ? m.cameras[state.view] : 0;
  const waiting = state.view !== "map" && !fps && performance.now() - state.viewSince > 2500;
  $("no-picture").hidden = !waiting;
  if (!waiting) return;
  const camera = {eye: "The rover eye", chase: "The chase camera", fly: "The fly camera"}[state.view]
    || "The onboard camera";
  $("no-picture-title").textContent = state.view === "fly" && !m.fly ? "The fly camera is starting" : "No picture yet";
  $("no-picture-text").textContent = !running
    ? "The simulation is not running. When it starts again, the station brings its cameras back."
    : state.view === "fly" && m.fly_note ? m.fly_note
      : state.view === "fly" && !m.fly ? "The station spawns it when Fly or Map is first used; it reads the terrain first."
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
  drawMotors();
  drawRocker();
  drawHistory($("rocker-history"), state.rockers.map(([a, b]) => [a * DEG, b * DEG]), [C.bone, C.sky], 2, "°");
  drawHistory($("yaw-history"), state.yaws.map(([a, b]) => [a * DEG, b * DEG]), [C.ochre, C.sky], 10, "°/s");
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

// Plan view, front up: each motor's current as a bar against the limit (the
// top), red at the limit; an ochre ring marks a wheel dug into loose ground.
function drawMotors() {
  const canvas = $("motors");
  const d = state.tm && state.tm.drivetrain;
  if (!d || canvas.offsetParent === null) return;
  const [ctx, w, h] = fit(canvas);
  const limit = state.info ? state.info.drivetrain.current_limit : 20;
  const spots = {fl: [0.3, 0], fr: [0.7, 0], rl: [0.3, 1], rr: [0.7, 1]};
  const bar = (h - 6) / 2;
  for (const [name, [fx, row]] of Object.entries(spots)) {
    const wheel = d.wheels[name];
    if (!wheel) continue;
    const x = fx * w + (fx < 0.5 ? -10 : 10);
    const top = 2 + row * (bar + 2);
    ctx.strokeStyle = C.dust;
    ctx.lineWidth = 1;
    ctx.strokeRect(x - 5.5, top + 0.5, 11, bar - 1);
    const fill = Math.min(1, Math.abs(wheel.i) / limit) * (bar - 3);
    ctx.fillStyle = wheel.sat ? C.signal : C.sky;
    ctx.fillRect(x - 4, top + bar - 1.5 - fill, 8, fill);
    if (wheel.dig > DUG) {
      ctx.strokeStyle = C.ochre;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x + (fx < 0.5 ? -11 : 11), top + bar / 2, 3, 0, 2 * Math.PI);
      ctx.stroke();
    }
  }
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.strokeRect(w * 0.42, 3, w * 0.16, h - 6);  // the chassis
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

// Two series over the last 30 s (data: [a, b] per sample, in `unit`); the
// scale grows with the values, from ±floor.
function drawHistory(canvas, data, strokes, floor, unit) {
  const [ctx, w, h] = fit(canvas);
  const peak = Math.max(floor, ...data.map(([a, b]) => Math.max(Math.abs(a), Math.abs(b)) * 1.2));
  const y = (value) => h / 2 - (value / peak) * (h / 2 - 6);
  ctx.strokeStyle = C.rule;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
  ctx.stroke();
  ctx.fillStyle = C.dust;
  ctx.font = `11px ${css.getPropertyValue("--text")}`;
  ctx.fillText(`±${peak.toFixed(peak < 10 ? 1 : 0)}${unit}`, 2, 11);
  ctx.fillText("30 s", w - 26, h - 3);
  strokes.forEach((stroke, k) => {
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

// --- Maps: the rail's and the map view, north up, world x east and y north ----------

// The Photo / Relief switches, on the rail's map and in the map view, once there is a photo.
function showLayer() {
  for (const button of document.querySelectorAll(".layer")) {
    button.setAttribute("aria-pressed", String(button.dataset.layer === state.mapLayer));
  }
  $("rail-layers").hidden = !state.images.photo;
  $("map-layers").hidden = state.view !== "map" || !state.images.photo;
}

function terrainSize() {
  const map = state.info && state.info.map;
  return map ? Math.max(...map.size) : null;
}

function wholeMap() {
  const size = terrainSize();
  const pose = state.tm && state.tm.pose;
  return size ? {cx: 0, cy: 0, span: size} : {cx: pose ? pose.x : 0, cy: pose ? pose.y : 0, span: 200};
}

function mapSpans() {
  const size = terrainSize();
  if (!size) return [40, 12, 150];
  return [size, ...[250, 60].filter((span) => span < size)];
}

function cycleMap() {
  state.mapLevel = (state.mapLevel + 1) % mapSpans().length;
  updateMapZoomLabel();
  drawMap();
}

function updateMapZoomLabel() {
  const span = mapSpans()[state.mapLevel];
  const whole = terrainSize() && state.mapLevel === 0;
  $("map-zoom").textContent = whole ? "Whole terrain" : `${span} m around the rover`;
}

// World (x, y) under a pointer event in the map view.
function mapWorld(event) {
  const canvas = $("map-view");
  const box = canvas.getBoundingClientRect();
  const v = state.mapView;
  const k = v.span / canvas.clientWidth;
  return [v.cx + (event.clientX - box.left - canvas.clientWidth / 2) * k,
    v.cy - (event.clientY - box.top - canvas.clientHeight / 2) * k];
}

// Zoom the map view by `factor` (> 1 out), keeping the point under the pointer (or the centre) in place.
function zoomMap(factor, event) {
  const v = state.mapView;
  const size = terrainSize() || 200;
  const span = Math.min(MAP_SPAN[1] * size, Math.max(MAP_SPAN[0], v.span * factor));
  if (event) {
    const [x, y] = mapWorld(event);
    v.cx = x + (v.cx - x) * span / v.span;
    v.cy = y + (v.cy - y) * span / v.span;
  }
  v.span = span;
  drawMapView();
}

function centreMap() {
  const pose = state.tm && state.tm.pose;
  if (pose) Object.assign(state.mapView, {cx: pose.x, cy: pose.y});
  drawMapView();
}

function niceStep(span) {
  const raw = span / 5;
  const power = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((k) => k * power).find((step) => step >= raw);
}

// The ground the fly camera's picture covers: its corners projected onto the
// level of the ground under it (in perspective, a ray above the horizon is
// cut at `reach` m).
function flyFootprint(f, reach) {
  const hfov = state.info.hfov.fly;
  const aspect = state.info.fly.aspect;
  const [cy, sy, cp, sp] = [Math.cos(f.yaw), Math.sin(f.yaw), Math.cos(f.pitch), Math.sin(f.pitch)];
  const turn = ([x, y, z]) => [cy * (cp * x + sp * z) - sy * y, sy * (cp * x + sp * z) + cy * y, -sp * x + cp * z];
  const t = Math.tan(hfov / 2);
  return [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => {  // across (right +), down +
    if (f.ortho > 0) {
      const [dx, dy] = turn([0, -a * f.ortho / 2, -b * f.ortho / aspect / 2]);
      return [f.x + dx, f.y + dy];
    }
    const [dx, dy, dz] = turn([1, -a * t, -b * t / aspect]);
    const s = dz < -1e-6 ? Math.min((f.z - f.ground) / -dz, reach) : reach;
    return [f.x + s * dx, f.y + s * dy];
  });
}

// One map: background (photo, relief or a grid), places, the rover's track,
// the rover and the fly camera with its footprint; view: {cx, cy, span}.
function paintMap(canvas, v, big) {
  const [ctx, w, h] = fit(canvas);
  const map = state.info && state.info.map;
  const m = state.tm || {};
  const k = w / v.span;
  const px = (x) => w / 2 + (x - v.cx) * k;
  const py = (y) => h / 2 - (y - v.cy) * k;
  const font = css.getPropertyValue("--text");

  ctx.fillStyle = C.dusk;
  ctx.fillRect(0, 0, w, h);
  const image = map && (state.images[state.mapLayer] || state.images.relief);
  if (image) {
    const [sx, sy] = map.size;
    ctx.imageSmoothingQuality = "high";
    ctx.drawImage(image, px(-sx / 2), py(sy / 2), sx * k, sy * k);
  } else {
    const step = niceStep(v.span);
    ctx.strokeStyle = C.rule;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil((v.cx - v.span) / step) * step; x <= v.cx + v.span; x += step) {
      ctx.moveTo(px(x), 0); ctx.lineTo(px(x), h);
    }
    for (let y = Math.ceil((v.cy - v.span) / step) * step; y <= v.cy + v.span; y += step) {
      ctx.moveTo(0, py(y)); ctx.lineTo(w, py(y));
    }
    ctx.stroke();
  }

  if (map) {
    ctx.font = `${big ? 13 : 12}px ${font}`;
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
  const f = m.fly;
  if (f && state.info) {
    const corners = flyFootprint(f, Math.max(4 * (f.z - f.ground), v.span));
    ctx.fillStyle = C.sage;
    ctx.strokeStyle = C.sage;
    ctx.globalAlpha = 0.18;
    ctx.beginPath();
    corners.forEach(([x, y], i) => (i ? ctx.lineTo(px(x), py(y)) : ctx.moveTo(px(x), py(y))));
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 0.9;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.globalAlpha = 1;
    // The camera: a lens-forward body pointing where it looks.
    ctx.save();
    ctx.translate(px(f.x), py(f.y));
    ctx.rotate(-f.yaw);
    ctx.fillStyle = C.sage;
    ctx.strokeStyle = C.dusk;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.rect(-7, -5, 10, 10);
    ctx.moveTo(3, -2); ctx.lineTo(9, -6); ctx.lineTo(9, 6); ctx.lineTo(3, 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
  if (m.pose) {
    const pose = m.pose;
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
  ctx.fillStyle = C.bone;
  ctx.font = `600 12px ${font}`;
  ctx.fillText("N", w - 14, big ? 64 : 16);
  // Scale bar, bottom left.
  const step = niceStep(v.span);
  const y0 = h - (big ? 112 : 10);
  ctx.strokeStyle = C.dusk;
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(10, y0); ctx.lineTo(10 + step * k, y0);
  ctx.stroke();
  ctx.strokeStyle = C.bone;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.font = `11px ${font}`;
  ctx.lineWidth = 3;
  const label = step >= 1000 ? `${step / 1000} km` : `${step} m`;
  ctx.strokeStyle = C.dusk;
  ctx.strokeText(label, 14 + step * k, y0 + 4);
  ctx.fillText(label, 14 + step * k, y0 + 4);
}

// The rail's map: the whole terrain, or 250 / 60 m around the rover (M).
function drawMap() {
  const pose = state.tm && state.tm.pose;
  const span = mapSpans()[state.mapLevel];
  const centred = !(terrainSize() && state.mapLevel === 0) && pose;
  paintMap($("map"), {cx: centred ? pose.x : 0, cy: centred ? pose.y : 0, span}, false);
}

function drawMapView() {
  if (state.view === "map" && state.mapView) paintMap($("map-view"), state.mapView, true);
}

// --- Start --------------------------------------------------------------------------

loadInfo();
connect();
setView("eye");
setInterval(tick, TICK);
window.addEventListener("resize", () => { draw(); drawMapView(); });
window.addEventListener("focus", () => document.body.classList.remove("unfocused"));

/* Aero drag tool -- run explorer.
 *
 * Every tab is one run: a shape, the parameters it was given, and the results
 * that came back. A solved run never changes, so computing inside one forks a
 * new tab carrying your edits instead of overwriting what you were reading.
 *
 * The browser holds no physics: it mirrors the server's scene, applies the
 * same placement transform for display, and renders whatever the solvers
 * returned. Everything numeric on screen came from Python.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/* --------------------------------------------------------------- config */

// Colour follows the solver, never its position in the list. A run with only
// SU2 in it still draws SU2 orange.
const SOLVERS = {
  openfoam: { label: 'OpenFOAM', varName: '--series-openfoam' },
  su2: { label: 'SU2', varName: '--series-su2' },
};
const SOLVER_ORDER = ['openfoam', 'su2'];

const CONTROL_SPECS = {
  wind: [
    { key: 'speed', label: 'Speed', unit: 'm/s', min: 0, max: 60, step: 0.5, decimals: 2 },
    { key: 'azimuth_deg', label: 'Azimuth', unit: '°', min: -180, max: 180, step: 1, decimals: 1 },
    { key: 'elevation_deg', label: 'Elevation', unit: '°', min: -60, max: 60, step: 1, decimals: 1 },
  ],
  orientation: [
    { key: 'yaw_deg', label: 'Yaw', unit: '°', min: -180, max: 180, step: 1, decimals: 1 },
    { key: 'pitch_deg', label: 'Pitch', unit: '°', min: -90, max: 90, step: 1, decimals: 1 },
    { key: 'roll_deg', label: 'Roll', unit: '°', min: -180, max: 180, step: 1, decimals: 1 },
  ],
  road: [
    { key: 'enabled', label: 'Road present', type: 'check' },
    { key: 'ride_height', label: 'Ride height', unit: 'm', min: 0, max: 3, step: 0.005, decimals: 3 },
    { key: 'moving', label: 'Road moves with the flow', type: 'check' },
  ],
  fluid: [
    { key: 'density', label: 'Density', unit: 'kg/m³', min: 0.4, max: 2.0, step: 0.001, decimals: 4 },
    { key: 'viscosity', label: 'Viscosity', unit: 'Pa·s', type: 'number', expo: true },
  ],
  packaging: [
    { key: 'clearance', label: 'Payload clearance', unit: 'm', min: 0.005, max: 0.3, step: 0.005, decimals: 3 },
    { key: 'anisotropy', label: 'Streamwise bias', min: 1, max: 6, step: 0.1, decimals: 1 },
    { key: 'resolution', label: 'Voxel resolution', min: 48, max: 220, step: 4, decimals: 0 },
    { key: 'streamline', label: 'Streamlined envelope', type: 'check' },
    { key: 'nose_angle_deg', label: 'Nose angle', unit: '°', min: 10, max: 80, step: 1, decimals: 0 },
    { key: 'tail_angle_deg', label: 'Tail angle', unit: '°', min: 5, max: 45, step: 1, decimals: 0 },
  ],
  solver: [
    { key: 'reference_speed', label: 'Reference speed', unit: 'm/s', min: 1, max: 60, step: 0.5, decimals: 2 },
    { key: 'speed_min', label: 'Curve from', unit: 'm/s', min: 0, max: 60, step: 0.5, decimals: 2 },
    { key: 'speed_max', label: 'Curve to', unit: 'm/s', min: 0, max: 60, step: 0.5, decimals: 2 },
    { key: 'speed_points', label: 'Curve points', min: 2, max: 25, step: 1, decimals: 0 },
    {
      key: 'sweep_mode', label: 'Speed handling', type: 'select',
      options: [['auto', 'Automatic'], ['scale', 'One run, scaled'], ['sweep', 'Solve every speed']],
    },
    {
      key: 'turbulence', label: 'Turbulence', type: 'select',
      options: [['kOmegaSST', 'k-ω SST'], ['laminar', 'Laminar']],
    },
    { key: 'iterations', label: 'Iterations', min: 50, max: 3000, step: 50, decimals: 0 },
    { key: 'mesh_resolution', label: 'Mesh resolution', min: 10, max: 120, step: 2, decimals: 0 },
  ],
};

/* ---------------------------------------------------------------- state */

const state = {
  runs: [],           // tab-bar summaries
  activeId: null,
  run: null,          // the full payload of the run on screen
  solvers: [],
  cores: null,
  runningId: null,    // which run is solving, whichever tab you are looking at
  runningJob: null,
  job: null,
  jobTimer: null,
  resultView: 'chart',
  chartHover: null,
  meshKey: null,      // what the viewport currently holds
};

const $ = (id) => document.getElementById(id);

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function seriesColor(solverName) {
  const meta = SOLVERS[solverName];
  return meta ? cssVar(meta.varName) : cssVar('--text-muted');
}

function solverLabel(name) {
  return SOLVERS[name] ? SOLVERS[name].label : name;
}

/* ------------------------------------------------------------------ api */

async function api(path, options = {}) {
  const response = await fetch(path, options);
  if (!response.ok) {
    let detail = `${response.status} ${response.statusText}`;
    try {
      const body = await response.json();
      if (body && body.detail) detail = body.detail;
    } catch (error) { /* keep the status line */ }
    throw new Error(detail);
  }
  if (response.status === 204) return null;
  const type = response.headers.get('content-type') || '';
  if (type.includes('application/json')) return response.json();
  return response;
}

function toast(message, isBad = false) {
  const element = $('toast');
  element.textContent = message;
  element.classList.toggle('is-bad', isBad);
  element.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { element.hidden = true; }, isBad ? 7000 : 3600);
}

/* ------------------------------------------------------------ formatting */

function fmt(value, decimals = 3) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—';
  const magnitude = Math.abs(value);
  if (magnitude !== 0 && (magnitude < 1e-3 || magnitude >= 1e6)) return value.toExponential(2);
  return value.toFixed(decimals).replace(/\.?0+$/, (match) => (match.includes('.') ? '' : match));
}

/* Table columns keep a fixed decimal count so the digits line up; fmt()'s
   trailing-zero trim is for tiles, where "1 m²" reads better than "1.000". */
function fixed(value, decimals = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—';
  return value.toFixed(decimals);
}

function fmtSi(value, digits = 3) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—';
  const magnitude = Math.abs(value);
  if (magnitude >= 1e6) return `${(value / 1e6).toFixed(2)}M`;
  if (magnitude >= 1e3) return `${(value / 1e3).toFixed(2)}k`;
  return value.toPrecision(digits).replace(/\.?0+$/, (match) => (match.includes('.') ? '' : match));
}

function formatDuration(seconds) {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return '—';
  if (seconds < 90) return `${Math.round(seconds)} s`;
  const minutes = seconds / 60;
  if (minutes < 90) return `${Math.round(minutes)} min`;
  const hours = Math.floor(minutes / 60);
  const rest = Math.round(minutes % 60);
  return rest ? `${hours} h ${String(rest).padStart(2, '0')} min` : `${hours} h`;
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

/* ------------------------------------------------------------- viewport */

const viewport = (() => {
  const canvas = $('view');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();

  // Physics is Z-up, so the view is too. No axis remapping anywhere.
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 5000);
  camera.up.set(0, 0, 1);
  camera.position.set(4, -5, 2.6);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x404050, 1.5));
  const key = new THREE.DirectionalLight(0xffffff, 1.9);
  key.position.set(3, -4, 6);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.5);
  fill.position.set(-4, 3, 1);
  scene.add(fill);

  const hullGroup = new THREE.Group();
  scene.add(hullGroup);

  let hullMesh = null;
  let hullEdges = null;
  let payloadMesh = null;
  let hullTransparent = false;
  const roadGroup = new THREE.Group();
  scene.add(roadGroup);
  const windGroup = new THREE.Group();
  scene.add(windGroup);
  const dropGroup = new THREE.Group();
  scene.add(dropGroup);

  let modelScale = 1;
  let framed = false;

  function clear(group) {
    while (group.children.length) {
      const child = group.children.pop();
      child.geometry?.dispose?.();
      child.material?.dispose?.();
    }
  }

  function decode(buffer) {
    const view = new DataView(buffer);
    const triangles = view.getUint32(0, true);
    return {
      positions: new Float32Array(buffer, 4, triangles * 9),
      normals: new Float32Array(buffer, 4 + triangles * 9 * 4, triangles * 9),
    };
  }

  /* The payload is a child of the hull group, so it inherits exactly the same
     placement transform. Anything else risks drawing it somewhere it is not,
     which would make "does it fit" a lie. */
  function setPayload(buffer) {
    if (payloadMesh) {
      hullGroup.remove(payloadMesh);
      payloadMesh.geometry.dispose();
      payloadMesh.material.dispose();
      payloadMesh = null;
    }
    if (!buffer) return;

    const { positions, normals } = decode(buffer);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));

    payloadMesh = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: new THREE.Color(cssVar('--payload')),
        metalness: 0.05,
        roughness: 0.75,
        transparent: true,
        opacity: 0.95,
      }),
    );
    hullGroup.add(payloadMesh);
  }

  /* Remembered rather than applied once: the caller sets this while adopting a
     run, which happens before the new mesh arrives, so setHull has to re-apply
     it to the fresh material. */
  function setHullTransparent(transparent) {
    hullTransparent = Boolean(transparent);
    applyHullTransparency();
  }

  function applyHullTransparency() {
    if (!hullMesh) return;
    hullMesh.material.transparent = hullTransparent;
    hullMesh.material.opacity = hullTransparent ? 0.3 : 1.0;
    hullMesh.material.depthWrite = !hullTransparent;
    hullMesh.material.needsUpdate = true;
    if (hullEdges) hullEdges.material.opacity = hullTransparent ? 0.28 : 0.18;
  }

  function setHull(buffer) {
    const { positions, normals } = decode(buffer);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    geometry.computeBoundingSphere();
    geometry.computeBoundingBox();

    // clear() drops every child, the payload included, so forget it too --
    // loadPayloadMesh re-adds it right after.
    clear(hullGroup);
    payloadMesh = null;
    hullMesh = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: new THREE.Color(cssVar('--hull')),
        metalness: 0.12,
        roughness: 0.55,
        side: THREE.DoubleSide,
      }),
    );
    hullGroup.add(hullMesh);

    hullEdges = new THREE.LineSegments(
      new THREE.EdgesGeometry(geometry, 28),
      new THREE.LineBasicMaterial({
        color: new THREE.Color(cssVar('--text-primary')), transparent: true, opacity: 0.18,
      }),
    );
    hullGroup.add(hullEdges);
    applyHullTransparency();

    modelScale = Math.max(geometry.boundingSphere.radius * 2, 0.05);
    framed = false;
    $('view-empty').hidden = true;
  }

  function clearAll() {
    clear(hullGroup);
    clear(roadGroup);
    clear(windGroup);
    clear(dropGroup);
    hullMesh = null;
    hullEdges = null;
    payloadMesh = null;
    $('view-empty').hidden = false;
  }

  function buildRoad(size) {
    clear(roadGroup);
    const plane = new THREE.Mesh(
      new THREE.PlaneGeometry(size, size),
      new THREE.MeshStandardMaterial({
        color: new THREE.Color(cssVar('--road')),
        roughness: 0.95,
        metalness: 0,
        transparent: true,
        opacity: 0.16,
        side: THREE.DoubleSide,
      }),
    );
    plane.position.z = -0.0015 * size; // sits just under the grid to avoid z-fighting
    roadGroup.add(plane);

    // One grid square per hull length, so the road reads as a scale reference.
    const divisions = Math.max(Math.round(size / modelScale) * 2, 8);
    const grid = new THREE.GridHelper(size, divisions, cssVar('--axis'), cssVar('--grid'));
    grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default; we need XY
    grid.material.transparent = true;
    grid.material.opacity = 0.75;
    roadGroup.add(grid);
  }

  function buildWind(direction, hullCentre) {
    clear(windGroup);
    const length = modelScale * 0.85;
    const dir = new THREE.Vector3(direction[0], direction[1], direction[2]).normalize();
    // Start upstream of the hull so the arrow reads as flow arriving at it.
    const origin = hullCentre.clone().addScaledVector(dir, -(modelScale * 0.6 + length));
    const arrow = new THREE.ArrowHelper(
      dir, origin, length, new THREE.Color(cssVar('--wind')),
      length * 0.26, length * 0.13,
    );
    arrow.line.material.linewidth = 2;
    windGroup.add(arrow);

    // Two faint outriggers make the flow direction readable from any angle.
    const side = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0, 0, 1));
    if (side.lengthSq() < 1e-6) side.set(0, 1, 0);
    side.normalize().multiplyScalar(modelScale * 0.34);
    for (const offset of [side, side.clone().negate()]) {
      const ghost = new THREE.ArrowHelper(
        dir, origin.clone().add(offset), length * 0.72, new THREE.Color(cssVar('--wind')),
        length * 0.2, length * 0.1,
      );
      ghost.line.material.transparent = true;
      ghost.line.material.opacity = 0.4;
      ghost.cone.material.transparent = true;
      ghost.cone.material.opacity = 0.4;
      windGroup.add(ghost);
    }
  }

  function buildDrop(box, rideHeight) {
    clear(dropGroup);
    if (rideHeight === null) return;
    const x = (box.min.x + box.max.x) / 2;
    const y = box.min.y;
    const geometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(x, y, 0),
      new THREE.Vector3(x, y, box.min.z),
    ]);
    dropGroup.add(new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({ color: new THREE.Color(cssVar('--text-muted')) }),
    ));
  }

  /* Mirrors Scene.placed_mesh(): rotate about the centroid, then lift so the
     lowest point sits at the ride height. The server centred the mesh for us. */
  function place(scene_) {
    if (!hullMesh) return null;
    const orientation = scene_.orientation;
    const toRad = Math.PI / 180;
    hullGroup.rotation.set(
      orientation.roll_deg * toRad,
      orientation.pitch_deg * toRad,
      orientation.yaw_deg * toRad,
      'ZYX',
    );
    hullGroup.position.set(0, 0, 0);
    hullGroup.updateMatrixWorld(true);

    const box = new THREE.Box3().setFromObject(hullGroup);
    const road = scene_.road;
    if (road.enabled) {
      hullGroup.position.z = road.ride_height - box.min.z;
      hullGroup.updateMatrixWorld(true);
    }
    return new THREE.Box3().setFromObject(hullGroup);
  }

  function update(scene_) {
    if (!hullMesh || !scene_) return;
    const box = place(scene_);
    const centre = box.getCenter(new THREE.Vector3());
    const road = scene_.road;

    roadGroup.visible = road.enabled;
    if (road.enabled) buildRoad(modelScale * 6);
    buildDrop(box, road.enabled ? road.ride_height : null);

    const vector = scene_.wind.vector;
    const magnitude = Math.hypot(vector[0], vector[1], vector[2]);
    buildWind(magnitude > 1e-9 ? vector : [1, 0, 0], centre);

    if (!framed) {
      // Fit the hull *and* its upstream wind arrow, but keep the orbit centre
      // on the hull so the body stays put as you turn around it.
      const fit = box.clone().union(new THREE.Box3().setFromObject(windGroup));
      frame(fit, centre);
      framed = true;
    }
  }

  function frame(box, target) {
    const centre = target || box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 0.05);
    const distance = radius / Math.sin((camera.fov * Math.PI) / 360) * 1.3;
    controls.target.copy(centre);
    camera.position.copy(centre).add(
      new THREE.Vector3(0.75, -1, 0.42).normalize().multiplyScalar(distance),
    );
    camera.near = Math.max(distance / 5000, 0.001);
    camera.far = distance * 40;
    camera.updateProjectionMatrix();
    controls.update();
  }

  function resize() {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (width === 0 || height === 0) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }

  function tick() {
    resize();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(tick);
  }

  window.addEventListener('resize', resize);
  tick();

  return {
    setHull, setPayload, setHullTransparent, update, clearAll,
    refit: () => { framed = false; },
  };
})();

/* -------------------------------------------------------------- controls */

function buildControl(section, spec) {
  const row = el('div', 'control');

  if (spec.type === 'check') {
    row.classList.add('control-check');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.id = `ctl-${section}-${spec.key}`;
    const label = el('label', 'control-label', spec.label);
    label.htmlFor = input.id;
    row.append(input, label);
    input.addEventListener('change', () => pushControl(section, spec.key, input.checked));
    return { row, apply: (value) => { input.checked = Boolean(value); }, disable: (off) => { input.disabled = off; } };
  }

  const label = el('label', 'control-label', spec.unit ? `${spec.label} (${spec.unit})` : spec.label);
  row.append(label);

  if (spec.type === 'select') {
    const select = el('select', 'control-select');
    for (const [value, text] of spec.options) {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = text;
      select.append(option);
    }
    row.append(select);
    label.htmlFor = select.id = `ctl-${section}-${spec.key}`;
    select.addEventListener('change', () => pushControl(section, spec.key, select.value));
    return { row, apply: (value) => { select.value = value; }, disable: (off) => { select.disabled = off; } };
  }

  const number = document.createElement('input');
  number.type = 'number';
  number.className = 'control-value';
  number.id = `ctl-${section}-${spec.key}`;
  label.htmlFor = number.id;
  if (spec.step !== undefined) number.step = spec.step;
  row.append(number);

  let slider = null;
  if (spec.type !== 'number' && spec.min !== undefined) {
    slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'control-slider';
    slider.min = spec.min;
    slider.max = spec.max;
    slider.step = spec.step;
    slider.setAttribute('aria-label', spec.label);
    row.append(slider);

    slider.addEventListener('input', () => {
      number.value = Number(slider.value).toFixed(spec.decimals ?? 3);
      pushControl(section, spec.key, Number(slider.value), true);
    });
  }

  number.addEventListener('change', () => {
    const value = Number(number.value);
    if (Number.isNaN(value)) return;
    if (slider) slider.value = value;
    pushControl(section, spec.key, value);
  });

  return {
    row,
    apply: (value) => {
      if (document.activeElement === number) return;
      // Very small quantities such as viscosity are unreadable as decimals.
      number.value = spec.expo
        ? Number(value).toExponential(3)
        : Number(value).toFixed(spec.decimals ?? 3);
      if (slider) slider.value = value;
    },
    disable: (off) => { number.disabled = off; if (slider) slider.disabled = off; },
  };
}

const controlRegistry = {};

function buildControls() {
  for (const [section, specs] of Object.entries(CONTROL_SPECS)) {
    const host = $(`controls-${section}`);
    if (!host) continue;
    controlRegistry[section] = {};
    for (const spec of specs) {
      const control = buildControl(section, spec);
      controlRegistry[section][spec.key] = control;
      host.append(control.row);
    }
  }
}

function applyControls(scene) {
  for (const [section, controls] of Object.entries(controlRegistry)) {
    const values = scene[section] || {};
    for (const [key, control] of Object.entries(controls)) {
      if (values[key] !== undefined && values[key] !== null) control.apply(values[key]);
    }
  }
}

/* A parameter that has moved since this run was solved gets its as-run value
   printed underneath, with a way back. The results stay on screen -- they are
   still a real measurement -- but never without saying what they belong to. */
function markChangedControls(changed) {
  for (const controls of Object.values(controlRegistry)) {
    for (const control of Object.values(controls)) {
      control.row.classList.remove('is-changed');
      control.row.querySelector('.was')?.remove();
    }
  }

  for (const change of changed || []) {
    const control = controlRegistry[change.section]?.[change.key];
    if (!control) continue;
    control.row.classList.add('is-changed');
    const was = el('div', 'was');
    was.append(document.createTextNode(`solved at ${change.as_run_text}`));
    const revert = el('button', null, 'revert');
    revert.type = 'button';
    revert.addEventListener('click', () => {
      control.apply(change.as_run);
      pushControl(change.section, change.key, change.as_run);
    });
    was.append(revert);
    control.row.append(was);
  }
}

function lockControls(locked) {
  for (const controls of Object.values(controlRegistry)) {
    for (const control of Object.values(controls)) control.disable(locked);
  }
  $('quality-select').disabled = locked;
  $('processes-input').disabled = locked;
  $('btn-reset-attitude').disabled = locked;
  for (const input of $('solver-list').querySelectorAll('input')) input.disabled = locked;
}

/* Local echo first so dragging a slider feels immediate, then a debounced
   patch to the server which owns the derived quantities. */
let pendingPatch = {};
let patchTimer = null;

function pushControl(section, key, value, live = false) {
  const run = state.run;
  if (!run || run.status === 'running') return;
  run.scene[section][key] = value;
  if (section === 'wind') {
    const speed = run.scene.wind.speed;
    const az = (run.scene.wind.azimuth_deg * Math.PI) / 180;
    const el_ = (run.scene.wind.elevation_deg * Math.PI) / 180;
    run.scene.wind.vector = [
      speed * Math.cos(el_) * Math.cos(az),
      speed * Math.cos(el_) * Math.sin(az),
      speed * Math.sin(el_),
    ];
  }
  viewport.update(run.scene);

  pendingPatch[section] = { ...(pendingPatch[section] || {}), [key]: value };
  clearTimeout(patchTimer);
  patchTimer = setTimeout(flushPatch, live ? 220 : 0);
}

async function flushPatch() {
  if (!Object.keys(pendingPatch).length || !state.run) return;
  const patch = pendingPatch;
  pendingPatch = {};
  try {
    const payload = await api(`/api/runs/${state.run.id}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(patch),
    });
    adoptRun(payload, { skipControls: true, keepCamera: true });
  } catch (error) {
    toast(`Update failed: ${error.message}`, true);
  }
}

/* --------------------------------------------------------------- tab bar */

function renderTabs() {
  const host = $('tabbar');
  host.textContent = '';

  for (const summary of state.runs) {
    const tab = el('button', 'tab');
    tab.type = 'button';
    if (summary.id === state.activeId) tab.classList.add('is-active');
    if (summary.kind === 'shape') tab.classList.add('is-shape');

    const glyph = el('span', 'glyph');
    if (summary.status === 'running') glyph.classList.add('glyph-running');
    else if (summary.status === 'failed') glyph.classList.add('glyph-failed');
    else if (summary.status === 'draft') glyph.classList.add('glyph-draft');
    else glyph.classList.add(summary.kind === 'shape' ? 'glyph-shape' : 'glyph-drag');

    tab.append(glyph, el('span', 'tab-label', summary.title));

    if (summary.changed_count) {
      const dot = el('i', 'changed-dot');
      dot.title = `${summary.changed_count} parameter(s) changed since this run was solved`;
      tab.append(dot);
    }

    const close = el('span', 'tab-close', '×');
    close.title = 'Close this run';
    close.addEventListener('click', async (event) => {
      event.stopPropagation();
      await closeRun(summary.id);
    });
    tab.append(close);

    // A solving run reports progress from every tab, not just its own.
    if (summary.status === 'running') {
      const bar = el('span', 'tab-progress');
      const fraction = state.job?.progress?.fraction;
      bar.style.width = `${Math.round((fraction ?? 0.04) * 100)}%`;
      tab.append(bar);
    }

    tab.addEventListener('click', () => selectRun(summary.id));
    host.append(tab);
  }

  const add = el('button', 'tab-add', '+');
  add.type = 'button';
  add.title = 'Import an STL as a new run';
  add.setAttribute('aria-label', 'New run');
  add.addEventListener('click', () => $('file-stl').click());
  host.append(add);

  const note = el('div', 'tabbar-note');
  const busy = state.runs.find((item) => item.status === 'running');
  if (busy) {
    note.append(el('span', 'glyph glyph-running'));
    const remaining = state.job?.progress?.remaining_text;
    note.append(document.createTextNode(
      `${busy.title} solving${remaining ? ` · ~${remaining} left` : ''}`,
    ));
  } else if (state.runs.length) {
    note.textContent = `${state.runs.length} run${state.runs.length === 1 ? '' : 's'} open`;
  }
  host.append(note);
}

/* --------------------------------------------------------------- solvers */

function renderSolverList() {
  const host = $('solver-list');
  host.textContent = '';
  const run = state.run;
  const selected = new Set(run ? run.scene.solver.backends : []);

  for (const name of SOLVER_ORDER) {
    const info = state.solvers.find((item) => item.name === name);
    if (!info) continue;

    const row = el('div', 'solver-row');
    if (!info.available) row.classList.add('is-off');

    const check = document.createElement('input');
    check.type = 'checkbox';
    check.id = `solver-${name}`;
    check.checked = selected.has(name);
    check.disabled = !run || run.status === 'running';

    const label = el('label', 'solver-name');
    label.htmlFor = check.id;
    label.append(el('i', `dot dot-${name}`), document.createTextNode(info.label));

    const pill = el('span', `pill ${info.available ? 'pill-ok' : 'pill-warn'}`,
      info.available ? 'ready' : 'not here');

    row.append(check, label, pill, el('div', 'solver-detail', info.detail));
    host.append(row);

    check.addEventListener('change', () => {
      const backends = SOLVER_ORDER.filter((item) => $(`solver-${item}`)?.checked);
      state.run.scene.solver.backends = backends;
      pendingPatch.solver = { ...(pendingPatch.solver || {}), backends };
      flushPatch();
      renderActions();
    });
  }
}

function renderProcesses() {
  const input = $('processes-input');
  const hint = $('processes-hint');
  const cores = state.cores;
  if (!cores) return;
  const pinned = state.run?.scene.solver.processes;
  if (document.activeElement !== input) input.value = pinned ?? '';
  input.max = cores.available;
  input.placeholder = `auto (${cores.default_processes})`;
  hint.textContent = pinned
    ? `Pinned to ${pinned} of ${cores.available} cores.`
    : `Blank uses ${cores.default_processes} ranks, 80% of the ${cores.available} cores here.`;
}

/* -------------------------------------------------------------- geometry */

function tile(label, value, unit, sub, accent) {
  const element = el('div', 'tile');

  const head = el('div', 'tile-label');
  if (accent) {
    const dot = el('i', 'dot');
    dot.style.background = accent;
    head.append(dot);
  }
  head.append(document.createTextNode(label));

  const body = el('div', 'tile-value');
  body.append(document.createTextNode(value));
  if (unit) body.append(el('span', 'tile-unit', unit));

  element.append(head, body);
  if (sub) element.append(el('div', 'tile-sub', sub));
  return element;
}

function renderGeometry() {
  const host = $('geometry-tiles');
  host.textContent = '';
  const metrics = state.run?.metrics;
  $('geometry-block').hidden = !metrics;
  if (!metrics) return;

  host.append(tile('Frontal area', fmt(metrics.frontal_area, 4), 'm²', 'true silhouette at this wind angle'));
  host.append(tile('Ref length', fmt(metrics.streamwise_length, 4), 'm', 'extent along the flow'));
  host.append(tile('Wetted area', fmt(metrics.wetted_area, 3), 'm²'));
  host.append(tile('Volume', metrics.watertight ? fmt(metrics.volume, 4) : '—', 'm³',
    metrics.watertight ? null : 'not watertight'));

  const extents = metrics.extents.map((value) => fmt(value, 2)).join(' × ');
  const box = tile('Bounding box', extents, 'm', `${metrics.triangle_count} triangles`);
  box.classList.add('tile-span');
  host.append(box);
}

function renderShapeFacts() {
  const run = state.run;
  const host = $('shape-facts');
  host.textContent = '';
  $('shape-block').hidden = !run;
  if (!run) return;

  $('shape-title').textContent = run.kind === 'shape' ? 'Payload' : 'Shape';

  const rows = [['File', run.scene.geometry.source_name]];
  if (run.metrics) {
    rows.push(['Frontal area', `${fmt(run.metrics.frontal_area, 4)} m²`]);
    rows.push(['Triangles', String(run.metrics.triangle_count)]);
  }
  if (run.scene.fairing && run.kind !== 'shape') {
    rows.push(['Closing radius', `${(run.scene.fairing.closing_radius * 1000).toFixed(0)} mm`]);
    rows.push(['Clearance', `${(run.scene.fairing.clearance * 1000).toFixed(0)} mm`]);
  }

  for (const [key, value] of rows) {
    const row = el('div', 'fact');
    row.append(el('dt', null, key), el('dd', null, value));
    host.append(row);
  }
}

function renderReynoldsNote() {
  const note = $('reynolds-note');
  const advice = state.run?.reynolds;
  if (!advice || state.run?.kind === 'shape') { note.hidden = true; return; }

  note.textContent = '';
  note.className = `note ${advice.crosses_critical_band || advice.ratio > 3 ? 'note-warn' : 'note-good'}`;

  const mode = state.run.resolved_mode === 'sweep' ? 'Solving every speed' : 'One run, curve scaled as V²';
  note.append(el('strong', null, `${mode}. `));
  note.append(document.createTextNode(advice.warnings.join(' ')));
  note.hidden = false;
}

/* ---------------------------------------------------------------- chart */

function chartSeries(results) {
  if (!results) return [];
  return results.runs
    .filter((run) => run.status === 'ok' && run.points.length)
    .map((run) => ({
      name: run.solver,
      label: solverLabel(run.solver),
      color: seriesColor(run.solver),
      mode: run.mode,
      points: run.points.slice().sort((a, b) => a.speed - b.speed),
    }));
}

function niceTicks(min, max, count) {
  if (!(max > min)) return [min];
  const raw = (max - min) / count;
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const normalized = raw / magnitude;
  const step = (normalized >= 5 ? 10 : normalized >= 2 ? 5 : normalized >= 1 ? 2 : 1) * magnitude;
  const ticks = [];
  for (let value = Math.ceil(min / step) * step; value <= max + step * 1e-6; value += step) {
    ticks.push(Math.abs(value) < step * 1e-9 ? 0 : value);
  }
  return ticks;
}

const chartLayouts = {};

/* One renderer, two plots. `spec.value` picks the quantity, so drag force and
   drag coefficient each get their own axes -- never a shared one. */
const CHART_SPECS = {
  force: {
    canvas: 'chart', wrap: 'chart-wrap', tooltip: 'chart-tooltip',
    value: (point) => point.drag_force, axis: 'drag force (N)',
    format: (v) => `${fixed(v, 2)} N`, tick: (v) => fmtSi(v),
  },
  cd: {
    canvas: 'cd-chart', wrap: 'cd-chart-wrap', tooltip: 'cd-chart-tooltip',
    value: (point) => point.drag_coefficient, axis: 'Cd',
    format: (v) => fixed(v, 4), tick: (v) => fixed(v, 2),
  },
};

function drawChart(key = 'force') {
  const spec = CHART_SPECS[key];
  const layout = chartLayouts[key] || (chartLayouts[key] = { series: [] });
  const canvas = $(spec.canvas);
  const wrap = $(spec.wrap);
  const width = wrap.clientWidth;
  const height = wrap.clientHeight;
  if (!width || !height) return;

  const ratio = Math.min(window.devicePixelRatio, 2);
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const series = chartSeries(state.run?.results);
  layout.series = series;
  if (!series.length) return;

  const surface = cssVar('--surface-1');
  const gridColor = cssVar('--grid');
  const axisColor = cssVar('--axis');
  const muted = cssVar('--text-muted');

  const padLeft = 52;
  const padRight = 66; // room for the direct labels at each line end
  const padTop = 10;
  const padBottom = 30;
  const plotWidth = width - padLeft - padRight;
  const plotHeight = height - padTop - padBottom;
  if (plotWidth < 40 || plotHeight < 40) return;

  const speeds = series.flatMap((item) => item.points.map((point) => point.speed));
  const values = series.flatMap((item) => item.points.map((point) => spec.value(point)));
  const xMin = Math.min(...speeds);
  const xMax = Math.max(...speeds);
  // Cd hovers well above zero, so a zero-based axis would flatten the very
  // variation this chart exists to show. Pad around the data instead.
  const dataMax = Math.max(...values);
  const dataMin = Math.min(...values);
  const span = (dataMax - dataMin) || Math.abs(dataMax) * 0.1 || 1;
  const yMax = key === 'force' ? (Math.max(dataMax, 0) * 1.08 || 1) : dataMax + span * 0.25;
  const yMin = key === 'force' ? Math.min(0, dataMin) : dataMin - span * 0.25;

  const xScale = (value) => padLeft + ((value - xMin) / (xMax - xMin || 1)) * plotWidth;
  const yScale = (value) => padTop + plotHeight - ((value - yMin) / (yMax - yMin || 1)) * plotHeight;
  Object.assign(layout, {
    left: padLeft, right: padLeft + plotWidth, top: padTop,
    bottom: padTop + plotHeight, xScale, yScale, xMin, xMax,
  });

  context.font = '10px system-ui, sans-serif';
  context.lineWidth = 1;

  // Horizontal grid + y ticks
  const yTicks = niceTicks(yMin, yMax, 5);
  context.strokeStyle = gridColor;
  context.fillStyle = muted;
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  for (const tick of yTicks) {
    const y = Math.round(yScale(tick)) + 0.5;
    context.beginPath();
    context.moveTo(padLeft, y);
    context.lineTo(padLeft + plotWidth, y);
    context.stroke();
    context.fillText(spec.tick(tick), padLeft - 8, y);
  }

  // X ticks
  const xTicks = niceTicks(xMin, xMax, 5);
  context.textAlign = 'center';
  context.textBaseline = 'top';
  for (const tick of xTicks) {
    const x = Math.round(xScale(tick)) + 0.5;
    context.fillText(fmt(tick, 1), x, padTop + plotHeight + 8);
  }

  // Axis rules
  context.strokeStyle = axisColor;
  context.beginPath();
  context.moveTo(padLeft + 0.5, padTop);
  context.lineTo(padLeft + 0.5, padTop + plotHeight + 0.5);
  context.lineTo(padLeft + plotWidth, padTop + plotHeight + 0.5);
  context.stroke();

  // Axis titles
  context.fillStyle = muted;
  context.textAlign = 'center';
  context.textBaseline = 'bottom';
  context.fillText('speed (m/s)', padLeft + plotWidth / 2, height - 1);
  context.save();
  context.translate(11, padTop + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.textBaseline = 'top';
  context.fillText(spec.axis, 0, 0);
  context.restore();

  // Series
  const labelSlots = [];
  for (const item of series) {
    context.strokeStyle = item.color;
    context.lineWidth = 2;
    context.lineJoin = 'round';
    context.beginPath();
    item.points.forEach((point, index) => {
      const x = xScale(point.speed);
      const y = yScale(spec.value(point));
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();

    // Solved points are filled; extrapolated ones are hollow. That keeps the
    // distinction off the colour channel, which already carries identity.
    for (const point of item.points) {
      const x = xScale(point.speed);
      const y = yScale(spec.value(point));
      context.beginPath();
      context.arc(x, y, 4, 0, Math.PI * 2);
      if (point.source === 'solved') {
        context.fillStyle = item.color;
        context.fill();
        context.lineWidth = 2;
        context.strokeStyle = surface;
        context.stroke();
      } else {
        context.fillStyle = surface;
        context.fill();
        context.lineWidth = 1.5;
        context.strokeStyle = item.color;
        context.stroke();
      }
    }

    const last = item.points[item.points.length - 1];
    let labelY = yScale(spec.value(last));
    while (labelSlots.some((slot) => Math.abs(slot - labelY) < 11)) labelY += 11;
    labelSlots.push(labelY);

    context.font = '600 10px system-ui, sans-serif';
    context.textAlign = 'left';
    context.textBaseline = 'middle';
    context.lineWidth = 3;
    context.strokeStyle = surface;
    context.strokeText(item.label, xScale(last.speed) + 7, labelY);
    context.fillStyle = item.color;
    context.fillText(item.label, xScale(last.speed) + 7, labelY);
    context.font = '10px system-ui, sans-serif';
  }

  // Hover crosshair
  if (state.chartHover !== null && series.length) {
    const x = xScale(state.chartHover);
    context.strokeStyle = axisColor;
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(Math.round(x) + 0.5, padTop);
    context.lineTo(Math.round(x) + 0.5, padTop + plotHeight);
    context.stroke();
  }
}

function drawCharts() {
  drawChart('force');
  drawChart('cd');
}

function renderChartLegend() {
  const host = $('chart-legend');
  host.textContent = '';
  const series = chartSeries(state.run?.results);
  if (series.length < 1) return;

  // Two or more series need a legend to carry identity. One does not -- the
  // solver tile below names it, and the endpoint is directly labelled.
  if (series.length > 1) {
    for (const item of series) {
      const key = el('span', 'legend-key');
      const line = el('i', 'legend-line');
      line.style.background = item.color;
      key.append(line, document.createTextNode(item.label));
      host.append(key);
    }
  }

  const anyScaled = series.some((item) => item.points.some((point) => point.source === 'scaled'));
  if (anyScaled) {
    const note = el('span', 'legend-key', 'hollow markers = scaled from one solve');
    note.style.color = cssVar('--text-muted');
    host.append(note);
  }
}

function setupChartHover(key = 'force') {
  const spec = CHART_SPECS[key];
  const wrap = $(spec.wrap);
  const tooltip = $(spec.tooltip);

  const hide = () => {
    state.chartHover = null;
    tooltip.hidden = true;
    drawCharts();
  };

  wrap.addEventListener('pointerleave', hide);
  wrap.addEventListener('pointermove', (event) => {
    const layout = chartLayouts[key];
    const series = layout?.series || [];
    if (!series.length || !layout.xScale) return;

    const rect = wrap.getBoundingClientRect();
    const x = event.clientX - rect.left;
    if (x < layout.left - 6 || x > layout.right + 6) { hide(); return; }

    // Snap to the nearest speed present in the data: the reader aims at a
    // speed, never at a 2px line.
    const all = [...new Set(series.flatMap((item) => item.points.map((point) => point.speed)))];
    let nearest = all[0];
    let best = Infinity;
    for (const speed of all) {
      const distance = Math.abs(layout.xScale(speed) - x);
      if (distance < best) { best = distance; nearest = speed; }
    }
    // Both charts share the crosshair, so the two readings always line up.
    state.chartHover = nearest;
    drawCharts();

    tooltip.textContent = '';
    tooltip.append(el('div', 'tooltip-head', `${fixed(nearest, 2)} m/s`));

    for (const item of series) {
      const point = item.points.reduce(
        (acc, candidate) => (Math.abs(candidate.speed - nearest) < Math.abs(acc.speed - nearest) ? candidate : acc),
        item.points[0],
      );
      const row = el('div', 'tooltip-row');

      const swatch = el('i', 'tooltip-key');
      swatch.style.background = item.color;

      row.append(swatch,
        el('span', 'tooltip-value', spec.format(spec.value(point))),
        el('span', 'tooltip-name', item.label));
      if (point.source === 'scaled') row.append(el('span', 'tooltip-tag', 'scaled'));
      tooltip.append(row);
    }

    tooltip.hidden = false;
    const width = tooltip.offsetWidth;
    const anchor = layout.xScale(nearest);
    const left = anchor + width + 16 < rect.width ? anchor + 12 : anchor - width - 12;
    tooltip.style.left = `${Math.max(4, left)}px`;
    tooltip.style.top = `${Math.max(4, event.clientY - rect.top - 16)}px`;
  });
}

/* ---------------------------------------------------------------- table */

function renderTable() {
  const host = $('table-wrap');
  host.textContent = '';
  const series = chartSeries(state.run?.results);
  if (!series.length) return;

  const speeds = [...new Set(series.flatMap((item) => item.points.map((point) => point.speed)))]
    .sort((a, b) => a - b);

  const table = el('table', 'data');
  table.append(el('caption', null,
    'Cd is listed per speed because it is not constant: it varies with Reynolds number. '
    + 'Values marked * were scaled from a single solve.'));

  const thead = document.createElement('thead');
  const headRow = document.createElement('tr');
  const columns = ['Speed (m/s)', 'Re'];
  for (const item of series) columns.push(`${item.label} Cd`, `${item.label} (N)`);
  for (const text of columns) {
    const th = el('th', null, text);
    th.scope = 'col';
    headRow.append(th);
  }
  thead.append(headRow);
  table.append(thead);

  const tbody = document.createElement('tbody');
  for (const speed of speeds) {
    const row = document.createElement('tr');
    const th = el('th', null, fixed(speed, 2));
    th.scope = 'row';
    row.append(th);

    const reference = series[0].points.find((point) => Math.abs(point.speed - speed) < 1e-9);
    row.append(el('td', null, reference ? fmtSi(reference.reynolds) : '—'));

    for (const item of series) {
      const point = item.points.find((candidate) => Math.abs(candidate.speed - speed) < 1e-9);
      const mark = point && point.source === 'scaled' ? ' *' : '';
      row.append(el('td', null, point ? `${fixed(point.drag_coefficient, 4)}${mark}` : '—'));
      row.append(el('td', null, point ? `${fixed(point.drag_force, 2)}${mark}` : '—'));
    }
    tbody.append(row);
  }
  table.append(tbody);
  host.append(table);
}

/* -------------------------------------------------------------- results */

function renderResultTiles() {
  const host = $('result-tiles');
  host.textContent = '';
  const run = state.run;
  if (!run || run.drag_area === null || run.drag_area === undefined) return;

  const dragAreaTile = tile('Drag area Cd·A', fixed(run.drag_area, 4), 'm²');
  const reference = run.reference;
  if (reference && reference.drag_area > 0) {
    const change = (run.drag_area - reference.drag_area) / reference.drag_area;
    const delta = el('div', `tile-delta ${change < 0 ? 'is-down' : 'is-up'}`,
      `${change < 0 ? '−' : '+'} ${Math.abs(change * 100).toFixed(1)}% vs ${reference.title}`);
    dragAreaTile.append(delta);
  } else {
    dragAreaTile.append(el('div', 'tile-sub', 'no earlier run to compare against'));
  }
  host.append(dragAreaTile);

  const point = (run.results?.runs || [])
    .filter((item) => item.status === 'ok')
    .flatMap((item) => item.points)
    .find((item) => item.source === 'solved');
  if (point) {
    host.append(tile('Cd', fixed(point.drag_coefficient, 4), null,
      `at ${fixed(point.speed, 1)} m/s · A ${fixed(point.frontal_area, 4)} m²`));
  }
}

function renderSolverTiles() {
  const host = $('solver-tiles');
  host.textContent = '';
  const results = state.run?.results;
  if (!results) return;

  for (const run of results.runs) {
    const colour = seriesColor(run.solver);
    if (run.status !== 'ok') {
      const element = tile(solverLabel(run.solver),
        run.status === 'unavailable' ? 'not run' : 'failed', null,
        run.message.slice(0, 140), colour);
      element.classList.add('tile-span');
      host.append(element);
      continue;
    }
    const reference = run.points.find((point) => point.source === 'solved') || run.points[0];
    host.append(tile(
      solverLabel(run.solver),
      fmt(reference.drag_coefficient, 4),
      'Cd',
      `${run.mode === 'sweep' ? 'solved each speed' : 'one solve, scaled'} · ${fixed(run.wall_time_s, 1)} s`,
      colour,
    ));
  }
}

function renderWarnings() {
  const host = $('result-warnings');
  host.textContent = '';
  const results = state.run?.results;
  if (!results) return;

  for (const warning of results.warnings) {
    const lowered = warning.toLowerCase();
    const bad = lowered.includes('disagree') || lowered.includes('not watertight');
    const good = lowered.includes('agree within');
    host.append(el('div', `note ${bad ? 'note-bad' : good ? 'note-good' : 'note-warn'}`, warning));
  }

  for (const run of results.runs) {
    if (run.status === 'ok' && run.message) {
      host.append(el('div', 'note note-warn', `${solverLabel(run.solver)}: ${run.message}`));
    }
  }
}

function renderResults() {
  const run = state.run;
  const hasResults = Boolean(run && run.kind === 'drag' && run.results?.runs?.length);
  $('results-block').hidden = !hasResults;
  if (!hasResults) return;

  renderResultTiles();
  renderChartLegend();
  renderSolverTiles();
  renderWarnings();

  if (state.resultView === 'chart') {
    $('chart-panels').hidden = false;
    $('table-wrap').hidden = true;
    requestAnimationFrame(drawCharts);
  } else {
    $('chart-panels').hidden = true;
    $('table-wrap').hidden = false;
    renderTable();
  }
}

/* The choice lives in the URL hash so a table view can be linked or bookmarked. */
function setResultView(view) {
  state.resultView = view === 'table' ? 'table' : 'chart';
  const isTable = state.resultView === 'table';
  $('btn-view-table').classList.toggle('is-active', isTable);
  $('btn-view-table').setAttribute('aria-pressed', String(isTable));
  $('btn-view-chart').classList.toggle('is-active', !isTable);
  $('btn-view-chart').setAttribute('aria-pressed', String(!isTable));
  const wanted = isTable ? '#table' : '';
  if (location.hash !== wanted) history.replaceState(null, '', wanted || location.pathname);
  renderResults();
}

/* ---------------------------------------------------------- shape results */

/* The staircase: how many separate bodies survive at each closing radius, and
   where it first reaches one. That radius is what the shell was built at, so
   the chart is the justification for the shape rather than decoration. */
function drawSweepChart() {
  const canvas = $('sweep-chart');
  const wrap = canvas.parentElement;
  const sweep = state.run?.sweep;
  const width = wrap.clientWidth;
  const height = wrap.clientHeight;
  if (!width || !height) return;

  const ratio = Math.min(window.devicePixelRatio, 2);
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  if (!sweep || !sweep.radii.length) return;

  const padLeft = 26;
  const padRight = 10;
  const padTop = 8;
  const padBottom = 16;
  const plotWidth = width - padLeft - padRight;
  const plotHeight = height - padTop - padBottom;
  if (plotWidth < 20 || plotHeight < 20) return;

  const rMax = Math.max(...sweep.radii) || 1;
  const cMax = Math.max(...sweep.components, 2);
  const x = (r) => padLeft + (r / rMax) * plotWidth;
  const y = (c) => padTop + plotHeight - ((c - 0.5) / (cMax + 0.5 - 0.5)) * plotHeight;

  context.font = '9px system-ui, sans-serif';
  context.strokeStyle = cssVar('--grid');
  context.fillStyle = cssVar('--text-muted');
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  context.lineWidth = 1;
  for (let count = 1; count <= cMax; count += Math.max(1, Math.floor(cMax / 3))) {
    const yy = Math.round(y(count)) + 0.5;
    context.beginPath();
    context.moveTo(padLeft, yy);
    context.lineTo(padLeft + plotWidth, yy);
    context.stroke();
    context.fillText(String(count), padLeft - 5, yy);
  }

  // The staircase itself, drawn as steps: the count holds until the next
  // sampled radius, which is what "this topology survives to here" means.
  context.strokeStyle = cssVar('--series-su2');
  context.lineWidth = 2;
  context.lineJoin = 'round';
  context.beginPath();
  sweep.radii.forEach((radius, index) => {
    const yy = y(sweep.components[index]);
    if (index === 0) context.moveTo(x(radius), yy);
    else {
      context.lineTo(x(radius), y(sweep.components[index - 1]));
      context.lineTo(x(radius), yy);
    }
  });
  context.lineTo(padLeft + plotWidth, y(sweep.components[sweep.components.length - 1]));
  context.stroke();

  if (sweep.merge_radius !== null && sweep.merge_radius !== undefined) {
    const mx = Math.round(x(sweep.merge_radius)) + 0.5;
    context.strokeStyle = cssVar('--good');
    context.lineWidth = 1.5;
    context.setLineDash([3, 3]);
    context.beginPath();
    context.moveTo(mx, padTop);
    context.lineTo(mx, padTop + plotHeight);
    context.stroke();
    context.setLineDash([]);
  }

  context.fillStyle = cssVar('--text-muted');
  context.textAlign = 'left';
  context.textBaseline = 'top';
  context.fillText('0', padLeft, padTop + plotHeight + 3);
  context.textAlign = 'right';
  context.fillText(`${Math.round(rMax * 1000)} mm`, padLeft + plotWidth, padTop + plotHeight + 3);
}

function renderShell() {
  const run = state.run;
  const shell = run?.shell;
  $('shell-block').hidden = !(run?.kind === 'shape' && shell);
  if (!shell) return;

  const host = $('shell-tiles');
  host.textContent = '';
  host.append(tile('Frontal area', fmt(shell.frontal_area, 4), 'm²', 'the number that sets drag'));
  host.append(tile('Closing radius', (shell.radius * 1000).toFixed(0), 'mm',
    shell.merge_radius !== null
      ? `merges at ${(shell.merge_radius * 1000).toFixed(0)} mm`
      : null));
  host.append(tile('Volume', shell.volume === null ? '—' : fmt(shell.volume, 3), 'm³'));
  host.append(tile('Bodies', String(shell.bodies), null,
    shell.bodies === 1 ? 'one closed shell, as intended' : 'still split — see the warnings'));

  const fit = tile('Payload fit',
    shell.contains_payload === true ? 'encloses' : shell.contains_payload === false ? 'sticks out' : 'unverified',
    null,
    `clearance ${(shell.clearance * 1000).toFixed(0)} mm · ${shell.triangle_count} triangles`);
  fit.classList.add('tile-span');
  host.append(fit);

  const note = $('sweep-note');
  if (shell.merge_radius === null || shell.merge_radius === undefined) {
    note.textContent = 'The payload never merged within the grid, so the shell was built at the largest radius available.';
  } else {
    note.textContent = `The payload is ${run.sweep?.bodies_at_zero ?? '?'} separate bodies uncovered and `
      + `becomes one at ${(shell.merge_radius * 1000).toFixed(0)} mm. The shell was built at `
      + `${(shell.radius * 1000).toFixed(0)} mm — the smallest radius that holds, since every `
      + 'millimetre past it is frontal area for nothing.';
  }

  const warnHost = $('shell-warnings');
  warnHost.textContent = '';
  for (const warning of run.shell_warnings || []) {
    const lowered = warning.toLowerCase();
    const bad = lowered.includes('does not fully enclose') || lowered.includes('separate bodies');
    warnHost.append(el('div', `note ${bad ? 'note-bad' : 'note-warn'}`, warning));
  }

  requestAnimationFrame(drawSweepChart);
}

/* ------------------------------------------------------------ run header */

function renderRunHead() {
  const run = state.run;
  $('runhead').hidden = !run;
  if (!run) return;

  const title = $('run-title');
  const description = $('run-description');
  if (document.activeElement !== title) title.value = run.title;
  if (document.activeElement !== description) description.value = run.description || '';

  const status = $('run-status');
  const labels = {
    draft: ['draft', 'pill-draft'],
    running: ['solving', 'pill-run'],
    done: [run.kind === 'shape' ? 'built' : 'solved', 'pill-ok'],
    failed: ['failed', 'pill-warn'],
  };
  const [text, className] = labels[run.status] || labels.draft;
  status.textContent = text;
  status.className = `pill ${className}`;

  // Every run says where its shape came from. The chain from payload to final
  // hull is then readable without guessing which tab produced which.
  const lineage = $('run-lineage');
  lineage.textContent = '';
  lineage.append(el('span', 'lineage-arrow', '↳'));
  if (run.parent_id) {
    lineage.append(document.createTextNode(run.origin === 'forked' ? 'forked from ' : 'from '));
    const link = el('a', null, run.parent_label);
    link.href = '#';
    link.addEventListener('click', (event) => {
      event.preventDefault();
      selectRun(run.parent_id);
    });
    lineage.append(link);
  } else {
    const origin = { sample: 'bundled sample', opened: 'opened from a file' }[run.origin] || 'imported';
    lineage.append(document.createTextNode(`${origin} · ${run.scene.geometry.source_name}`));
  }
}

function renderChangedNote() {
  const run = state.run;
  const changed = run?.changed || [];
  const block = $('changed-block');
  block.hidden = !changed.length;
  if (!changed.length) return;

  const note = $('changed-note');
  note.textContent = '';
  note.append(el('strong', null,
    `${changed.length} parameter${changed.length === 1 ? '' : 's'} changed since this run was solved. `));
  note.append(document.createTextNode(
    'The results below are still the ones this run produced. Compute drag to solve the new values as a new run.',
  ));
}

function renderAsRun() {
  const run = state.run;
  const lines = run?.as_run || [];
  $('asrun-block').hidden = !(lines.length && run.status !== 'running');
  if (!lines.length) return;

  const host = $('asrun-body');
  host.textContent = '';
  host.append(el('div', 'asrun-head', 'the inputs these results came from'));
  host.append(el('div', null, `${run.scene.geometry.source_name} · A ${fmt(run.metrics?.frontal_area, 4)} m²`));
  for (const line of lines) host.append(el('div', null, line));

  const stamp = $('asrun-stamp');
  const when = (run.solved_at || '').replace('T', ' ').replace('+00:00', ' UTC');
  const took = run.duration_s ? ` · took ${formatDuration(run.duration_s)}` : '';
  stamp.textContent = when ? `solved ${when}${took}` : '';
}

/* -------------------------------------------------------------- progress */

function renderProgress() {
  const run = state.run;
  const running = run?.status === 'running';
  $('progress-block').hidden = !running;
  $('error-block').hidden = run?.status !== 'failed';

  if (run?.status === 'failed') $('error-text').textContent = run.error || 'The run failed.';
  if (!running) return;

  const job = state.job && state.job.run_id === run.id ? state.job : null;
  const line = $('eta-line');
  line.textContent = '';
  const progress = job?.progress;
  if (progress) {
    $('progress-fill').style.width = `${Math.round(progress.fraction * 100)}%`;
    line.append(el('strong', null, `about ${progress.remaining_text} left`));
    line.append(document.createTextNode(
      ` · ${progress.units_done}/${progress.units_total} solves · ${formatDuration(job.elapsed_seconds)} elapsed`,
    ));
  } else {
    $('progress-fill').style.width = '4%';
    line.textContent = job ? 'starting…' : 'working…';
  }

  const log = $('run-log');
  const lines = (job?.events || []).filter((event) => event.message).map((event) => event.message);
  log.textContent = lines.join('\n');
  log.scrollTop = log.scrollHeight;
}

/* --------------------------------------------------------------- actions

   Each verb sits in the section holding the settings it uses -- solve under
   the quality and backends, derive under the packaging knobs -- so what a
   button is about to do is answered by what is directly above it. */

function renderActions() {
  const run = state.run;
  const compute = $('btn-compute');
  const derive = $('btn-derive');
  const computeWhy = $('compute-why');
  const deriveWhy = $('derive-why');
  const eta = $('eta-estimate');
  computeWhy.textContent = '';
  deriveWhy.textContent = '';
  eta.textContent = '';
  if (!run) return;

  const busyElsewhere = Boolean(state.runningId && state.runningId !== run.id);
  const isRunning = run.status === 'running';
  const isShape = run.kind === 'shape';
  const noBackends = (run.scene.solver.backends || []).length === 0;

  compute.textContent = isRunning
    ? 'Solving…'
    : isShape ? 'Compute drag on the shell' : 'Compute drag';
  derive.textContent = isRunning && isShape
    ? 'Building…'
    : isShape ? 'Derive again' : 'Derive a lower-drag shape';

  compute.disabled = isRunning || busyElsewhere || noBackends || (isShape && !run.shell);
  derive.disabled = isRunning || busyElsewhere;

  // A blocked button says why it is blocked; that reason outranks whatever the
  // button would otherwise have explained about itself.
  const blocked = isRunning
    ? 'This run is working. Its parameters are frozen; every other tab stays live.'
    : busyElsewhere
      ? `One solve at a time — ${
        state.runs.find((item) => item.id === state.runningId)?.title || 'another run'
      } is running. Reading and editing this run is unaffected.`
      : null;

  if (blocked) {
    computeWhy.textContent = blocked;
    deriveWhy.textContent = blocked;
  } else {
    if (noBackends) {
      computeWhy.textContent = 'Tick at least one solver above.';
    } else if (isShape) {
      computeWhy.textContent = run.shell
        ? 'Opens the shell as its own run and solves it. This run stays as it is.'
        : 'Build the shell first.';
    } else if (run.results?.runs?.length) {
      computeWhy.append(document.createTextNode('Opens a new run carrying '));
      computeWhy.append(el('b', null,
        run.changed?.length
          ? `your ${run.changed.length} changed parameter${run.changed.length === 1 ? '' : 's'}`
          : 'the same parameters'));
      computeWhy.append(document.createTextNode('. This one is kept.'));
    } else {
      computeWhy.textContent = 'Solves into this run — it has no results to overwrite yet.';
    }

    deriveWhy.textContent = isShape
      ? 'Re-wraps the same payload with these settings, as a new run.'
      : 'Wraps this shape in one closed shell, as a new run. This one is kept.';
  }

  if (isRunning) {
    const job = state.job && state.job.run_id === run.id ? state.job : null;
    eta.append(el('span', 'glyph glyph-running'));
    if (job?.progress) {
      eta.append(el('strong', null, `~${job.progress.remaining_text} left`));
      eta.append(document.createTextNode(
        ` · ${job.progress.units_done}/${job.progress.units_total} solves`,
      ));
    } else {
      eta.append(document.createTextNode('starting…'));
    }
  } else if (run.estimate && !isShape) {
    eta.append(el('strong', null, `~${formatDuration(run.estimate.total_seconds)}`));
    eta.append(document.createTextNode(
      run.estimate.calibrated
        ? ` estimated, from ${run.estimate.samples} past solves`
        : ' estimated (uncalibrated)',
    ));
  }
}

/* -------------------------------------------------------------- viewport */

function renderLegend() {
  const host = $('view-legend');
  host.textContent = '';
  const run = state.run;
  if (!run) return;

  // The shape is named where the eyes already are, so "which one is this?"
  // never needs the panel.
  const shapeKey = el('span', 'key');
  const isShell = run.kind === 'shape' && Boolean(run.shell);
  shapeKey.append(el('i', `swatch ${run.kind === 'shape' && !isShell ? 'swatch-payload' : 'swatch-hull'}`));
  shapeKey.append(document.createTextNode(
    run.kind === 'shape'
      ? (isShell ? `Shell — ${run.title}` : `Payload — ${run.scene.geometry.source_name}`)
      : `Shape — ${run.scene.geometry.source_name}`,
  ));
  host.append(shapeKey);

  if (run.show_payload) {
    const payloadKey = el('span', 'key');
    payloadKey.append(el('i', 'swatch swatch-payload'), document.createTextNode('Payload'));
    host.append(payloadKey);
  }

  const windKey = el('span', 'key');
  windKey.append(el('i', 'swatch swatch-wind'), document.createTextNode('Wind'));
  host.append(windKey);

  if (run.scene.road.enabled) {
    const roadKey = el('span', 'key');
    roadKey.append(el('i', 'swatch swatch-road'), document.createTextNode('Road'));
    host.append(roadKey);
  }
}

/* The mesh only travels when it actually changed: switching tabs, adopting a
   shell, or a shape run finishing. Everything else is a transform the browser
   applies itself. */
function meshKeyFor(run) {
  if (!run) return null;
  return [run.id, run.scene.geometry.source_name, run.shell_source || '', run.status].join('|');
}

async function loadMeshes(force = false) {
  const run = state.run;
  if (!run) { viewport.clearAll(); state.meshKey = null; return; }

  const key = meshKeyFor(run);
  if (!force && key === state.meshKey) return;

  try {
    const response = await api(`/api/runs/${run.id}/mesh`);
    viewport.setHull(await response.arrayBuffer());
    viewport.setHullTransparent(Boolean(run.show_payload));
    viewport.refit();

    if (run.show_payload) {
      try {
        const payload = await api(`/api/runs/${run.id}/payload-mesh`);
        viewport.setPayload(await payload.arrayBuffer());
      } catch (error) { viewport.setPayload(null); }
    } else {
      viewport.setPayload(null);
    }
    viewport.update(run.scene);
    state.meshKey = key;
  } catch (error) {
    toast(`Could not load the shape: ${error.message}`, true);
  }
}

/* ------------------------------------------------------------ rendering */

function render() {
  const run = state.run;

  renderTabs();
  renderRunHead();
  renderChangedNote();
  renderShapeFacts();
  renderGeometry();
  renderReynoldsNote();
  renderSolverList();
  renderProcesses();
  renderProgress();
  renderResults();
  renderShell();
  renderAsRun();
  renderActions();
  renderLegend();

  const isShape = run?.kind === 'shape';
  // Both action sections show on every run: the packaging knobs have to be
  // settable *before* deriving, and a shape run's solver settings are the ones
  // its shell inherits when it is opened as a run.
  for (const block of document.querySelectorAll('.param-block')) block.hidden = !run;
  $('freeze-block').hidden = run?.status !== 'running';
  $('empty-results').hidden = Boolean(
    !run || run.status === 'running' || run.status === 'failed'
    || (run.kind === 'drag' && run.results?.runs?.length)
    || (run.kind === 'shape' && run.shell),
  );
  if (!$('empty-results').hidden) {
    $('empty-results-hint').textContent = isShape
      ? 'Deriving builds the single-body shell here, from the payload on the left.'
      : 'Compute drag fills this panel in. The parameters on the left are the ones it will use.';
  }

  $('btn-download').disabled = !run;
  $('btn-download-stl').disabled = !run;
  lockControls(run ? run.status === 'running' : true);
}

function adoptRun(payload, options = {}) {
  state.run = payload;
  state.activeId = payload.id;
  if (!options.skipControls) {
    applyControls(payload.scene);
    $('quality-select').value = payload.scene.solver.quality || 'balanced';
  }
  markChangedControls(payload.changed);
  if (payload.job) state.job = payload.job;
  render();
  if (!options.keepCamera) viewport.update(payload.scene);
}

function adoptState(payload) {
  state.runs = payload.runs || [];
  state.solvers = payload.solvers || [];
  state.cores = payload.cores || null;
  state.runningId = payload.running_id || null;
  state.runningJob = payload.running_job || null;
  if (payload.active_id) state.activeId = payload.active_id;
}

/* -------------------------------------------------------------- actions */

async function refreshState() {
  adoptState(await api('/api/state'));
  renderTabs();
}

async function selectRun(runId, options = {}) {
  if (!runId) {
    state.run = null;
    state.activeId = null;
    viewport.clearAll();
    render();
    return;
  }
  try {
    const payload = await api(`/api/runs/${runId}`);
    await api(`/api/runs/${runId}/activate`, { method: 'POST' });
    state.activeId = runId;
    adoptRun(payload);
    await loadMeshes(options.forceMesh);
    watchRunningJob();
  } catch (error) {
    toast(error.message, true);
  }
}

async function openRun(payload) {
  adoptState(payload.state);
  adoptRun(payload.run);
  await loadMeshes();
  render();
}

async function closeRun(runId) {
  try {
    const payload = await api(`/api/runs/${runId}`, { method: 'DELETE' });
    adoptState(payload);
    if (payload.active_id) await selectRun(payload.active_id);
    else await selectRun(null);
  } catch (error) { toast(error.message, true); }
}

/* One poll loop for whichever run is solving, so the tab bar keeps its
   progress bar moving while you read a different run. The guard matters:
   switching tabs calls this, and a second loop started while the first is
   mid-fetch would double the polling rate for the rest of the solve. */
function watchRunningJob() {
  if (!state.runningJob || state.polling) return;
  clearTimeout(state.jobTimer);
  state.jobTimer = setTimeout(pollJob, 400);
}

async function pollJob() {
  clearTimeout(state.jobTimer);
  const jobId = state.runningJob;
  if (!jobId) return;

  state.polling = true;
  try {
    const job = await api(`/api/jobs/${jobId}`);
    state.job = job;

    if (job.status === 'running') {
      renderTabs();
      if (job.run_id === state.activeId) { renderProgress(); renderActions(); }
      state.polling = false;
      state.jobTimer = setTimeout(pollJob, 900);
      return;
    }

    state.polling = false;
    state.runningJob = null;
    if (job.state) adoptState(job.state);
    if (job.status === 'failed') {
      toast(job.error || 'The run failed', true);
    } else {
      toast(job.kind === 'shape' ? 'Shell ready' : 'Run complete');
    }

    if (job.run && job.run.id === state.activeId) {
      adoptRun(job.run);
      await loadMeshes();
    } else {
      await refreshState();
    }
    render();
  } catch (error) {
    toast(`Lost the run: ${error.message}`, true);
    state.polling = false;
    state.runningJob = null;
    state.job = null;
    render();
  }
}

async function startCompute() {
  let run = state.run;
  if (!run) return;
  try {
    // A shape run holds geometry, not drag. Computing from one is really two
    // steps -- open the shell as its own run, then solve that -- so do both
    // rather than making the obvious button the wrong one.
    if (run.kind === 'shape') {
      const opened = await api(`/api/runs/${run.id}/adopt`, { method: 'POST' });
      adoptState(opened.state);
      run = opened.run;
      state.run = run;
    }
    const payload = await api(`/api/runs/${run.id}/compute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ backends: run.scene.solver.backends }),
    });
    adoptState(payload.state);
    state.runningJob = payload.job.id;
    state.job = payload.job;
    if (payload.forked) toast(`Solving as a new run: ${payload.run.title}`);
    await selectRun(payload.run_id);
  } catch (error) {
    toast(error.message, true);
  }
}

async function startDerive() {
  const run = state.run;
  if (!run) return;
  try {
    const payload = await api(`/api/runs/${run.id}/derive`, { method: 'POST' });
    adoptState(payload.state);
    state.runningJob = payload.job.id;
    state.job = payload.job;
    toast('Searching for the smallest single-body shell');
    await selectRun(payload.run_id);
  } catch (error) {
    toast(error.message, true);
  }
}

async function adoptShell() {
  const run = state.run;
  if (!run) return;
  try {
    const payload = await api(`/api/runs/${run.id}/adopt`, { method: 'POST' });
    await openRun(payload);
    toast(`Opened ${payload.run.title} — compute its drag to cost the shape`);
  } catch (error) { toast(error.message, true); }
}

/* -------------------------------------------------------------- library */

async function refreshLibrary() {
  const payload = await api('/api/library');
  $('library-path').textContent = payload.directory;
  const host = $('library-list');
  host.textContent = '';

  if (!payload.scenes.length) {
    host.append(el('p', 'hint', 'Nothing saved yet.'));
    return;
  }

  for (const entry of payload.scenes) {
    const row = el('div', 'library-row');
    const name = el('div');
    name.append(el('div', 'library-name', entry.scene_name));
    name.append(el('div', 'library-sub', `${entry.name} · ${entry.modified.replace('T', ' ')}`));

    const pill = el('span', `pill ${entry.computed ? 'pill-ok' : 'pill-muted'}`,
      entry.computed ? 'solved' : 'not solved');

    const open = el('button', 'btn', 'Open as run');
    open.addEventListener('click', async () => {
      try {
        const loaded = await api('/api/library/open', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name: entry.name }),
        });
        $('library-modal').hidden = true;
        await openRun(loaded);
        toast(`Opened ${entry.name}`);
      } catch (error) { toast(error.message, true); }
    });

    row.append(name, pill, open);
    host.append(row);
  }
}

/* ----------------------------------------------------------------- wire */

function wire() {
  for (const button of document.querySelectorAll('[data-sample]')) {
    button.addEventListener('click', async () => {
      try {
        await openRun(await api('/api/runs/sample', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name: button.dataset.sample }),
        }));
        toast('New run — compute its drag, or derive a shape around it');
      } catch (error) { toast(error.message, true); }
    });
  }

  $('file-stl').addEventListener('change', async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    const body = new FormData();
    body.append('file', file);
    try {
      await openRun(await api('/api/runs/import', { method: 'POST', body }));
      toast(`Imported ${file.name} as a new run`);
    } catch (error) { toast(error.message, true); }
    event.target.value = '';
  });

  $('file-scene').addEventListener('change', async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    const body = new FormData();
    body.append('file', file);
    try {
      const payload = await api('/api/runs/open', { method: 'POST', body });
      await openRun(payload);
      toast(payload.run.results ? 'Opened a solved run' : 'Opened a run, not solved yet');
    } catch (error) { toast(error.message, true); }
    event.target.value = '';
  });

  $('btn-compute').addEventListener('click', startCompute);
  $('btn-derive').addEventListener('click', startDerive);
  $('btn-adopt').addEventListener('click', adoptShell);

  $('quality-select').addEventListener('change', async (event) => {
    if (!state.run) return;
    try {
      adoptRun(await api(`/api/runs/${state.run.id}/quality`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ quality: event.target.value }),
      }));
    } catch (error) { toast(error.message, true); }
  });

  $('processes-input').addEventListener('change', async (event) => {
    if (!state.run) return;
    // Blank means "decide from this machine" rather than zero ranks, so it is
    // sent through as null and the server clears the override.
    const raw = event.target.value.trim();
    try {
      adoptRun(await api(`/api/runs/${state.run.id}/processes`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ processes: raw === '' ? null : Number(raw) }),
      }));
      await refreshState();
      renderSolverList();
    } catch (error) {
      toast(error.message, true);
      renderProcesses();
    }
  });

  const saveMeta = async () => {
    if (!state.run) return;
    try {
      const payload = await api(`/api/runs/${state.run.id}/meta`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          title: $('run-title').value,
          description: $('run-description').value,
        }),
      });
      adoptRun(payload, { skipControls: true, keepCamera: true });
      await refreshState();
    } catch (error) { toast(error.message, true); }
  };
  $('run-title').addEventListener('change', saveMeta);
  $('run-description').addEventListener('change', saveMeta);

  $('btn-download').addEventListener('click', () => {
    if (state.run) window.location.href = `/api/runs/${state.run.id}/download`;
  });
  $('btn-download-stl').addEventListener('click', () => {
    if (state.run) window.location.href = `/api/runs/${state.run.id}/hull.stl`;
  });

  $('btn-library').addEventListener('click', async () => {
    $('library-modal').hidden = false;
    $('library-name').value = state.run ? state.run.title.split(' · ')[0] : '';
    try { await refreshLibrary(); } catch (error) { toast(error.message, true); }
  });
  $('btn-library-close').addEventListener('click', () => { $('library-modal').hidden = true; });
  $('library-modal').addEventListener('click', (event) => {
    if (event.target === $('library-modal')) $('library-modal').hidden = true;
  });

  $('btn-library-save').addEventListener('click', async () => {
    if (!state.run) return;
    try {
      const name = $('library-name').value || state.run.title;
      await api('/api/library/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ run_id: state.run.id, name }),
      });
      await refreshLibrary();
      toast(`Saved ${name}`);
    } catch (error) { toast(error.message, true); }
  });

  $('btn-reset-attitude').addEventListener('click', () => {
    for (const key of ['yaw_deg', 'pitch_deg', 'roll_deg']) {
      controlRegistry.orientation[key].apply(0);
      pushControl('orientation', key, 0);
    }
  });

  $('btn-view-chart').addEventListener('click', () => setResultView('chart'));
  $('btn-view-table').addEventListener('click', () => setResultView('table'));

  window.addEventListener('resize', () => {
    drawCharts();
    drawSweepChart();
  });

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') $('library-modal').hidden = true;
  });
}

/* ----------------------------------------------------------------- boot */

async function boot() {
  buildControls();
  wire();
  setupChartHover('force');
  setupChartHover('cd');
  setResultView(location.hash === '#table' ? 'table' : 'chart');

  try {
    await refreshState();
    if (state.activeId) await selectRun(state.activeId);
    else render();
    watchRunningJob();
  } catch (error) {
    toast(`Could not reach the server: ${error.message}`, true);
  }
}

boot();

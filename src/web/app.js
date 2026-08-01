/* Aero drag tool -- scene editor and results view.
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
  openfoam: { label: 'OpenFOAM', varName: '--series-openfoam', kind: 'cfd' },
  su2: { label: 'SU2', varName: '--series-su2', kind: 'cfd' },
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
  scene: null,
  metrics: null,
  reynolds: null,
  resolvedMode: null,
  estimate: null,
  solvers: [],
  job: null,
  jobTimer: null,
  resultView: 'chart',
  chartHover: null,
  fairing: null,
  ranking: [],
  hasPayload: false,
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
  toast.timer = setTimeout(() => { element.hidden = true; }, isBad ? 7000 : 3200);
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
    document.querySelector('.key-payload').classList.remove('is-hidden');
  }

  /* Remembered rather than applied once: the caller sets this while adopting a
     scene, which happens before the new hull mesh arrives, so setHull has to
     re-apply it to the fresh material. */
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
    const view = new DataView(buffer);
    const triangles = view.getUint32(0, true);
    const positions = new Float32Array(buffer, 4, triangles * 9);
    const normals = new Float32Array(buffer, 4 + triangles * 9 * 4, triangles * 9);

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
      new THREE.LineBasicMaterial({ color: new THREE.Color(cssVar('--text-primary')), transparent: true, opacity: 0.18 }),
    );
    hullGroup.add(hullEdges);
    applyHullTransparency();

    modelScale = Math.max(geometry.boundingSphere.radius * 2, 0.05);
    framed = false;
    $('view-empty').hidden = true;
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

    document.querySelector('.key-road').classList.toggle('is-hidden', !road.enabled);

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
    camera.position.copy(centre).add(new THREE.Vector3(0.75, -1, 0.42).normalize().multiplyScalar(distance));
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

  return { setHull, setPayload, setHullTransparent, update, refit: () => { framed = false; } };
})();

/* -------------------------------------------------------------- controls */

function buildControl(section, spec) {
  const row = document.createElement('div');
  row.className = 'control';

  if (spec.type === 'check') {
    row.classList.add('control-check');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.id = `ctl-${section}-${spec.key}`;
    const label = document.createElement('label');
    label.className = 'control-label';
    label.htmlFor = input.id;
    label.textContent = spec.label;
    row.append(input, label);
    input.addEventListener('change', () => pushControl(section, spec.key, input.checked));
    return { row, apply: (value) => { input.checked = Boolean(value); } };
  }

  const label = document.createElement('label');
  label.className = 'control-label';
  label.textContent = spec.unit ? `${spec.label} (${spec.unit})` : spec.label;
  row.append(label);

  if (spec.type === 'select') {
    const select = document.createElement('select');
    select.className = 'control-select';
    for (const [value, text] of spec.options) {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = text;
      select.append(option);
    }
    row.append(select);
    label.htmlFor = select.id = `ctl-${section}-${spec.key}`;
    select.addEventListener('change', () => pushControl(section, spec.key, select.value));
    return { row, apply: (value) => { select.value = value; } };
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
      number.value = spec.expo ? Number(value).toExponential(3) : Number(value).toFixed(spec.decimals ?? 3);
      if (slider) slider.value = value;
    },
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

function applyControls(scene_) {
  for (const [section, controls] of Object.entries(controlRegistry)) {
    const values = scene_[section] || {};
    for (const [key, control] of Object.entries(controls)) {
      if (values[key] !== undefined && values[key] !== null) control.apply(values[key]);
    }
  }
}

/* Local echo first so dragging a slider feels immediate, then a debounced
   patch to the server which owns the derived quantities. */
let pendingPatch = {};
let patchTimer = null;

function pushControl(section, key, value, live = false) {
  if (!state.scene) return;
  state.scene[section][key] = value;
  if (section === 'wind') {
    const speed = state.scene.wind.speed;
    const az = (state.scene.wind.azimuth_deg * Math.PI) / 180;
    const el = (state.scene.wind.elevation_deg * Math.PI) / 180;
    state.scene.wind.vector = [
      speed * Math.cos(el) * Math.cos(az),
      speed * Math.cos(el) * Math.sin(az),
      speed * Math.sin(el),
    ];
  }
  viewport.update(state.scene);

  pendingPatch[section] = { ...(pendingPatch[section] || {}), [key]: value };
  clearTimeout(patchTimer);
  patchTimer = setTimeout(flushPatch, live ? 220 : 0);
}

async function flushPatch() {
  if (!Object.keys(pendingPatch).length) return;
  const patch = pendingPatch;
  pendingPatch = {};
  try {
    const payload = await api('/api/scene', {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(patch),
    });
    adoptScene(payload, { keepCamera: true, skipControls: true });
  } catch (error) {
    toast(`Update failed: ${error.message}`, true);
  }
}

/* --------------------------------------------------------------- solvers */

function renderSolverList() {
  const host = $('solver-list');
  host.textContent = '';
  const selected = new Set(state.scene ? state.scene.solver.backends : []);

  for (const name of SOLVER_ORDER) {
    const info = state.solvers.find((item) => item.name === name);
    if (!info) continue;

    const row = document.createElement('div');
    row.className = 'solver-row';
    if (!info.available) row.classList.add('is-off');

    const check = document.createElement('input');
    check.type = 'checkbox';
    check.id = `solver-${name}`;
    check.checked = selected.has(name);
    check.disabled = !state.scene;

    const label = document.createElement('label');
    label.className = 'solver-name';
    label.htmlFor = check.id;
    const dot = document.createElement('i');
    dot.className = `dot dot-${name}`;
    label.append(dot, document.createTextNode(info.label));

    const pill = document.createElement('span');
    pill.className = `pill ${info.available ? 'pill-ok' : 'pill-warn'}`;
    pill.textContent = info.available ? 'ready' : 'not here';

    const detail = document.createElement('div');
    detail.className = 'solver-detail';
    detail.textContent = info.detail;

    row.append(check, label, pill, detail);
    host.append(row);

    check.addEventListener('change', () => {
      const backends = SOLVER_ORDER.filter((item) => $(`solver-${item}`)?.checked);
      state.scene.solver.backends = backends;
      pendingPatch.solver = { ...(pendingPatch.solver || {}), backends };
      flushPatch();
      updateRunButton();
    });
  }
}

function updateRunButton() {
  const button = $('btn-run');
  if (!state.scene) {
    button.disabled = true;
    button.textContent = 'Compute drag';
    $('btn-analyze').disabled = true;
    $('btn-compare').disabled = true;
    $('btn-download-stl').disabled = true;
    return;
  }
  const running = Boolean(state.job && state.job.status === 'running');
  const backends = state.scene.solver.backends || [];
  button.disabled = running || backends.length === 0;
  button.textContent = running ? 'Running…' : `Compute drag (${backends.length})`;
  $('btn-analyze').disabled = running || !state.hasPayload;
  $('btn-compare').disabled = running || !(state.fairing?.candidates || []).length;
  $('btn-download-stl').disabled = running || !state.scene;
  setInputsLocked(running);
}

/* -------------------------------------------------------------- geometry */

function tile(label, value, unit, sub, accent) {
  const element = document.createElement('div');
  element.className = 'tile';

  const head = document.createElement('div');
  head.className = 'tile-label';
  if (accent) {
    const dot = document.createElement('i');
    dot.className = 'dot';
    dot.style.background = accent;
    head.append(dot);
  }
  head.append(document.createTextNode(label));

  const body = document.createElement('div');
  body.className = 'tile-value';
  body.append(document.createTextNode(value));
  if (unit) {
    const unitSpan = document.createElement('span');
    unitSpan.className = 'tile-unit';
    unitSpan.textContent = unit;
    body.append(unitSpan);
  }

  element.append(head, body);
  if (sub) {
    const subElement = document.createElement('div');
    subElement.className = 'tile-sub';
    subElement.textContent = sub;
    element.append(subElement);
  }
  return element;
}

function renderGeometry() {
  const host = $('geometry-tiles');
  host.textContent = '';
  const metrics = state.metrics;
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

function renderReynoldsNote() {
  const note = $('reynolds-note');
  const advice = state.reynolds;
  if (!advice) { note.hidden = true; return; }

  note.textContent = '';
  note.className = `note ${advice.crosses_critical_band || advice.ratio > 3 ? 'note-warn' : 'note-good'}`;

  const head = document.createElement('strong');
  const mode = state.resolvedMode === 'sweep' ? 'Solving every speed' : 'One run, curve scaled as V²';
  head.textContent = `${mode}. `;
  note.append(head);
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

  const series = chartSeries(state.scene?.results);
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
  Object.assign(layout, { left: padLeft, right: padLeft + plotWidth, top: padTop, bottom: padTop + plotHeight, xScale, yScale, xMin, xMax });

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
  const series = chartSeries(state.scene?.results);
  if (series.length < 1) return;

  // Two or more series need a legend to carry identity. One does not -- the
  // solver tile below names it, and the endpoint is directly labelled.
  if (series.length > 1) {
    for (const item of series) {
      const key = document.createElement('span');
      key.className = 'legend-key';
      const line = document.createElement('i');
      line.className = 'legend-line';
      line.style.background = item.color;
      key.append(line, document.createTextNode(item.label));
      host.append(key);
    }
  }

  const anyScaled = series.some((item) => item.points.some((point) => point.source === 'scaled'));
  if (anyScaled) {
    const note = document.createElement('span');
    note.className = 'legend-key';
    note.style.color = cssVar('--text-muted');
    note.textContent = 'hollow markers = scaled from one solve';
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
    const head = document.createElement('div');
    head.className = 'tooltip-head';
    head.textContent = `${fixed(nearest, 2)} m/s`;
    tooltip.append(head);

    for (const item of series) {
      const point = item.points.reduce(
        (acc, candidate) => (Math.abs(candidate.speed - nearest) < Math.abs(acc.speed - nearest) ? candidate : acc),
        item.points[0],
      );
      const row = document.createElement('div');
      row.className = 'tooltip-row';

      const swatch = document.createElement('i');
      swatch.className = 'tooltip-key';
      swatch.style.background = item.color;

      const value = document.createElement('span');
      value.className = 'tooltip-value';
      value.textContent = spec.format(spec.value(point));

      const name = document.createElement('span');
      name.className = 'tooltip-name';
      name.textContent = item.label;

      row.append(swatch, value, name);
      if (point.source === 'scaled') {
        const tag = document.createElement('span');
        tag.className = 'tooltip-tag';
        tag.textContent = 'scaled';
        row.append(tag);
      }
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
  const series = chartSeries(state.scene?.results);
  if (!series.length) return;

  const speeds = [...new Set(series.flatMap((item) => item.points.map((point) => point.speed)))]
    .sort((a, b) => a - b);

  const table = document.createElement('table');
  table.className = 'data';
  const caption = document.createElement('caption');
  caption.textContent = 'Cd is listed per speed because it is not constant: it varies with Reynolds '
    + 'number. Values marked * were scaled from a single solve.';
  table.append(caption);

  const thead = document.createElement('thead');
  const headRow = document.createElement('tr');
  const columns = ['Speed (m/s)', 'Re'];
  for (const item of series) columns.push(`${item.label} Cd`, `${item.label} (N)`);
  for (const text of columns) {
    const th = document.createElement('th');
    th.scope = 'col';
    th.textContent = text;
    headRow.append(th);
  }
  thead.append(headRow);
  table.append(thead);

  const tbody = document.createElement('tbody');
  for (const speed of speeds) {
    const row = document.createElement('tr');
    const th = document.createElement('th');
    th.scope = 'row';
    th.textContent = fixed(speed, 2);
    row.append(th);

    const reference = series[0].points.find((point) => Math.abs(point.speed - speed) < 1e-9);
    const re = document.createElement('td');
    re.textContent = reference ? fmtSi(reference.reynolds) : '—';
    row.append(re);

    for (const item of series) {
      const point = item.points.find((candidate) => Math.abs(candidate.speed - speed) < 1e-9);
      const mark = point && point.source === 'scaled' ? ' *' : '';

      const cd = document.createElement('td');
      cd.textContent = point ? `${fixed(point.drag_coefficient, 4)}${mark}` : '—';
      row.append(cd);

      const force = document.createElement('td');
      force.textContent = point ? `${fixed(point.drag_force, 2)}${mark}` : '—';
      row.append(force);
    }
    tbody.append(row);
  }
  table.append(tbody);
  host.append(table);
}

/* -------------------------------------------------------------- results */

function renderSolverTiles() {
  const host = $('solver-tiles');
  host.textContent = '';
  const results = state.scene?.results;
  if (!results) return;

  for (const run of results.runs) {
    const colour = seriesColor(run.solver);
    if (run.status !== 'ok') {
      const element = tile(solverLabel(run.solver), run.status === 'unavailable' ? 'not run' : 'failed', null,
        run.message.slice(0, 140), colour);
      element.classList.add('tile-span');
      host.append(element);
      continue;
    }
    const reference = run.points.find((point) => point.source === 'solved') || run.points[0];
    const element = tile(
      solverLabel(run.solver),
      fmt(reference.drag_coefficient, 4),
      'Cd',
      `${run.mode === 'sweep' ? 'solved each speed' : 'one solve, scaled'} · ${fixed(run.wall_time_s, 1)} s`,
      colour,
    );
    host.append(element);
  }
}

function renderWarnings() {
  const host = $('result-warnings');
  host.textContent = '';
  const results = state.scene?.results;
  if (!results) return;

  for (const warning of results.warnings) {
    const note = document.createElement('div');
    const lowered = warning.toLowerCase();
    const bad = lowered.includes('disagree') || lowered.includes('not watertight');
    const good = lowered.includes('agree within');
    note.className = `note ${bad ? 'note-bad' : good ? 'note-good' : 'note-warn'}`;
    note.textContent = warning;
    host.append(note);
  }

  for (const run of results.runs) {
    if (run.status === 'ok' && run.message) {
      const note = document.createElement('div');
      note.className = 'note note-warn';
      note.textContent = `${solverLabel(run.solver)}: ${run.message}`;
      host.append(note);
    }
  }
}

function renderResults() {
  const section = $('results-section');
  const results = state.scene?.results;
  if (!results || !results.runs.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  renderChartLegend();
  renderSolverTiles();
  renderWarnings();
  renderResultMeta();
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

/* ---------------------------------------------------- fairing candidates */

function formatDuration(seconds) {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return '—';
  if (seconds < 90) return `${Math.round(seconds)} s`;
  const minutes = seconds / 60;
  if (minutes < 90) return `${Math.round(minutes)} min`;
  const hours = Math.floor(minutes / 60);
  const rest = Math.round(minutes % 60);
  return rest ? `${hours} h ${String(rest).padStart(2, '0')} min` : `${hours} h`;
}

/* The staircase: how many separate bodies survive at each closing radius.
   Flat runs are the stable topologies, and the wider the run the more the
   design is a real choice rather than two lumps happening to nearly touch. */
function drawSweepChart() {
  const block = $('sweep-block');
  const sweep = state.fairing?.sweep;
  if (!sweep || !sweep.radii?.length) { block.hidden = true; return; }
  block.hidden = false;

  const canvas = $('sweep-chart');
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return;

  const ratio = Math.min(window.devicePixelRatio, 2);
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const padLeft = 20;
  const padBottom = 16;
  const padTop = 6;
  const plotWidth = width - padLeft - 6;
  const plotHeight = height - padTop - padBottom;

  const radii = sweep.radii;
  const counts = sweep.components;
  const rMax = Math.max(...radii) || 1;
  const cMax = Math.max(...counts, 1);

  const x = (r) => padLeft + (r / rMax) * plotWidth;
  const y = (c) => padTop + plotHeight - ((c - 0.5) / (cMax + 0.5 - 0.5)) * plotHeight;

  context.strokeStyle = cssVar('--grid');
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(padLeft, padTop + plotHeight + 0.5);
  context.lineTo(padLeft + plotWidth, padTop + plotHeight + 0.5);
  context.stroke();

  // Step plot: the count holds until the next sample.
  context.strokeStyle = cssVar('--series-openfoam');
  context.lineWidth = 2;
  context.beginPath();
  radii.forEach((radius, index) => {
    const px = x(radius);
    const py = y(counts[index]);
    if (index === 0) context.moveTo(px, py);
    else { context.lineTo(px, y(counts[index - 1])); context.lineTo(px, py); }
  });
  context.lineTo(padLeft + plotWidth, y(counts[counts.length - 1]));
  context.stroke();

  context.fillStyle = cssVar('--text-muted');
  context.font = '9px system-ui, sans-serif';
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  for (const count of [...new Set(counts)]) context.fillText(String(count), padLeft - 4, y(count));

  context.textAlign = 'center';
  context.textBaseline = 'top';
  context.fillText('0', padLeft, padTop + plotHeight + 3);
  context.fillText(`${Math.round(rMax * 1000)} mm`, padLeft + plotWidth, padTop + plotHeight + 3);
}

function renderCandidates() {
  const host = $('candidate-list');
  host.textContent = '';
  const fairing = state.fairing;
  const candidates = fairing?.candidates || [];

  const bestIndex = state.ranking.length ? state.ranking[0].index : null;

  for (const candidate of candidates) {
    const card = document.createElement('div');
    card.className = 'candidate';
    if (candidate.selected) card.classList.add('is-selected');
    if (candidate.index === bestIndex) card.classList.add('is-best');
    card.tabIndex = 0;
    card.setAttribute('role', 'button');

    const head = document.createElement('div');
    head.className = 'candidate-head';
    const title = document.createElement('span');
    title.className = 'candidate-title';
    title.textContent = candidate.components === 1
      ? 'One merged shell'
      : `${candidate.components} separate bodies`;
    const area = document.createElement('span');
    area.className = 'rank-value';
    area.textContent = fixed(candidate.frontal_area, 3);
    const unit = document.createElement('span');
    unit.className = 'rank-unit';
    unit.textContent = ' m²';
    area.append(unit);
    head.append(title, area);

    const metrics = document.createElement('div');
    metrics.className = 'candidate-metrics';
    const parts = [`r = ${Math.round(candidate.radius * 1000)} mm`];
    if (candidate.min_gap) parts.push(`gap ${Math.round(candidate.min_gap * 1000)} mm`);
    if (candidate.volume) parts.push(`${fixed(candidate.volume, 2)} m³`);
    metrics.textContent = parts.join(' · ');

    const flags = document.createElement('div');
    flags.className = 'candidate-flags';
    const addTag = (text, kind) => {
      const tag = document.createElement('span');
      tag.className = `tag tag-${kind}`;
      tag.textContent = text;
      flags.append(tag);
    };
    if (candidate.contains_payload === true) addTag('payload fits', 'good');
    else if (candidate.contains_payload === false) addTag('payload sticks out', 'bad');
    else addTag('fit unchecked', 'warn');
    if (candidate.choked) addTag('choked gap', 'warn');
    if (!candidate.watertight) addTag('not watertight', 'bad');

    const results = candidate.results;
    if (results) {
      const run = (results.runs || []).find((item) => item.status === 'ok');
      const point = run?.points?.find((item) => item.source === 'solved') || run?.points?.[0];
      if (point) addTag(`Cd·A ${fixed(point.drag_coefficient * point.frontal_area, 4)} m²`, 'good');
    }

    card.append(head, metrics, flags);
    const choose = () => selectCandidate(candidate.index);
    card.addEventListener('click', choose);
    card.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); choose(); }
    });
    host.append(card);
  }

  const warnHost = $('fairing-warnings');
  warnHost.textContent = '';
  for (const warning of fairing?.warnings || []) {
    const note = document.createElement('div');
    note.className = 'note note-warn';
    note.textContent = warning;
    warnHost.append(note);
  }

  requestAnimationFrame(drawSweepChart);
}

function renderRanking() {
  const block = $('ranking-block');
  const host = $('ranking-list');
  host.textContent = '';
  if (!state.ranking.length) { block.hidden = true; return; }
  block.hidden = false;

  state.ranking.forEach((entry, position) => {
    const row = document.createElement('div');
    row.className = 'rank-row';
    if (position === 0) row.classList.add('is-best');

    const place = document.createElement('span');
    place.className = 'rank-place';
    place.textContent = `${position + 1}`;

    const label = document.createElement('div');
    const name = document.createElement('div');
    name.className = 'rank-label';
    name.textContent = entry.components === 1 ? 'One merged shell' : `${entry.components} separate bodies`;
    const sub = document.createElement('div');
    sub.className = 'rank-sub';
    sub.textContent = `${solverLabel(entry.solver)} · Cd ${fixed(entry.drag_coefficient, 3)} · `
      + `A ${fixed(entry.frontal_area, 3)} m²`;
    label.append(name, sub);

    const value = document.createElement('div');
    value.className = 'rank-value';
    value.textContent = fixed(entry.drag_area, 4);
    const unit = document.createElement('span');
    unit.className = 'rank-unit';
    unit.textContent = ' m² Cd·A';
    value.append(unit);

    row.append(place, label, value);
    host.append(row);
  });
}

async function selectCandidate(index) {
  try {
    const payload = await api('/api/fairing/select', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ index }),
    });
    adoptScene(payload);
    await loadMesh();
    await loadPayloadMesh();
    viewport.update(state.scene);
    toast('Fairing applied to the scene');
  } catch (error) { toast(error.message, true); }
}

/* ------------------------------------------------------------------ eta */

function renderEta() {
  const line = $('eta-line');
  const bar = $('progress-bar');
  const fill = $('progress-fill');
  line.textContent = '';

  const job = state.job;
  if (job && job.status === 'running') {
    const progress = job.progress;
    bar.hidden = false;
    if (progress) {
      fill.style.width = `${Math.round(progress.fraction * 100)}%`;
      const strong = document.createElement('strong');
      strong.textContent = `about ${progress.remaining_text} left`;
      line.append(strong);
      line.append(document.createTextNode(
        ` · ${progress.units_done}/${progress.units_total} solves · `
        + `${formatDuration(job.elapsed_seconds)} elapsed`,
      ));
    } else {
      fill.style.width = '4%';
      line.textContent = 'starting…';
    }
    return;
  }

  bar.hidden = true;
  fill.style.width = '0%';

  const estimate = state.estimate;
  if (!estimate || !state.scene) return;
  const strong = document.createElement('strong');
  strong.textContent = `~${formatDuration(estimate.total_seconds)}`;
  line.append(strong);
  const note = document.createElement('span');
  note.className = 'eta-note';
  note.textContent = estimate.calibrated
    ? ` estimated, from ${estimate.samples} past solves on this machine`
    : ' estimated (uncalibrated — the first real run will sharpen this)';
  line.append(note);
}

/* While a solver runs, every parameter is frozen. Otherwise the panel could
   show values the running job was never given, and the results would look like
   they belong to settings that were never used. */
function setInputsLocked(locked) {
  const targets = document.querySelectorAll(
    '.panel-left input, .panel-left select, .panel-left button, '
    + '#quality-select, #solver-list input, #btn-library, #btn-save',
  );
  for (const element of targets) {
    if (locked) {
      if (!element.disabled) {
        element.disabled = true;
        element.dataset.lockedByRun = '1';
      }
    } else if (element.dataset.lockedByRun) {
      element.disabled = false;
      delete element.dataset.lockedByRun;
    }
  }
  document.querySelector('.layout').classList.toggle('is-running', locked);
  document.body.classList.toggle('is-running', locked);

  const note = $('lock-note');
  if (note) note.hidden = !locked;
}

function renderResultMeta() {
  const results = state.scene?.results;
  const title = $('result-title');
  const description = $('result-description');
  const stamp = $('result-stamp');
  if (!results) return;

  if (document.activeElement !== title) title.value = results.title || '';
  if (document.activeElement !== description) description.value = results.description || '';

  const when = (results.computed_at || '').replace('T', ' ').replace('+00:00', ' UTC');
  stamp.textContent = when ? `computed ${when} on ${results.host}` : '';
}

async function saveResultMeta() {
  if (!state.scene?.results) return;
  try {
    await api('/api/results', {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        title: $('result-title').value,
        description: $('result-description').value,
      }),
    });
    state.scene.results.title = $('result-title').value;
    state.scene.results.description = $('result-description').value;
  } catch (error) { toast(error.message, true); }
}

/* ------------------------------------------------------------ scene load */

async function loadMesh() {
  const response = await api('/api/scene/mesh');
  const buffer = await response.arrayBuffer();
  viewport.setHull(buffer);
}

function adoptScene(payload, options = {}) {
  if (!payload || !payload.scene) {
    state.scene = null;
    return;
  }
  state.scene = payload.scene;
  state.metrics = payload.metrics;
  state.reynolds = payload.reynolds;
  state.resolvedMode = payload.resolved_mode;
  state.estimate = payload.estimate;
  state.fairing = payload.fairing;
  state.hasPayload = Boolean(payload.has_payload);
  // A solve started elsewhere (or before a reload) still owns these inputs.
  if (payload.active_job && !state.job) {
    state.job = { ...payload.active_job, status: 'running' };
    $('run-log').hidden = false;
    pollJob();
  }

  if (!options.skipControls) applyControls(state.scene);
  $('quality-select').value = state.scene.solver.quality || 'balanced';
  $('scene-name').value = state.scene.name;
  $('scene-name').disabled = false;
  $('btn-save').disabled = false;
  $('btn-download').disabled = false;
  $('btn-download-stl').disabled = false;

  const status = $('scene-status');
  status.textContent = payload.computed ? 'computed' : 'not computed';
  status.className = `pill ${payload.computed ? 'pill-ok' : 'pill-muted'}`;

  viewport.update(state.scene);
  // Until a fairing is generated the hull *is* the payload, so showing both
  // would just draw the same mesh twice, one ghosted over the other.
  viewport.setHullTransparent(Boolean(state.hasPayload && state.scene.fairing));
  $('btn-analyze').disabled = !state.hasPayload;
  renderGeometry();
  renderReynoldsNote();
  renderSolverList();
  renderCandidates();
  renderEta();
  renderResults();
  updateRunButton();
}

async function loadPayloadMesh() {
  // The server frames the payload on the *hull's* centroid so one transform
  // places both, which means this must be re-fetched whenever the hull changes.
  if (!state.hasPayload || !state.scene?.fairing) { viewport.setPayload(null); return; }
  try {
    const response = await api('/api/scene/payload-mesh');
    viewport.setPayload(await response.arrayBuffer());
  } catch (error) { viewport.setPayload(null); }
}

async function refreshRanking() {
  try {
    const payload = await api('/api/fairing/ranking');
    state.ranking = payload.ranking || [];
  } catch (error) { state.ranking = []; }
  renderRanking();
  renderCandidates();
}

async function loadSceneFresh(payload) {
  adoptScene(payload);
  await loadMesh();
  await loadPayloadMesh();
  viewport.refit();
  viewport.update(state.scene);
}

/* ------------------------------------------------------------------ run */

async function startRun() {
  try {
    $('run-log').hidden = false;
    $('run-log').textContent = 'starting…';
    const payload = await api('/api/run', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ backends: state.scene.solver.backends }),
    });
    state.job = payload.job;
    updateRunButton();
    pollJob();
  } catch (error) {
    toast(`Could not start: ${error.message}`, true);
    $('run-log').hidden = true;
  }
}

function renderRunLog(job) {
  const log = $('run-log');
  const lines = job.events
    .filter((event) => event.message)
    .map((event) => event.message);
  log.textContent = lines.join('\n');
  log.scrollTop = log.scrollHeight;
}

async function pollJob() {
  clearTimeout(state.jobTimer);
  if (!state.job) return;
  try {
    const job = await api(`/api/jobs/${state.job.id}`);
    const kind = state.job.kind || job.kind;
    state.job = job;
    renderRunLog(job);
    renderEta();

    if (job.status === 'running') {
      state.jobTimer = setTimeout(pollJob, 900);
      return;
    }
    updateRunButton();
    if (job.status === 'failed') {
      toast(job.error || 'Run failed', true);
      renderEta();
      return;
    }

    if (kind === 'fairing') {
      state.fairing = job.results;
      state.ranking = [];
      renderCandidates();
      renderRanking();
      const count = job.results?.candidates?.length || 0;
      toast(count ? `${count} candidate fairings ready` : 'No candidates found');
    } else if (kind === 'compare') {
      state.fairing = job.results;
      await refreshRanking();
      const fresh = await api('/api/scene');
      adoptScene(fresh, { skipControls: true });
      await loadMesh();
      await loadPayloadMesh();
      viewport.update(state.scene);
      toast('Comparison complete — best design selected');
    } else {
      if (job.scene) adoptScene(job.scene, { skipControls: true });
      toast('Run complete');
    }
    renderEta();
  } catch (error) {
    toast(`Lost the run: ${error.message}`, true);
    state.job = null;
    updateRunButton();
    renderEta();
  }
}

/* -------------------------------------------------------------- library */

async function refreshLibrary() {
  const payload = await api('/api/library');
  $('library-path').textContent = payload.directory;
  const host = $('library-list');
  host.textContent = '';

  if (!payload.scenes.length) {
    const empty = document.createElement('p');
    empty.className = 'hint';
    empty.textContent = 'No saved scenes yet.';
    host.append(empty);
    return;
  }

  for (const entry of payload.scenes) {
    const row = document.createElement('div');
    row.className = 'library-row';

    const name = document.createElement('div');
    name.className = 'library-name';
    name.textContent = entry.name;
    const meta = document.createElement('div');
    meta.className = 'library-meta';
    meta.textContent = entry.modified.replace('T', ' ').replace('+00:00', ' UTC');
    name.append(meta);

    const pill = document.createElement('span');
    pill.className = `pill ${entry.computed ? 'pill-ok' : 'pill-muted'}`;
    pill.textContent = entry.computed ? 'computed' : 'scene only';

    const open = document.createElement('button');
    open.className = 'btn';
    open.textContent = 'Open';
    open.addEventListener('click', async () => {
      try {
        const payloadScene = await api('/api/library/load', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name: entry.name }),
        });
        await loadSceneFresh(payloadScene);
        $('library-modal').hidden = true;
        toast(`Opened ${entry.name}`);
      } catch (error) {
        toast(error.message, true);
      }
    });

    row.append(name, pill, open);
    host.append(row);
  }
}

/* ----------------------------------------------------------------- wire */

function wire() {
  for (const button of document.querySelectorAll('[data-sample]')) {
    button.addEventListener('click', async () => {
      const name = button.dataset.sample;
      try {
        await loadSceneFresh(await api('/api/scene/sample', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name }),
        }));
        toast(`Loaded the sample ${name} — compute its drag, or analyse packaging to fair it`);
      } catch (error) { toast(error.message, true); }
    });
  }

  $('file-stl').addEventListener('change', async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    const body = new FormData();
    body.append('file', file);
    try {
      await loadSceneFresh(await api('/api/scene/stl', { method: 'POST', body }));
      toast(`Imported ${file.name} — compute its drag, or analyse packaging to fair it`);
    } catch (error) { toast(error.message, true); }
    event.target.value = '';
  });

  $('file-scene').addEventListener('change', async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    const body = new FormData();
    body.append('file', file);
    try {
      const payload = await api('/api/scene/file', { method: 'POST', body });
      await loadSceneFresh(payload);
      toast(payload.computed ? 'Imported a computed scene' : 'Imported a scene, not computed yet');
    } catch (error) { toast(error.message, true); }
    event.target.value = '';
  });

  $('btn-analyze').addEventListener('click', async () => {
    try {
      $('run-log').hidden = false;
      $('run-log').textContent = 'analysing…';
      const payload = await api('/api/fairing/analyze', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({}),
      });
      state.job = { ...payload.job, kind: 'fairing' };
      updateRunButton();
      pollJob();
    } catch (error) { toast(error.message, true); }
  });

  $('btn-compare').addEventListener('click', async () => {
    try {
      $('run-log').hidden = false;
      $('run-log').textContent = 'comparing…';
      const payload = await api('/api/fairing/compare', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          backends: state.scene.solver.backends,
          quality: $('quality-select').value,
        }),
      });
      state.job = { ...payload.job, kind: 'compare' };
      updateRunButton();
      pollJob();
    } catch (error) { toast(error.message, true); }
  });

  $('quality-select').addEventListener('change', async (event) => {
    try {
      const payload = await api('/api/quality', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ quality: event.target.value }),
      });
      adoptScene(payload);
    } catch (error) { toast(error.message, true); }
  });

  $('btn-download').addEventListener('click', () => { window.location.href = '/api/scene/download'; });
  $('btn-download-stl').addEventListener('click', () => { window.location.href = '/api/scene/hull.stl'; });

  $('result-title').addEventListener('change', saveResultMeta);
  $('result-description').addEventListener('change', saveResultMeta);

  $('btn-library').addEventListener('click', async () => {
    $('library-modal').hidden = false;
    $('library-name').value = state.scene ? state.scene.name : '';
    try { await refreshLibrary(); } catch (error) { toast(error.message, true); }
  });
  $('btn-library-close').addEventListener('click', () => { $('library-modal').hidden = true; });
  $('library-modal').addEventListener('click', (event) => {
    if (event.target === $('library-modal')) $('library-modal').hidden = true;
  });

  const save = async () => {
    if (!state.scene) return;
    try {
      const name = $('library-name').value || state.scene.name;
      await api('/api/library/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name }),
      });
      await refreshLibrary();
      toast(`Saved ${name}`);
    } catch (error) { toast(error.message, true); }
  };
  $('btn-library-save').addEventListener('click', save);
  $('btn-save').addEventListener('click', async () => {
    $('library-modal').hidden = false;
    $('library-name').value = state.scene ? state.scene.name : '';
    await refreshLibrary();
  });

  $('scene-name').addEventListener('change', (event) => {
    if (!state.scene) return;
    pendingPatch.name = event.target.value;
    state.scene.name = event.target.value;
    flushPatch();
  });

  $('btn-reset-attitude').addEventListener('click', () => {
    for (const key of ['yaw_deg', 'pitch_deg', 'roll_deg']) pushControl('orientation', key, 0);
    applyControls(state.scene);
  });

  $('btn-run').addEventListener('click', startRun);

  $('btn-view-chart').addEventListener('click', () => setResultView('chart'));
  $('btn-view-table').addEventListener('click', () => setResultView('table'));
  window.addEventListener('hashchange', () => {
    setResultView(location.hash === '#table' ? 'table' : 'chart');
  });

  window.addEventListener('resize', () => {
    if (state.resultView === 'chart') drawCharts();
    drawSweepChart();
  });
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
    renderResults();
    if (state.scene) viewport.update(state.scene);
  });
}

/* ----------------------------------------------------------------- boot */

async function boot() {
  buildControls();
  setupChartHover('force');
  setupChartHover('cd');
  wire();
  if (location.hash === '#table') state.resultView = 'table';

  try {
    const payload = await api('/api/solvers');
    state.solvers = payload.solvers;
  } catch (error) {
    state.solvers = [];
  }
  renderSolverList();

  try {
    const payload = await api('/api/scene');
    if (payload && payload.scene) {
      await loadSceneFresh(payload);
      // A comparison from an earlier visit is still on the server; show its
      // ranking rather than making the user re-run to see it.
      if ((state.fairing?.candidates || []).some((item) => item.results)) await refreshRanking();
    }
  } catch (error) { /* nothing loaded yet is normal */ }

  updateRunButton();
}

boot();

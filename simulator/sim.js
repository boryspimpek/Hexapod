const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const LEGS = Object.fromEntries(["lf","rf","lr","rr"].map(n => [n, {n}]));
// Scena Three.js.
const canvas = document.getElementById("c");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);
scene.fog = new THREE.Fog(0x0e1116, 700, 1400);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 3000);
camera.up.set(0, 0, 1);
scene.add(new THREE.AmbientLight(0xffffff, 0.55));

const sun = new THREE.DirectionalLight(0xffffff, 0.8);
sun.position.set(200, -300, 500);
scene.add(sun);

const robotRoot = new THREE.Group();
const robotModel = new THREE.Group();
robotRoot.add(robotModel);
scene.add(robotRoot);
const grid = new THREE.GridHelper(4000, 80, 0x262d38, 0x262d38);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
const originMarker = new THREE.AxesHelper(100);
scene.add(originMarker);

const bodyMaterial = new THREE.MeshStandardMaterial({
  color: 0x2b3441,
  roughness: 0.5,
  metalness: 0.3,
});
const body = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), bodyMaterial);
const nose = new THREE.Mesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshStandardMaterial({ color: 0x35d0ba }),
);
robotModel.add(body, nose);

const limbMaterials = [0x8fa3bd, 0x5f7794, 0xb9c6d8].map(
  (color) =>
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.4,
      metalness: 0.4,
    }),
);
const jointGeometry = new THREE.SphereGeometry(1, 16, 12);
const segmentGeometry = new THREE.CylinderGeometry(1, 1, 1, 12);
const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xe4e8ee });
const footMaterial = new THREE.MeshStandardMaterial({ color: 0x35d0ba });
const unreachableFootMaterial = new THREE.MeshStandardMaterial({
  color: 0xff6b57,
});

Object.values(LEGS).forEach((leg) => {
  leg.segments = limbMaterials.map((material) => {
    const segment = new THREE.Mesh(segmentGeometry, material);
    robotModel.add(segment);
    return segment;
  });
  leg.joints = [0, 1, 2].map(() => {
    const joint = new THREE.Mesh(jointGeometry, jointMaterial);
    joint.scale.setScalar(7);
    robotModel.add(joint);
    return joint;
  });
  leg.footMesh = new THREE.Mesh(jointGeometry, footMaterial);
  leg.footMesh.scale.setScalar(9);
  robotModel.add(leg.footMesh);
});

// Support polygon and CoM projection (drawn in robot frame, on the ground plane).
const SUPPORT_COLORS = {ok: 0x35d0ba, near: 0xffb347, out: 0xff6b57};
const supportGroup = new THREE.Group();
robotModel.add(supportGroup);
const overlay = {transparent: true, depthWrite: false};
const supportFillGeometry = new THREE.BufferGeometry();
supportFillGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(12), 3));
supportFillGeometry.setIndex([0, 1, 2, 0, 2, 3]);
const supportFill = new THREE.Mesh(supportFillGeometry,
  new THREE.MeshBasicMaterial({...overlay, opacity: 0.18, side: THREE.DoubleSide}));
const supportOutlineGeometry = new THREE.BufferGeometry();
supportOutlineGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(12), 3));
const supportOutline = new THREE.LineLoop(supportOutlineGeometry,
  new THREE.LineBasicMaterial({...overlay, opacity: 0.9}));
const supportLine = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 8),
  new THREE.MeshBasicMaterial({...overlay, opacity: 0.8}));
const comMarker = new THREE.Mesh(new THREE.SphereGeometry(7, 16, 12),
  new THREE.MeshBasicMaterial({color: 0xffffff, depthTest: false, transparent: true}));
const plumbGeometry = new THREE.BufferGeometry();
plumbGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
const plumbLine = new THREE.Line(plumbGeometry,
  new THREE.LineDashedMaterial({...overlay, color: 0xffffff, opacity: 0.6, dashSize: 6, gapSize: 4}));
const comGroundMarker = new THREE.Mesh(new THREE.RingGeometry(6, 10, 24),
  new THREE.MeshBasicMaterial({...overlay, opacity: 0.95, side: THREE.DoubleSide}));
[supportFill, supportOutline, supportLine, comGroundMarker, plumbLine, comMarker]
  .forEach((object, index) => { object.renderOrder = 10 + index; supportGroup.add(object); });

const worldUp = new THREE.Vector3(0, 1, 0);

function positionSegment(mesh, start, end, radius) {
  const direction = end.clone().sub(start);
  const length = direction.length();
  mesh.position.copy(start).addScaledVector(direction, 0.5);
  mesh.quaternion.setFromUnitVectors(worldUp, direction.normalize());
  mesh.scale.set(radius, length, radius);
}

// Obracanie i przybliżanie kamery.
let azimuth = -0.9;
let elevation = 0.45;
let cameraDistance = 560;
const cameraTarget = new THREE.Vector3(40, 0, -40);
const panRight = new THREE.Vector3();
const panUp = new THREE.Vector3();
let dragMode = null;
let activePointerId = null;
let previousPointerX = 0;
let previousPointerY = 0;

canvas.addEventListener("pointerdown", (event) => {
  if (activePointerId !== null || (event.button !== 0 && event.button !== 1)) return;
  event.preventDefault();
  dragMode = event.button === 1 ? "pan" : "rotate";
  activePointerId = event.pointerId;
  previousPointerX = event.clientX;
  previousPointerY = event.clientY;
  canvas.setPointerCapture(event.pointerId);
});
function stopDragging(event) {
  if (event.pointerId !== activePointerId) return;
  dragMode = null;
  activePointerId = null;
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
}
canvas.addEventListener("pointerup", stopDragging);
canvas.addEventListener("pointercancel", stopDragging);
canvas.addEventListener("lostpointercapture", stopDragging);
canvas.addEventListener("auxclick", (event) => {
  if (event.button === 1) event.preventDefault();
});
canvas.addEventListener("pointermove", (event) => {
  if (event.pointerId !== activePointerId) return;

  const deltaX = event.clientX - previousPointerX;
  const deltaY = event.clientY - previousPointerY;
  if (dragMode === "pan") {
    const unitsPerPixel = 2 * cameraDistance * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))
      / canvas.clientHeight;
    panRight.setFromMatrixColumn(camera.matrixWorld, 0);
    panUp.setFromMatrixColumn(camera.matrixWorld, 1);
    cameraTarget.addScaledVector(panRight, -deltaX * unitsPerPixel);
    cameraTarget.addScaledVector(panUp, deltaY * unitsPerPixel);
  } else {
    azimuth -= deltaX * 0.008;
    elevation = clamp(
      elevation + deltaY * 0.008,
      0.05,
      1.5,
    );
  }
  previousPointerX = event.clientX;
  previousPointerY = event.clientY;
});
canvas.addEventListener(
  "wheel",
  (event) => {
    cameraDistance = clamp(cameraDistance * (1 + event.deltaY * 0.001), 200, 1400);
    event.preventDefault();
  },
  { passive: false },
);

function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
}

addEventListener("resize", resize);
resize();


const status = document.getElementById("status");
const keys = new Set();
const parameters = {};
async function loadParameters() {
  const gaitResponse = await fetch("/api/gaits", {signal: AbortSignal.timeout(3000)});
  if (!gaitResponse.ok) throw new Error(`HTTP ${gaitResponse.status}`);
  const gaits = await gaitResponse.json();
  const selector = document.getElementById("gait-select");
  for (const name of gaits.names) selector.add(new Option(name, name));
  selector.value = gaits.default;
  state.gait = state.requested_gait = gaits.default;
  selector.addEventListener("change", () => { state.requested_gait = selector.value; });
  const response = await fetch("/api/parameters", {signal: AbortSignal.timeout(3000)});
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  const specs = await response.json();
  for (const [name, spec] of Object.entries(specs)) {
    if (name === "x_offset_rear" || name === "y_offset_rear") continue;
    parameters[name] = spec.default;
    const row = document.createElement("div");
    row.className = "parameter-row";
    const label = document.createElement("label");
    label.htmlFor = name;
    label.textContent = ({
      x_offset_front: "X offset (front/rear mirrored)",
      y_offset_front: "Y offset (front/rear equal)",
    })[name] || name;
    const output = document.createElement("output");
    output.htmlFor = name;
    const input = document.createElement("input");
    input.type = "range";
    input.id = name;
    input.min = spec.min;
    input.max = spec.max;
    input.step = spec.step;
    input.value = spec.default;
    const update = () => {
      parameters[name] = Number(input.value);
      output.textContent = `${parameters[name]} ${spec.unit}`;
    };
    input.addEventListener("input", update);
    update();
    row.append(label, output, input);
    document.getElementById("gait-parameters").appendChild(row);
  }
}
let state = {phase: 0, ramp: 0, direction: [0, 0], turn: 0, previous_dpad: 0};
let resetVersion = 0;
let previousWorldPosition = null;
const followCamera = document.getElementById("follow-camera");
const showSupport = document.getElementById("show-support");
document.getElementById("reset-position").addEventListener("click", () => {
  resetVersion++;
  delete state.odometry;
  previousWorldPosition = null;
  clearTopView();
  resetMarginChart();
});
followCamera.addEventListener("change", () => {
  if (followCamera.checked && previousWorldPosition) {
    cameraTarget.set(...previousWorldPosition, -40);
  }
  if (followCamera.checked) viewCenter = null;
});
Object.values(LEGS).forEach(leg => {
  const row = document.createElement("tr");
  row.innerHTML = `<td>${leg.n.toUpperCase()}</td>${Array.from({length: 3}, () =>
    '<td class="angle-cell"><span class="angle-before"></span><span class="angle-after"></span><span class="angle-delta"></span></td>'
  ).join("")}`;
  document.querySelector("#tbl tbody").appendChild(row);
  leg.tableRow = row;
  leg.target = new THREE.Mesh(new THREE.SphereGeometry(11, 12, 8),
    new THREE.MeshBasicMaterial({color: 0x35d0ba, wireframe: true}));
  robotModel.add(leg.target);
});
addEventListener("keydown", e => {
  if (["KeyX", "KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "ShiftLeft", "ShiftRight"]
      .includes(e.code) && !e.target.matches("input,button,select")) {
    keys.add(e.code); e.preventDefault();
  }
});
addEventListener("keyup", e => keys.delete(e.code));
addEventListener("blur", () => keys.clear());
document.addEventListener("visibilitychange", () => keys.clear());
function controls() {
  if (document.hidden) return {x: 0, y: 0, right_x: 0, right_y: 0, r1: false, dpad: 0, gait_button: false};
  const pad = [...(navigator.getGamepads?.() || [])].find(p => p && p.connected);
  // Same per-axis deadzone/rescaling as PS4Controller, then the main.py axis swap.
  const axis = v => Math.abs(v) < .15 ? 0 : Math.sign(v) * (Math.abs(v) - .15) / .85;
  return pad ? {x: axis(-(pad.axes[1] ?? 0)), y: axis(pad.axes[0] ?? 0),
                right_x: axis(pad.axes[2] ?? 0), right_y: axis(-(pad.axes[3] ?? 0)),
                r1: !!pad.buttons[5]?.pressed,
                gait_button: !!pad.buttons[0]?.pressed,
                dpad: Number(!!pad.buttons[12]?.pressed) - Number(!!pad.buttons[13]?.pressed)} :
    {x: Number(keys.has("KeyW")) - Number(keys.has("KeyS")),
     y: Number(keys.has("KeyD")) - Number(keys.has("KeyA")),
     right_x: Number(keys.has("ArrowRight")) - Number(keys.has("ArrowLeft")),
     right_y: Number(keys.has("ArrowUp")) - Number(keys.has("ArrowDown")),
     r1: keys.has("ShiftLeft") || keys.has("ShiftRight"),
     gait_button: keys.has("KeyX"),
     dpad: keys.has("ShiftLeft") || keys.has("ShiftRight") ? 0 :
       Number(keys.has("ArrowUp")) - Number(keys.has("ArrowDown"))};
}
function drawFrame(frame) {
  const {position, yaw} = frame.odometry;
  const [cx, cy, cz] = frame.reference_center;
  robotModel.position.set(-cx, -cy, -cz);
  robotRoot.position.set(...position, cz);
  robotRoot.rotation.z = yaw;
  if (followCamera.checked) {
    if (previousWorldPosition) {
      cameraTarget.x += position[0] - previousWorldPosition[0];
      cameraTarget.y += position[1] - previousWorldPosition[1];
    } else {
      cameraTarget.set(...position, -40);
    }
  }
  previousWorldPosition = [...position];
  // Snap by whole cells: the lattice stays fixed as the visible patch follows.
  grid.position.x = Math.round(position[0] / 50) * 50;
  grid.position.y = Math.round(position[1] / 50) * 50;
  originMarker.position.z = frame.ground;
  body.scale.set(...frame.body.size);
  body.position.set(...frame.body.center);
  // Ry(pitch) Rx(roll), matching the shared Python pose calculation.
  const [roll, pitch] = frame.body_rotation.map(THREE.MathUtils.degToRad);
  body.rotation.set(roll, pitch, 0, "YXZ");
  nose.rotation.copy(body.rotation);
  nose.scale.set(12, 30, 18); nose.position.set(...frame.body.nose);
  grid.position.z = frame.ground;
  Object.entries(frame.legs).forEach(([name, result]) => {
    const leg = LEGS[name];
    const points = result.points.map(p => new THREE.Vector3(...p));
    for (let i = 0; i < 3; i++) {
      positionSegment(leg.segments[i], points[i], points[i + 1], [6, 5.5, 4.5][i]);
      leg.joints[i].position.copy(points[i]);
      const cell = leg.tableRow.children[i + 1];
      const jointName = ["coxa", "femur", "tibia"][i];
      const requested = result.requested_angles[jointName];
      const limitedAngle = result.geometric_angles[jointName];
      const delta = limitedAngle - requested;
      cell.querySelector(".angle-before").textContent = requested.toFixed(1);
      cell.querySelector(".angle-after").textContent = `→ ${limitedAngle.toFixed(1)}`;
      cell.querySelector(".angle-delta").textContent = `Δ ${delta > 0 ? "+" : ""}${delta.toFixed(1)}°`;
      cell.classList.toggle("limited", result.limited[i]);
    }
    const limited = result.limited.some(Boolean);
    leg.footMesh.position.copy(points[3]);
    leg.footMesh.material = limited ? unreachableFootMaterial : footMaterial;
    leg.target.position.set(...result.target_world);
  });
  const support = frame.support;
  drawSupport(support);
  drawTopView(frame);
  recordMargin(frame);
  updateStabilityStats();
  drawMarginChart();
  status.hidden = true;  // shown only for connection / simulation errors
}
function drawSupport(support) {
  // Lines and points have no area: always statically unstable (trot is dynamic).
  support.level = support.kind !== "polygon" ? "near"
    : support.margin <= 0 ? "out"
    : support.margin < support.warning_margin ? "near" : "ok";
  supportGroup.visible = showSupport.checked;
  if (!supportGroup.visible) return;
  const color = SUPPORT_COLORS[support.level];
  const ground = support.com_ground[2] + 0.5;
  const vertices = support.polygon;
  const isArea = vertices.length >= 3;
  supportFill.visible = supportOutline.visible = isArea;
  supportLine.visible = vertices.length === 2;
  if (isArea) {
    const fill = supportFillGeometry.attributes.position;
    const outline = supportOutlineGeometry.attributes.position;
    vertices.forEach(([x, y], i) => { fill.setXYZ(i, x, y, ground); outline.setXYZ(i, x, y, ground); });
    fill.needsUpdate = outline.needsUpdate = true;
    supportFillGeometry.setDrawRange(0, vertices.length === 3 ? 3 : 6);
    supportOutlineGeometry.setDrawRange(0, vertices.length);
    supportFillGeometry.computeBoundingSphere();
    supportOutlineGeometry.computeBoundingSphere();
  } else if (supportLine.visible) {
    positionSegment(supportLine, new THREE.Vector3(...vertices[0], ground),
                    new THREE.Vector3(...vertices[1], ground), 2.5);
  }
  supportFill.material.color.setHex(color);
  supportOutline.material.color.setHex(color);
  supportLine.material.color.setHex(color);
  comGroundMarker.material.color.setHex(color);
  comMarker.position.set(...support.com);
  comGroundMarker.position.set(support.com_ground[0], support.com_ground[1], ground + 0.1);
  plumbGeometry.attributes.position.setXYZ(0, ...support.com);
  plumbGeometry.attributes.position.setXYZ(1, support.com_ground[0], support.com_ground[1], ground);
  plumbGeometry.attributes.position.needsUpdate = true;
  plumbGeometry.computeBoundingSphere();
  plumbLine.computeLineDistances();
}

// 2D top view, +X up and +Y (left) to the left. Inset: robot axes, nose up.
// Full screen: world axes from the odometry, so the robot walks and turns as in 3D.
const topCanvas = document.getElementById("topview");
const top2d = topCanvas.getContext("2d");
const panel = document.getElementById("panel");
const cssColor = hex => `#${hex.toString(16).padStart(6, "0")}`;
const VIEW_MODES = ["3d", "inset", "2d"];
let viewMode = "inset";
let insetRange = 280;   // half-size of the inset [mm], eased to fit the feet
let fullRange = 450;    // half of the shorter visible side in full screen [mm]
let viewCenter = null;  // world [x, y] at the center of the full-screen view
let viewScale = 1;      // px per mm of the last full-screen draw
let previousViewPosition = null;
let lastFrame = null;
let trailFrame = null;
const comTrail = [];    // world CoM ground projections

function clearTopView() {
  comTrail.length = 0;
  viewCenter = previousViewPosition = null;
}
function setViewMode(mode) {
  viewMode = mode;
  document.querySelectorAll('input[name="view-mode"]').forEach(input => {
    input.checked = input.value === mode;
  });
  canvas.hidden = mode === "2d";
  topCanvas.hidden = mode === "3d";
  topCanvas.classList.toggle("full", mode === "2d");
  if (lastFrame) drawTopView(lastFrame);
}
document.querySelectorAll('input[name="view-mode"]').forEach(input =>
  input.addEventListener("change", () => { setViewMode(input.value); input.blur(); }));
addEventListener("keydown", event => {
  if (event.code !== "KeyV" || event.repeat || event.target.matches("input,button,select")) return;
  setViewMode(VIEW_MODES[(VIEW_MODES.indexOf(viewMode) + 1) % VIEW_MODES.length]);
});
topCanvas.addEventListener("wheel", event => {
  fullRange = clamp(fullRange * (1 + event.deltaY * 0.001), 150, 5000);
  event.preventDefault();
  if (lastFrame) drawTopView(lastFrame);
}, {passive: false});
let topDrag = null;
topCanvas.addEventListener("pointerdown", event => {
  if (event.button !== 0 && event.button !== 1) return;
  event.preventDefault();
  topDrag = {id: event.pointerId, x: event.clientX, y: event.clientY};
  topCanvas.setPointerCapture(event.pointerId);
});
topCanvas.addEventListener("pointermove", event => {
  if (!topDrag || event.pointerId !== topDrag.id || !viewCenter) return;
  // Screen right is world -Y, screen down is world -X.
  viewCenter[1] += (event.clientX - topDrag.x) / viewScale;
  viewCenter[0] += (event.clientY - topDrag.y) / viewScale;
  topDrag.x = event.clientX; topDrag.y = event.clientY;
  if (lastFrame) drawTopView(lastFrame);
});
["pointerup", "pointercancel", "lostpointercapture"].forEach(type =>
  topCanvas.addEventListener(type, event => {
    if (topDrag && event.pointerId === topDrag.id) topDrag = null;
  }));

function updateTrails(frame, toWorld) {
  const com = toWorld(frame.support.com_ground);
  const last = comTrail[comTrail.length - 1];
  if (!last || Math.hypot(com[0] - last[0], com[1] - last[1]) > 2) comTrail.push(com);
  if (comTrail.length > 1500) comTrail.shift();
}

function drawTopView(frame) {
  lastFrame = frame;
  const [cx, cy] = frame.reference_center;
  const {position, yaw} = frame.odometry;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const toWorld = p => [position[0] + c * (p[0] - cx) - s * (p[1] - cy),
                        position[1] + s * (p[0] - cx) + c * (p[1] - cy)];
  if (frame !== trailFrame) { updateTrails(frame, toWorld); trailFrame = frame; }
  if (viewMode === "3d") return;
  const full = viewMode === "2d";
  const ratio = Math.min(devicePixelRatio, 2);
  const width = topCanvas.clientWidth, height = topCanvas.clientHeight;
  if (topCanvas.width !== Math.round(width * ratio) || topCanvas.height !== Math.round(height * ratio)) {
    topCanvas.width = Math.round(width * ratio);
    topCanvas.height = Math.round(height * ratio);
  }
  const g = top2d;
  g.setTransform(ratio, 0, 0, ratio, 0, 0);
  g.clearRect(0, 0, width, height);
  const support = frame.support;
  // Robot points -> view coordinates (world in full screen, robot-relative in the inset).
  const toView = full ? toWorld : p => [p[0] - cx, p[1] - cy];
  let center, scale, screenX, screenY;
  if (full) {
    if (!viewCenter) viewCenter = [...position];
    else if (followCamera.checked && previousViewPosition) {
      viewCenter[0] += position[0] - previousViewPosition[0];
      viewCenter[1] += position[1] - previousViewPosition[1];
    }
    previousViewPosition = [...position];
    // Keep the robot centered in the area left of the panel.
    const visibleWidth = Math.max(200, width - panel.offsetWidth - 24);
    center = viewCenter;
    scale = viewScale = Math.min(visibleWidth, height) / 2 / fullRange;
    screenX = visibleWidth / 2; screenY = height / 2;
  } else {
    previousViewPosition = null;
    const extent = Math.max(...Object.values(frame.legs).flatMap(leg =>
      leg.points.map(p => Math.max(Math.abs(p[0] - cx), Math.abs(p[1] - cy)))));
    insetRange += (Math.max(200, extent * 1.4 + 30) - insetRange) * 0.05;
    center = [0, 0];
    scale = Math.min(width, height) / 2 / insetRange;
    screenX = width / 2; screenY = height / 2 + 10;  // room for the header
  }
  const sx = q => screenX - (q[1] - center[1]) * scale;
  const sy = q => screenY - (q[0] - center[0]) * scale;
  const path = (points, close, map = toView) => {
    g.beginPath();
    points.forEach((p, i) => { const q = map(p); (i ? g.lineTo : g.moveTo).call(g, sx(q), sy(q)); });
    if (close) g.closePath();
  };
  const view = q => q;
  // Grid in view axes; the spacing grows when zoomed out.
  const spacing = [50, 100, 250, 500, 1000].find(mm => mm * scale >= 18) || 1000;
  const reach = Math.max(width, height) / scale;
  g.lineWidth = 1;
  for (let axis = 0; axis < 2; axis++) {
    const from = Math.floor((center[axis] - reach) / spacing) * spacing;
    for (let mm = from; mm <= center[axis] + reach; mm += spacing) {
      g.strokeStyle = Math.abs(mm) < 1e-6 ? "#3a4452" : "#232a34";
      const a = [0, 0], b = [0, 0];
      a[axis] = b[axis] = mm;
      a[1 - axis] = center[1 - axis] - reach; b[1 - axis] = center[1 - axis] + reach;
      path([a, b], false, view); g.stroke();
    }
  }
  // Body (roll/pitch ignored in plan view) and nose.
  const [bx, by] = frame.body.center;
  const [lx, ly] = frame.body.size;
  g.fillStyle = "#2b3441"; g.strokeStyle = "#5f7794"; g.lineWidth = 1;
  path([[bx + lx / 2, by + ly / 2], [bx + lx / 2, by - ly / 2],
        [bx - lx / 2, by - ly / 2], [bx - lx / 2, by + ly / 2]], true);
  g.fill(); g.stroke();
  g.fillStyle = cssColor(SUPPORT_COLORS.ok);
  const [nx, ny] = frame.body.nose;
  path([[nx + 6, ny + 9], [nx + 6, ny - 9], [nx - 6, ny - 9], [nx - 6, ny + 9]], true); g.fill();
  // Legs: segments, then feet (filled = contact, ring = swing, red = servo limit).
  const middle = toView([cx, cy]);
  const labelSize = full ? 12 : 10;
  Object.entries(frame.legs).forEach(([name, leg]) => {
    g.strokeStyle = "#8fa3bd"; g.lineWidth = full ? 4 : 3;
    path(leg.points); g.stroke();
    const foot = toView(leg.points[3]);
    const contact = support.feet.includes(name);
    const color = leg.limited.some(Boolean) ? cssColor(SUPPORT_COLORS.out) : "#e4e8ee";
    g.beginPath(); g.arc(sx(foot), sy(foot), full ? 6 : 5, 0, 2 * Math.PI);
    g.lineWidth = 2; g.strokeStyle = color; g.fillStyle = color;
    contact ? g.fill() : g.stroke();
    // Label outward from the body center, in screen space (works when rotated).
    const outX = Math.sign(sx(foot) - sx(middle)) || 1, outY = Math.sign(sy(foot) - sy(middle)) || 1;
    g.fillStyle = "#8a94a3"; g.font = `${labelSize}px Segoe UI, system-ui, sans-serif`;
    g.textAlign = outX > 0 ? "left" : "right";
    g.fillText(name.toUpperCase(), sx(foot) + outX * 9, sy(foot) + outY * 9 + labelSize / 3);
    g.textAlign = "left";
  });
  // Support polygon / line above the body.
  const color = cssColor(SUPPORT_COLORS[support.level]);
  if (support.polygon.length >= 2) {
    path(support.polygon, support.polygon.length >= 3);
    if (support.polygon.length >= 3) {
      g.globalAlpha = 0.22; g.fillStyle = color; g.fill(); g.globalAlpha = 1;
    }
    g.lineWidth = support.polygon.length === 2 ? 4 : 2; g.strokeStyle = color; g.stroke();
  }
  // Path of the CoM projection, above the body.
  if (full && comTrail.length > 1) {
    g.strokeStyle = "rgba(53, 208, 186, 0.6)"; g.lineWidth = 1.5;
    path(comTrail, false, view); g.stroke();
  }
  // CoM projection: crosshair with a ring in the stability color.
  const com = toView(support.com_ground);
  const comX = sx(com), comY = sy(com), mark = full ? 1.3 : 1;
  g.strokeStyle = color; g.lineWidth = 2.5;
  g.beginPath(); g.arc(comX, comY, 8 * mark, 0, 2 * Math.PI); g.stroke();
  g.strokeStyle = "#ffffff"; g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(comX - 12 * mark, comY); g.lineTo(comX + 12 * mark, comY);
  g.moveTo(comX, comY - 12 * mark); g.lineTo(comX, comY + 12 * mark); g.stroke();
  // Readout.
  const big = full ? 18 : 12, small = full ? 13 : 11, left = full ? 24 : 12, top = full ? 34 : 20;
  g.font = `600 ${big}px Segoe UI, system-ui, sans-serif`; g.fillStyle = color;
  g.fillText(formatReading(stabilityReading(support)), left, top);
  g.font = `${small}px Segoe UI, system-ui, sans-serif`; g.fillStyle = "#8a94a3";
  g.fillText(`${support.feet.length} legs · ${support.kind} · ${frame.gait} · grid ${spacing} mm`,
             left, top + small + 6);
  if (full) g.fillText("Wheel: zoom · drag: pan · V: switch view", left, height - 20);
}
// One number per moment. Gaits with two legs swinging together (trot): distance
// of the CoM from the support line on 2 legs. Others (crawl): signed margin
// inside the triangle on 3 legs. On 4 legs the robot is comfortably supported: "—".
function stabilityReading(support) {
  if (support.feet.length >= 4) return {label: "4 legs, CoM", value: null};
  if (support.kind === "line") return {label: "CoM to line", value: Math.abs(support.margin)};
  if (support.kind === "polygon") return {label: "CoM margin", value: support.margin};
  return {label: "CoM", value: null};
}
const formatReading = reading =>
  `${reading.label} ${reading.value === null ? "—" : `${reading.value.toFixed(1)} mm`}`;

// Chart over one gait cycle (x = gait phase, so leg exchanges line up).
const marginChart = document.getElementById("margin-chart");
const chart2d = marginChart.getContext("2d");
const CHART_MODES = {
  line: {title: "CoM distance from line vs gait phase", range: [0, 40], ticks: [0, 10, 20, 30, 40],
         keep: s => s.kind === "line",
         // On 4 legs nothing is at risk: plot 0 so the curve stays continuous.
         plotValue: s => (s.kind === "line" ? s.value : s.legs >= 4 ? 0 : null)},
  polygon: {title: "CoM margin (3 legs) vs gait phase", range: [-20, 80], ticks: [-20, 0, 20, 40, 60, 80],
            keep: s => s.kind === "polygon" && s.legs === 3,
            plotValue: s => (s.kind === "polygon" && s.legs === 3 ? s.value : null)},
};
let cycleSamples = [], previousCycle = [], lastSamplePhase = null, chartGait = null;
let chartMode = "polygon";
let chartHover = null;  // hovered phase, or null

function resetMarginChart() {
  cycleSamples = []; previousCycle = []; lastSamplePhase = null;
}
function recordMargin(frame) {
  const support = frame.support;
  if (frame.gait !== chartGait) { resetMarginChart(); chartGait = frame.gait; }
  chartMode = support.metric;
  if (!(frame.ramp > 0)) return;  // standing: nothing to plot
  const phase = frame.sample_phase;
  if (lastSamplePhase !== null && phase < lastSamplePhase - 0.5) {
    // Wrapped into a new cycle; keep it as "previous" only if it was (almost) complete.
    const span = cycleSamples.length ? cycleSamples.at(-1).phase - cycleSamples[0].phase : 0;
    if (span >= 0.9) previousCycle = cycleSamples;
    cycleSamples = [];
  }
  lastSamplePhase = phase;
  cycleSamples.push({phase, value: stabilityReading(support).value, kind: support.kind,
                     legs: support.feet.length, warning: support.warning_margin});
}
function updateStabilityStats() {
  const complete = previousCycle.length > 0;
  const samples = complete ? previousCycle : cycleSamples;
  const set = (id, text) => { document.getElementById(id).textContent = text; };
  set("chart-title", CHART_MODES[chartMode].title);
  set("stats-caption", complete ? "Last full cycle"
    : samples.length ? "Current cycle (in progress)" : "Walk to collect data");
  const share = count => samples.length ? `${(100 * count / samples.length).toFixed(0)} %` : "—";
  const kept = samples.filter(CHART_MODES[chartMode].keep);
  const pick = better => kept.reduce((best, s) => (!best || better(s, best) ? s : best), null);
  const cell = document.getElementById("stat-a");
  if (chartMode === "line") {
    const worst = pick((s, best) => s.value > best.value);
    set("stat-a-label", "Max CoM distance from line");
    set("stat-a", worst ? `${worst.value.toFixed(1)} mm` : "—");
    set("stat-c-label", "Cycle on 2 legs");
    set("stat-c", share(kept.length));
    cell.className = "";
  } else {
    const worst = pick((s, best) => s.value < best.value);
    set("stat-a-label", "Min CoM margin (3 legs)");
    set("stat-a", worst ? `${worst.value.toFixed(1)} mm` : "—");
    set("stat-c-label", "Cycle with margin < 10 mm");
    set("stat-c", share(kept.filter(s => s.value < s.warning).length));
    cell.className = !worst ? "" : worst.value <= 0 ? "out" : worst.value < worst.warning ? "near" : "ok";
  }
}
function drawMarginChart() {
  const ratio = Math.min(devicePixelRatio, 2);
  const width = marginChart.clientWidth, height = marginChart.clientHeight;
  if (!width) return;
  if (marginChart.width !== Math.round(width * ratio) || marginChart.height !== Math.round(height * ratio)) {
    marginChart.width = Math.round(width * ratio);
    marginChart.height = Math.round(height * ratio);
  }
  const g = chart2d;
  g.setTransform(ratio, 0, 0, ratio, 0, 0);
  g.clearRect(0, 0, width, height);
  const mode = CHART_MODES[chartMode];
  const left = 30, right = 8, top = 8, bottom = 20;
  const plotW = width - left - right, plotH = height - top - bottom;
  const [lo, hi] = mode.range;
  const px = phase => left + phase * plotW;
  const py = mm => top + (hi - clamp(mm, lo, hi)) / (hi - lo) * plotH;
  g.font = "10px Segoe UI, system-ui, sans-serif";
  if (chartMode === "polygon") {
    // Area below zero = CoM outside the support triangle.
    g.fillStyle = "rgba(255, 107, 87, 0.08)";
    g.fillRect(left, py(0), plotW, py(lo) - py(0));
  }
  // Recessive grid and axis labels.
  g.lineWidth = 1; g.fillStyle = "#8a94a3";
  g.textAlign = "right"; g.textBaseline = "middle";
  for (const mm of mode.ticks) {
    g.strokeStyle = "#232a34";
    g.beginPath(); g.moveTo(left, py(mm)); g.lineTo(left + plotW, py(mm)); g.stroke();
    g.fillText(String(mm), left - 5, py(mm));
  }
  g.textAlign = "center"; g.textBaseline = "top";
  for (const phase of [0, 0.25, 0.5, 0.75, 1]) {
    g.strokeStyle = "#232a34";
    g.beginPath(); g.moveTo(px(phase), top); g.lineTo(px(phase), top + plotH); g.stroke();
    g.fillText(String(phase), px(phase), top + plotH + 5);
  }
  const warning = (cycleSamples[0] || previousCycle[0])?.warning ?? 10;
  if (chartMode === "line") {
    // Gray bands: phases on 4 legs, where the plotted 0 means "supported".
    g.fillStyle = "rgba(143, 163, 189, 0.10)";
    let run = null;
    for (const s of [...cycleSamples, null]) {
      if (s && s.legs >= 4) { run = run ? [run[0], s.phase] : [s.phase, s.phase]; continue; }
      if (run) g.fillRect(px(run[0]), top, Math.max(2, px(run[1]) - px(run[0])), plotH);
      run = null;
    }
  }
  if (chartMode === "polygon") {
    // Reference lines: 0 (edge of the triangle) and the warning margin.
    g.strokeStyle = cssColor(SUPPORT_COLORS.out);
    g.beginPath(); g.moveTo(left, py(0)); g.lineTo(left + plotW, py(0)); g.stroke();
    g.strokeStyle = cssColor(SUPPORT_COLORS.near); g.setLineDash([4, 3]);
    g.beginPath(); g.moveTo(left, py(warning)); g.lineTo(left + plotW, py(warning)); g.stroke();
    g.setLineDash([]);
    g.textAlign = "right"; g.textBaseline = "bottom"; g.fillStyle = "#8a94a3";
    g.fillText(`${warning} mm`, left + plotW - 2, py(warning) - 1);
  }
  // Only the moments the metric describes; gaps where it has no value.
  const visible = s => mode.plotValue(s) !== null;
  const plot = (samples, color, lineWidth) => {
    g.strokeStyle = color; g.lineWidth = lineWidth; g.lineJoin = "round";
    g.beginPath();
    let previous = null;
    for (const s of samples) {
      if (!visible(s)) { previous = null; continue; }
      if (previous && s.phase - previous.phase < 0.05) g.lineTo(px(s.phase), py(mode.plotValue(s)));
      else g.moveTo(px(s.phase), py(mode.plotValue(s)));
      previous = s;
    }
    g.stroke();
  };
  plot(cycleSamples, cssColor(SUPPORT_COLORS.ok), 2);
  const latest = cycleSamples.at(-1);
  if (latest && visible(latest)) {
    g.fillStyle = cssColor(SUPPORT_COLORS.ok); g.strokeStyle = "#161b23"; g.lineWidth = 2;
    g.beginPath(); g.arc(px(latest.phase), py(mode.plotValue(latest)), 4, 0, 2 * Math.PI); g.fill(); g.stroke();
  }
  // Hover: crosshair + readout of the nearest sample of the current cycle.
  if (chartHover !== null) {
    const nearest = samples => samples.filter(s => visible(s) && Math.abs(s.phase - chartHover) < 0.02)
      .reduce((best, s) => (!best || Math.abs(s.phase - chartHover) < Math.abs(best.phase - chartHover)
                            ? s : best), null);
    const sample = nearest(cycleSamples);
    const x = px(chartHover);
    g.strokeStyle = "#8a94a3"; g.lineWidth = 1;
    g.beginPath(); g.moveTo(x, top); g.lineTo(x, top + plotH); g.stroke();
    let text = `phase ${chartHover.toFixed(2)} · 4 legs or no data`;
    if (sample) {
      g.fillStyle = "#e4e8ee"; g.strokeStyle = "#161b23"; g.lineWidth = 2;
      g.beginPath(); g.arc(px(sample.phase), py(mode.plotValue(sample)), 4, 0, 2 * Math.PI); g.fill(); g.stroke();
      text = sample.legs >= 4 ? `phase ${sample.phase.toFixed(2)} · 4 legs, supported`
        : `phase ${sample.phase.toFixed(2)} · ${sample.value.toFixed(1)} mm · ${sample.legs} legs`;
    }
    g.font = "11px Segoe UI, system-ui, sans-serif";
    const boxW = g.measureText(text).width + 12;
    const boxX = clamp(x - boxW / 2, left, left + plotW - boxW);
    g.fillStyle = "rgba(14, 17, 22, 0.92)"; g.fillRect(boxX, top, boxW, 18);
    g.fillStyle = "#e4e8ee"; g.textAlign = "left"; g.textBaseline = "middle";
    g.fillText(text, boxX + 6, top + 9);
  }
  g.textAlign = "left"; g.textBaseline = "alphabetic";
}
marginChart.addEventListener("pointermove", event => {
  const rect = marginChart.getBoundingClientRect();
  chartHover = clamp((event.clientX - rect.left - 30) / (rect.width - 38), 0, 1);
  drawMarginChart();
});
marginChart.addEventListener("pointerleave", () => { chartHover = null; drawMarginChart(); });

// One request at a time; each tab owns its motion state. Fixed 20 ms robot timestep.
async function tick() {
  const started = performance.now();
  const requestVersion = resetVersion;
  const requestedGait = state.requested_gait;
  try {
    const response = await fetch("/api/frame", {method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({...state, ...controls(), ...parameters, elapsed: .02}),
      signal: AbortSignal.timeout(3000)});
    const frame = await response.json();
    if (!response.ok) throw new Error(frame.error || `HTTP ${response.status}`);
    if (requestVersion !== resetVersion) return;
    const latestRequestedGait = state.requested_gait;
    state = {odometry: frame.odometry,
             gait: frame.gait,
             requested_gait: latestRequestedGait !== requestedGait ? latestRequestedGait : frame.requested_gait,
             previous_gait_button: frame.previous_gait_button,
             phase: frame.phase, ramp: frame.ramp, direction: frame.direction, turn: frame.turn,
             previous_dpad: frame.previous_dpad};
    document.getElementById("gait-select").value = state.requested_gait;
    // Keep the slider and its label in sync with D-pad/keyboard height changes.
    if (frame.z_height !== parameters.z_height) {
      const input = document.getElementById("z_height");
      input.value = frame.z_height;
      input.dispatchEvent(new Event("input"));
    }
    drawFrame(frame);
  } catch (error) {
    keys.clear(); status.className = "warning"; status.hidden = false;
    status.textContent = `Simulation error: ${error.message}`;
  } finally {
    setTimeout(tick, Math.max(0, 20 - (performance.now() - started)));
  }
}
function animate() {
  camera.position.set(cameraTarget.x + cameraDistance * Math.cos(elevation) * Math.cos(azimuth),
    cameraTarget.y + cameraDistance * Math.cos(elevation) * Math.sin(azimuth),
    cameraTarget.z + cameraDistance * Math.sin(elevation));
  camera.lookAt(cameraTarget);
  if (!canvas.hidden) renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
loadParameters().then(tick).catch(error => {
  status.className = "warning"; status.hidden = false;
  status.textContent = `Simulation error: ${error.message}. Reload to retry.`;
});
animate();

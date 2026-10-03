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

const grid = new THREE.GridHelper(1200, 24, 0x35d0ba, 0x262d38);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

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
scene.add(body, nose);

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
    scene.add(segment);
    return segment;
  });
  leg.joints = [0, 1, 2].map(() => {
    const joint = new THREE.Mesh(jointGeometry, jointMaterial);
    joint.scale.setScalar(7);
    scene.add(joint);
    return joint;
  });
  leg.footMesh = new THREE.Mesh(jointGeometry, footMaterial);
  leg.footMesh.scale.setScalar(9);
  scene.add(leg.footMesh);
});

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
  const response = await fetch("/api/parameters", {signal: AbortSignal.timeout(3000)});
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  const specs = await response.json();
  for (const [name, spec] of Object.entries(specs)) {
    parameters[name] = spec.default;
    const row = document.createElement("div");
    row.className = "parameter-row";
    const label = document.createElement("label");
    label.htmlFor = name;
    label.textContent = name;
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
let state = {phase: 0, ramp: 0, direction: [0, 0]};
Object.values(LEGS).forEach(leg => {
  const row = document.createElement("tr");
  row.innerHTML = `<td>${leg.n.toUpperCase()}</td><td></td><td></td><td></td>`;
  document.querySelector("#tbl tbody").appendChild(row);
  leg.tableRow = row;
  leg.target = new THREE.Mesh(new THREE.SphereGeometry(11, 12, 8),
    new THREE.MeshBasicMaterial({color: 0x35d0ba, wireframe: true}));
  scene.add(leg.target);
});
addEventListener("keydown", e => {
  if (["KeyW", "KeyA", "KeyS", "KeyD"].includes(e.code) && !e.target.matches("input,button")) {
    keys.add(e.code); e.preventDefault();
  }
});
addEventListener("keyup", e => keys.delete(e.code));
addEventListener("blur", () => keys.clear());
document.addEventListener("visibilitychange", () => keys.clear());
function controls() {
  if (document.hidden) return {x: 0, y: 0};
  const pad = [...(navigator.getGamepads?.() || [])].find(p => p && p.connected);
  // Same per-axis deadzone/rescaling as PS4Controller, then the main.py axis swap.
  const axis = v => Math.abs(v) < .15 ? 0 : Math.sign(v) * (Math.abs(v) - .15) / .85;
  return pad ? {x: axis(-pad.axes[1]), y: axis(pad.axes[0])} :
    {x: Number(keys.has("KeyW")) - Number(keys.has("KeyS")),
     y: Number(keys.has("KeyD")) - Number(keys.has("KeyA"))};
}
function drawFrame(frame) {
  const origins = Object.values(frame.origins);
  const xs = origins.map(p => p[0]), ys = origins.map(p => p[1]);
  const cx = (Math.min(...xs) + Math.max(...xs)) / 2;
  body.scale.set(Math.max(...xs) - Math.min(...xs) + 30,
    Math.max(...ys) - Math.min(...ys), 30);
  body.position.set(cx, 0, 0);
  nose.scale.set(12, 30, 18); nose.position.set(Math.max(...xs) + 15, 0, 0);
  grid.position.z = frame.ground;
  let limitedCount = 0;
  Object.entries(frame.legs).forEach(([name, result]) => {
    const leg = LEGS[name];
    const points = result.points.map(p => new THREE.Vector3(...p));
    for (let i = 0; i < 3; i++) {
      positionSegment(leg.segments[i], points[i], points[i + 1], [6, 5.5, 4.5][i]);
      leg.joints[i].position.copy(points[i]);
      const cell = leg.tableRow.children[i + 1];
      cell.textContent = result.servo_angles[["coxa", "femur", "tibia"][i]].toFixed(1);
      cell.style.color = result.limited[i] ? "var(--warning)" : "";
    }
    const limited = result.limited.some(Boolean);
    limitedCount += Number(limited);
    leg.footMesh.position.copy(points[3]);
    leg.footMesh.material = limited ? unreachableFootMaterial : footMaterial;
    leg.target.position.set(...result.target_world);
  });
  status.className = limitedCount ? "warning" : "";
  status.textContent = `Phase ${frame.phase.toFixed(2)}
Ramp ${(frame.ramp * 100).toFixed(0)}%`;
}
// One request at a time; each tab owns its motion state. Fixed 20 ms robot timestep.
async function tick() {
  const started = performance.now();
  try {
    const response = await fetch("/api/frame", {method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({...state, ...controls(), ...parameters, elapsed: .02}),
      signal: AbortSignal.timeout(3000)});
    const frame = await response.json();
    if (!response.ok) throw new Error(frame.error || `HTTP ${response.status}`);
    state = {phase: frame.phase, ramp: frame.ramp, direction: frame.direction};
    drawFrame(frame);
  } catch (error) {
    keys.clear(); status.className = "warning";
    status.textContent = `Simulation error: ${error.message}`;
  }
  setTimeout(tick, Math.max(0, 20 - (performance.now() - started)));
}
function animate() {
  camera.position.set(cameraTarget.x + cameraDistance * Math.cos(elevation) * Math.cos(azimuth),
    cameraTarget.y + cameraDistance * Math.cos(elevation) * Math.sin(azimuth),
    cameraTarget.z + cameraDistance * Math.sin(elevation));
  camera.lookAt(cameraTarget);
  renderer.render(scene, camera); requestAnimationFrame(animate);
}
loadParameters().then(tick).catch(error => {
  status.className = "warning";
  status.textContent = `Simulation error: ${error.message}. Reload to retry.`;
});
animate();

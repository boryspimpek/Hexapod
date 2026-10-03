// Parametry modelu i sterowania.
const P = {
  L: 200,
  W: 120,
  H: 40,
  coxa: 43,
  femur: 100,
  tibia: 149,
  stand: 40,
  reach: 121,
};
const O = { x: 0, y: 0, z: 0, yaw: 0 };
const RANGE = { x: 70, y: 70, z: 50, yaw: 25 };
const LEGS = [
  { n: "FL", sx: 1, sy: 1, m: Math.PI / 2, inv: [false, false, false] },
  { n: "FR", sx: 1, sy: -1, m: -Math.PI / 2, inv: [false, true, true] },
  { n: "RL", sx: -1, sy: 1, m: Math.PI / 2, inv: [false, false, false] },
  { n: "RR", sx: -1, sy: -1, m: -Math.PI / 2, inv: [false, true, true] },
];

const degrees = (radians) => (radians * 180) / Math.PI;
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

// Analityczne odwrotne zadanie kinematyki dla jednej nogi.
function solveLeg(leg, stance, gaitOffset) {
  const hip = new THREE.Vector3((leg.sx * P.L) / 2, (leg.sy * P.W) / 2, 0);
  const stride = P.coxa + P.reach;
  const yaw = (stance.yaw * Math.PI) / 180;
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  const baseX = hip.x + Math.cos(leg.m) * stride;
  const baseY = hip.y + Math.sin(leg.m) * stride;
  const targetX =
    baseX * cosYaw - baseY * sinYaw + stance.x + gaitOffset.x;
  const targetY =
    baseX * sinYaw + baseY * cosYaw + stance.y + gaitOffset.y;
  const targetZ = -P.stand + stance.z + gaitOffset.z;
  const vx = targetX - hip.x;
  const vy = targetY - hip.y;
  const vz = targetZ;
  const cosMount = Math.cos(leg.m);
  const sinMount = Math.sin(leg.m);
  const localX = vx * cosMount + vy * sinMount;
  const localY = -vx * sinMount + vy * cosMount;
  const gamma = Math.atan2(localY, localX);
  const distanceFromCoxa = Math.hypot(localX, localY) - P.coxa;
  const femur = P.femur;
  const tibia = P.tibia;
  let distance = Math.hypot(distanceFromCoxa, vz);
  let reachable = true;
  const clampedDistance = clamp(
    distance,
    Math.abs(femur - tibia) + 1,
    femur + tibia - 0.5,
  );

  if (clampedDistance !== distance) {
    reachable = false;
  }
  distance = clampedDistance;

  const femurAngle =
    Math.atan2(vz, distanceFromCoxa) +
    Math.acos(
      clamp(
        (femur ** 2 + distance ** 2 - tibia ** 2) / (2 * femur * distance),
        -1,
        1,
      ),
    );
  const kneeAngle = Math.acos(
    clamp(
      (femur ** 2 + tibia ** 2 - distance ** 2) / (2 * femur * tibia),
      -1,
      1,
    ),
  );
  const tibiaAngle = femurAngle - (Math.PI - kneeAngle);
  const direction = new THREE.Vector3(
    Math.cos(leg.m + gamma),
    Math.sin(leg.m + gamma),
    0,
  );
  const coxaJoint = hip.clone().addScaledVector(direction, P.coxa);
  const femurJoint = coxaJoint
    .clone()
    .addScaledVector(direction, femur * Math.cos(femurAngle))
    .add(new THREE.Vector3(0, 0, femur * Math.sin(femurAngle)));
  const foot = femurJoint
    .clone()
    .addScaledVector(direction, tibia * Math.cos(tibiaAngle))
    .add(new THREE.Vector3(0, 0, tibia * Math.sin(tibiaAngle)));
  const angles = [
    degrees(gamma) + 90,
    degrees(femurAngle) + 90,
    -(degrees(kneeAngle) - 180 + 25),
  ];

  return {
    hip,
    coxaJoint,
    femurJoint,
    foot,
    reachable,
    angles: angles.map((angle, index) =>
      leg.inv[index] ? 180 - angle : angle,
    ),
  };
}

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

LEGS.forEach((leg) => {
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

// Kontrolki panelu.
function createSlider(host, target, key, label, min, max, step, unit, onChange) {
  const row = document.createElement("div");
  row.className = "row";
  row.innerHTML = `<span>${label}</span><input type="range" min="${min}" max="${max}" step="${step}" value="${target[key]}"><span></span>`;

  const input = row.querySelector("input");
  const value = row.lastElementChild;
  const update = () => {
    target[key] = Number(input.value);
    value.textContent = input.value + unit;
    onChange?.();
  };

  input.addEventListener("input", update);
  value.textContent = target[key] + unit;
  row.setValue = (nextValue) => {
    input.value = nextValue;
    value.textContent = Math.round(nextValue) + unit;
  };
  host.appendChild(row);
  return row;
}

const geometryControls = document.getElementById("g1");
const footControls = document.getElementById("g2");
[
  ["coxa", "coxa", 10, 80],
  ["femur", "femur", 40, 140],
  ["tibia", "tibia", 40, 180],
  ["stand", "wys. stania", 10, 200],
  ["reach", "wysunięcie", 0, 200],
  ["L", "korpus dł.", 100, 320],
  ["W", "korpus szer.", 60, 200],
  ["H", "korpus wys.", 20, 80],
].forEach(([key, label, min, max]) =>
  createSlider(geometryControls, P, key, label, min, max, 1, "", buildBody),
);

const xSlider = createSlider(
  footControls,
  O,
  "x",
  "X (przód)",
  -RANGE.x,
  RANGE.x,
  1,
  "",
);
const ySlider = createSlider(
  footControls,
  O,
  "y",
  "Y (lewo)",
  -RANGE.y,
  RANGE.y,
  1,
  "",
);
const zSlider = createSlider(
  footControls,
  O,
  "z",
  "Z (góra)",
  -RANGE.z,
  RANGE.z,
  1,
  "",
);
const yawSlider = createSlider(
  footControls,
  O,
  "yaw",
  "Yaw",
  -RANGE.yaw,
  RANGE.yaw,
  1,
  "°",
);

document.getElementById("reset").addEventListener("click", () => {
  O.x = 0;
  O.y = 0;
  O.z = 0;
  O.yaw = 0;
  [xSlider, ySlider, zSlider, yawSlider].forEach((slider) =>
    slider.setValue(0),
  );
});

const angleTableBody = document.querySelector("#tbl tbody");
LEGS.forEach((leg) => {
  const row = document.createElement("tr");
  row.innerHTML = `<td>${leg.n}</td><td></td><td></td><td></td>`;
  angleTableBody.appendChild(row);
  leg.tableRow = row;
});

function buildBody() {
  body.scale.set(P.L, P.W, P.H);
  nose.scale.set(24, P.W * 0.5, P.H * 0.6);
  nose.position.set(P.L / 2 + 8, 0, 0);
}

buildBody();

// Obracanie i przybliżanie kamery.
let azimuth = -0.9;
let elevation = 0.45;
let cameraDistance = 560;
let isDragging = false;
let previousPointerX = 0;
let previousPointerY = 0;

canvas.addEventListener("pointerdown", (event) => {
  isDragging = true;
  previousPointerX = event.clientX;
  previousPointerY = event.clientY;
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener("pointerup", () => {
  isDragging = false;
});
canvas.addEventListener("pointermove", (event) => {
  if (!isDragging) return;

  azimuth -= (event.clientX - previousPointerX) * 0.008;
  elevation = clamp(
    elevation + (event.clientY - previousPointerY) * 0.008,
    0.05,
    1.5,
  );
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

// Odczyt pada oraz klawiatury.
const padIndicator = document.getElementById("dot");
const padName = document.getElementById("padname");
const leftStick = document.querySelector("#sl i");
const rightStick = document.querySelector("#sr i");
const applyDeadZone = (value) => (Math.abs(value) < 0.08 ? 0 : value);
const keys = {};
addEventListener("keydown", (event) => (keys[event.code] = 1));
addEventListener("keyup", (event) => (keys[event.code] = 0));

const gaitToggle = document.getElementById("gaitOn");
const gaitSettings = { stepLength: 40, stepHeight: 40, frequency: 2.2 };
const gaitControls = document.getElementById("g3");
createSlider(
  gaitControls,
  gaitSettings,
  "stepLength",
  "dł. kroku",
  20,
  110,
  1,
  "",
);
createSlider(
  gaitControls,
  gaitSettings,
  "stepHeight",
  "wys. kroku",
  10,
  60,
  1,
  "",
);
createSlider(
  gaitControls,
  gaitSettings,
  "frequency",
  "częstotl.",
  0.1,
  4,
  0.1,
  " Hz",
);

function updateStickLabels() {
  document.querySelector("#sl b").textContent = gaitToggle.checked
    ? "L: ruch"
    : "L: X/Y stóp";
  document.querySelector("#sr b").textContent = gaitToggle.checked
    ? "R: skręt."
    : "R: wys./yaw";
}

gaitToggle.addEventListener("change", updateStickLabels);
updateStickLabels();

let previousOptionsButton = false;

function readControls() {
  const gamepad = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find(
    (candidate) => candidate && candidate.connected,
  );
  let leftX = (keys.KeyD ? 1 : 0) - (keys.KeyA ? 1 : 0);
  let leftY = (keys.KeyS ? 1 : 0) - (keys.KeyW ? 1 : 0);
  let rightX = (keys.KeyE ? 1 : 0) - (keys.KeyQ ? 1 : 0);
  let rightY = 0;

  if (!gamepad) {
    padIndicator.classList.remove("on");
    padName.textContent = "Brak pada — naciśnij przycisk (lub WASD/QE)";
  } else {
    padIndicator.classList.add("on");
    padName.textContent = gamepad.id.slice(0, 38);
    leftX = applyDeadZone(gamepad.axes[0]);
    leftY = applyDeadZone(gamepad.axes[1]);
    rightX = applyDeadZone(gamepad.axes[2]);
    rightY = applyDeadZone(gamepad.axes[3]);

    const optionsPressed = Boolean(
      gamepad.buttons[9] && gamepad.buttons[9].pressed,
    );
    if (optionsPressed && !previousOptionsButton) {
      gaitToggle.checked = !gaitToggle.checked;
      updateStickLabels();
    }
    previousOptionsButton = optionsPressed;
  }

  leftStick.style.transform = `translate(${leftX * 26}px, ${leftY * 26}px)`;
  rightStick.style.transform = `translate(${rightX * 26}px, ${rightY * 26}px)`;

  return {
    leftX,
    leftY,
    rightX,
    rightY,
    x: -leftY * RANGE.x,
    y: -leftX * RANGE.y,
    z: -rightY * RANGE.z,
    yaw: -rightX * RANGE.yaw,
  };
}

// Pętla animacji chodu i renderowania.
let frame = 0;
let phase = 0;
let previousFrameTime = 0;
const smoothedVelocity = { x: 0, y: 0, yaw: 0 };
LEGS.forEach((leg) => {
  leg.phase = leg.sx * leg.sy > 0 ? 0 : 0.5;
});

function animate(now) {
  const deltaTime = Math.min(
    0.05,
    (now - (previousFrameTime || now)) / 1000,
  );
  previousFrameTime = now;

  const controls = readControls();
  const gaitEnabled = gaitToggle.checked;
  const stance = gaitEnabled
    ? { x: O.x, y: O.y, z: O.z, yaw: O.yaw }
    : {
        x: O.x + controls.x,
        y: O.y + controls.y,
        z: O.z + controls.z,
        yaw: O.yaw + controls.yaw,
      };

  const smoothing = 1 - Math.exp(-deltaTime * 8);
  const targetVelocity = gaitEnabled
    ? { x: -controls.leftY, y: -controls.leftX, yaw: -controls.rightX }
    : { x: 0, y: 0, yaw: 0 };

  for (const axis in smoothedVelocity) {
    smoothedVelocity[axis] +=
      (targetVelocity[axis] - smoothedVelocity[axis]) * smoothing;
  }

  const activity = clamp(
    Math.max(
      Math.hypot(smoothedVelocity.x, smoothedVelocity.y),
      Math.abs(smoothedVelocity.yaw),
    ),
    0,
    1,
  );
  phase = (phase + deltaTime * gaitSettings.frequency * activity) % 1;
  grid.position.z = -P.stand;

  const segmentRadii = [6, 5.5, 4.5];
  LEGS.forEach((leg) => {
    const legPhase = (phase + leg.phase) % 1;
    const baseX =
      Math.cos(leg.m) * (P.coxa + P.reach) + (leg.sx * P.L) / 2;
    const baseY =
      Math.sin(leg.m) * (P.coxa + P.reach) + (leg.sy * P.W) / 2;
    const strideX =
      gaitSettings.stepLength *
      (smoothedVelocity.x - (smoothedVelocity.yaw * baseY) / 100);
    const strideY =
      gaitSettings.stepLength *
      (smoothedVelocity.y + (smoothedVelocity.yaw * baseX) / 100);

    let cyclePosition;
    let lift = 0;
    if (legPhase < 0.5) {
      cyclePosition = 0.5 - legPhase * 2;
    } else {
      const swingPhase = (legPhase - 0.5) * 2;
      cyclePosition = -0.5 + (1 - Math.cos(Math.PI * swingPhase)) / 2;
      lift =
        gaitSettings.stepHeight * activity * Math.sin(Math.PI * swingPhase);
    }

    const result = solveLeg(leg, stance, {
      x: cyclePosition * strideX,
      y: cyclePosition * strideY,
      z: lift,
    });
    positionSegment(leg.segments[0], result.hip, result.coxaJoint, segmentRadii[0]);
    positionSegment(
      leg.segments[1],
      result.coxaJoint,
      result.femurJoint,
      segmentRadii[1],
    );
    positionSegment(
      leg.segments[2],
      result.femurJoint,
      result.foot,
      segmentRadii[2],
    );

    leg.joints[0].position.copy(result.hip);
    leg.joints[1].position.copy(result.coxaJoint);
    leg.joints[2].position.copy(result.femurJoint);
    leg.footMesh.position.copy(result.foot);
    leg.footMesh.material = result.reachable
      ? footMaterial
      : unreachableFootMaterial;

    if (frame % 6 === 0) {
      const cells = leg.tableRow.children;
      cells[1].textContent = result.angles[0].toFixed(0);
      cells[2].textContent = result.angles[1].toFixed(0);
      cells[3].textContent = result.angles[2].toFixed(0);
      leg.tableRow.style.color = result.reachable ? "" : "var(--warning)";
    }
  });

  frame += 1;
  camera.position.set(
    cameraDistance * Math.cos(elevation) * Math.cos(azimuth),
    cameraDistance * Math.cos(elevation) * Math.sin(azimuth),
    cameraDistance * Math.sin(elevation) - 40,
  );
  camera.lookAt(0, 0, -40);
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

requestAnimationFrame(animate);
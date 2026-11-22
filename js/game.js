import * as THREE from '../vendor/three.module.js';

const LAPS = 3;
const N = 400;
export const ROAD_HALF = 8;

const TUNING = {
  boostTime: 1.7,
};

// Radius comes from steer angle and wheelbase. The friction circle caps
// cornering, so extra steering past the limit pushes the car wide.
const VEHICLE = {
  wheelbase: 2.55,
  mu: 16.5,
  accel: 17,
  brake: 28,
  drag: 0.0072,
  rollDrag: 0.18,
  maxSteer: 0.46,
  grade: 8.5,
};

const CAM = {
  back: 15,
  height: 36,
  ahead: 11,
  carMix: 0.38,
  lambda: 4.4,
  fovPortrait: 52,
  fovLandscape: 40,
};

const ROSTER = [
  { name: 'JUNO', number: 4, color: 0x6f5bd0, css: '#6f5bd0', spawnT: 0.034, baseLane: -2.2, ai: true, skill: 0.98, aggression: 0.9, top: 42 },
  { name: 'NOVA', number: 3, color: 0xf0b429, css: '#f0b429', spawnT: 0.022, baseLane: 2.35, ai: true, skill: 0.93, aggression: 0.4, top: 41 },
  { name: 'RIO', number: 2, color: 0x148f78, css: '#148f78', spawnT: 0.011, baseLane: -2.05, ai: true, skill: 0.9, aggression: 0.62, top: 40.5 },
  { name: 'YOU', number: 1, color: 0xff4d2e, css: '#ff4d2e', spawnT: 0.0018, baseLane: 2.15, ai: false, skill: 1, aggression: 0, top: 43 },
];

const desiredPos = new THREE.Vector3();
const desiredLook = new THREE.Vector3();
const camPos = new THREE.Vector3();
const lookPos = new THREE.Vector3(0, 20, 30);
const dummy = new THREE.Object3D();
const tmpColor = new THREE.Color();
const SUN_DIR = new THREE.Vector3(-58, 74, 26);

function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }
function damp(a, b, lambda, dt) { return a + (b - a) * (1 - Math.exp(-lambda * dt)); }
function lerp(a, b, t) { return a + (b - a) * t; }
function wrap01(t) { t %= 1; return t < 0 ? t + 1 : t; }
function dampAngle(a, b, lambda, dt) {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return a + d * (1 - Math.exp(-lambda * dt));
}
function blankQuery() {
  return { t: 0, y: 0, bank: 0, signed: 0, rx: 1, rz: 0, tx: 0, tz: 1, dist: 0, seg: 0 };
}

const qA = blankQuery();
const qB = blankQuery();
const qC = blankQuery();

function elevation(t) {
  return 3.55 + Math.sin(t * Math.PI * 2) * 1.22 + Math.sin(t * Math.PI * 4 + 0.7) * 0.34;
}

function trackShape() {
  const pts = [];
  const steps = 28;
  const rx = 134;
  const rz = 198;
  const power = 1.62;
  for (let i = 0; i < steps; i++) {
    const a = Math.PI + (i / steps) * Math.PI * 2;
    const c = Math.cos(a);
    const s = Math.sin(a);
    let x = Math.sign(c) * Math.pow(Math.abs(c), 2 / power) * rx;
    let z = Math.sign(s) * Math.pow(Math.abs(s), 2 / power) * rz;
    if (x > 72) x -= Math.sin((z / rz) * Math.PI * 2) * 30;
    pts.push(new THREE.Vector3(x, 0, z));
  }
  return pts;
}

export function buildTrack() {
  const curve = new THREE.CatmullRomCurve3(trackShape(), true, 'centripetal', 0.5);
  curve.arcLengthDivisions = 1400;
  const samples = [];
  for (let i = 0; i < N; i++) {
    const p = curve.getPointAt(i / N);
    samples.push({ x: p.x, y: elevation(i / N), z: p.z, tx: 0, tz: 1, rx: 1, rz: 0, bank: 0, curv: 0 });
  }
  for (let i = 0; i < N; i++) {
    const next = samples[(i + 1) % N];
    const prev = samples[(i - 1 + N) % N];
    let tx = next.x - prev.x;
    let tz = next.z - prev.z;
    const len = Math.hypot(tx, tz) || 1;
    tx /= len;
    tz /= len;
    samples[i].tx = tx;
    samples[i].tz = tz;
    samples[i].rx = tz;
    samples[i].rz = -tx;
  }
  const banks = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    const a = samples[i];
    const b = samples[(i + 1) % N];
    const cross = a.tx * b.tz - a.tz * b.tx;
    a.curv = cross;
    banks[i] = clamp(cross * 5.2, -0.2, 0.2);
  }
  for (let pass = 0; pass < 4; pass++) {
    const next = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      next[i] = (banks[(i - 1 + N) % N] + banks[i] * 2 + banks[(i + 1) % N]) / 4;
    }
    banks.set(next);
  }
  let length = 0;
  for (let i = 0; i < N; i++) {
    samples[i].bank = banks[i];
    const n = samples[(i + 1) % N];
    length += Math.hypot(n.x - samples[i].x, n.y - samples[i].y, n.z - samples[i].z);
  }
  const span = 7;
  for (let i = 0; i < N; i++) {
    const a = samples[i];
    const b = samples[(i + span) % N];
    const cross = a.tx * b.tz - a.tz * b.tx;
    const dot = a.tx * b.tx + a.tz * b.tz;
    const turn = Math.abs(Math.atan2(cross, dot));
    const ds = length * span / N;
    a.radius = ds / Math.max(turn, 0.025);
  }
  const brakeR = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    let r = samples[i].radius;
    for (let k = 1; k <= 16; k++) r = Math.min(r, samples[(i + k) % N].radius + k * 2.8);
    brakeR[i] = r;
  }
  for (let i = 0; i < N; i++) samples[i].brakeRadius = brakeR[i];
  return { samples, length };
}

let samples = [];
let trackLength = 1;
let minLap = 8;
let bounds = null;
let state = 'title';
let cars = [];
let player = null;
let pads = [];
let startLights = [];
let scene = null;
let camera = null;
let renderer = null;
let sky = null;
let sun = null;
let maxAniso = 4;
let fovBase = 52;
let reduceMotion = false;
let camSnapped = false;
let titleT = 0.72;
let raceTime = 0;
let lapStamp = 0;
let sessionBest = Infinity;
let steerSmooth = 0;
let shake = 0;
let wrongT = 0;
let hintLeft = 0;
let bannerLeft = 0;
let goLeft = 0;
let countLeft = 3;
let countShown = -1;
let dustAcc = 0;
let hitCd = 0;
let scrapeLeft = 0;
let soundOn = true;
let tiltWanted = false;
let tiltOn = false;
let tiltSteer = 0;
let brakeHeld = false;
let stickX = 0;
let stickY = 0;
let stickId = null;
let mapCache = null;
let puffMesh = null;
let puffs = [];
let pCursor = 0;
const PCOUNT = 42;
const keys = new Set();

const audio = { ctx: null, gain: null, oscA: null, oscB: null, filter: null, scrape: null, ready: false };

let app, view, stick, stickKnob, posEl, lapEl, bestEl, timeEl, pauseBtn, bannerEl, wrongEl, hintEl;
let mapEl, brakeBtn, speedEl, boostFlag, countEl, soundBtn, tiltBtn, startBtn, resumeBtn, retryBtn;
let againBtn, finishTitle, finishTime, resultsEl, pauseStat;

function frameAt(t) {
  const n = samples.length;
  let u = wrap01(t) * n;
  let i = Math.floor(u);
  let f = u - i;
  if (i >= n) { i = 0; f = 0; }
  const a = samples[i];
  const b = samples[(i + 1) % n];
  const tx = a.tx + (b.tx - a.tx) * f;
  const tz = a.tz + (b.tz - a.tz) * f;
  const rx = a.rx + (b.rx - a.rx) * f;
  const rz = a.rz + (b.rz - a.rz) * f;
  const tlen = Math.hypot(tx, tz) || 1;
  const rlen = Math.hypot(rx, rz) || 1;
  return {
    x: a.x + (b.x - a.x) * f,
    y: a.y + (b.y - a.y) * f,
    z: a.z + (b.z - a.z) * f,
    tx: tx / tlen,
    tz: tz / tlen,
    rx: rx / rlen,
    rz: rz / rlen,
    bank: a.bank + (b.bank - a.bank) * f,
    curv: a.curv + (b.curv - a.curv) * f,
    brakeRadius: a.brakeRadius + (b.brakeRadius - a.brakeRadius) * f,
    t: wrap01(t),
  };
}

function closest(x, z, out, hint) {
  const n = samples.length;
  let bestD = Infinity;
  let bestI = 0;
  let bestU = 0;
  let bestPx = 0;
  let bestPz = 0;
  const test = (i) => {
    const a = samples[i];
    const b = samples[(i + 1) % n];
    const abx = b.x - a.x;
    const abz = b.z - a.z;
    const ab2 = abx * abx + abz * abz || 1;
    let u = ((x - a.x) * abx + (z - a.z) * abz) / ab2;
    if (u < 0) u = 0;
    else if (u > 1) u = 1;
    const px = a.x + abx * u;
    const pz = a.z + abz * u;
    const d = (x - px) * (x - px) + (z - pz) * (z - pz);
    if (d < bestD) {
      bestD = d;
      bestI = i;
      bestU = u;
      bestPx = px;
      bestPz = pz;
    }
  };
  if (hint == null || hint < 0) {
    for (let i = 0; i < n; i++) test(i);
  } else {
    for (let k = -46; k <= 46; k++) test((hint + k + n) % n);
    if (bestD > 81) {
      bestD = Infinity;
      for (let i = 0; i < n; i++) test(i);
    }
  }
  const a = samples[bestI];
  const b = samples[(bestI + 1) % n];
  const tx = a.tx + (b.tx - a.tx) * bestU;
  const tz = a.tz + (b.tz - a.tz) * bestU;
  const rx = a.rx + (b.rx - a.rx) * bestU;
  const rz = a.rz + (b.rz - a.rz) * bestU;
  const tlen = Math.hypot(tx, tz) || 1;
  const rlen = Math.hypot(rx, rz) || 1;
  out.tx = tx / tlen;
  out.tz = tz / tlen;
  out.rx = rx / rlen;
  out.rz = rz / rlen;
  out.y = a.y + (b.y - a.y) * bestU;
  out.bank = a.bank + (b.bank - a.bank) * bestU;
  out.t = (bestI + bestU) / n;
  if (out.t >= 1) out.t -= 1;
  out.signed = (x - bestPx) * out.rx + (z - bestPz) * out.rz;
  out.dist = Math.sqrt(bestD);
  out.seg = bestI;
  return out;
}

function boot() {
  app = document.getElementById('app');
  view = document.getElementById('view');
  stick = document.getElementById('stick');
  stickKnob = document.getElementById('stick-knob');
  posEl = document.getElementById('pos');
  lapEl = document.getElementById('lap');
  bestEl = document.getElementById('best');
  timeEl = document.getElementById('time');
  pauseBtn = document.getElementById('pause');
  bannerEl = document.getElementById('banner');
  wrongEl = document.getElementById('wrong');
  hintEl = document.getElementById('hint');
  mapEl = document.getElementById('map');
  brakeBtn = document.getElementById('brake');
  speedEl = document.getElementById('speed-num');
  boostFlag = document.getElementById('boost-flag');
  countEl = document.getElementById('count');
  soundBtn = document.getElementById('sound-toggle');
  tiltBtn = document.getElementById('tilt-toggle');
  startBtn = document.getElementById('start');
  resumeBtn = document.getElementById('resume');
  retryBtn = document.getElementById('retry');
  againBtn = document.getElementById('again');
  finishTitle = document.getElementById('finish-title');
  finishTime = document.getElementById('finish-time');
  resultsEl = document.getElementById('results');
  pauseStat = document.getElementById('pause-stat');

  reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  soundOn = loadPref('sunline-sound', true);
  tiltWanted = loadPref('sunline-tilt', false);
  renderToggles();
  bindUI();

  renderer = new THREE.WebGLRenderer({
    canvas: view,
    antialias: true,
    alpha: false,
    powerPreference: 'high-performance',
  });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  maxAniso = Math.min(8, renderer.capabilities.getMaxAnisotropy());

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf0a56e);
  scene.fog = new THREE.Fog(0xf0a56e, 90, 320);
  camera = new THREE.PerspectiveCamera(52, 1, 0.1, 900);

  const hemi = new THREE.HemisphereLight(0xffe0c4, 0x8a4e3c, 1.45);
  scene.add(hemi);
  sun = new THREE.DirectionalLight(0xfff1d4, 3.5);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  sun.shadow.bias = -0.0003;
  sun.shadow.normalBias = 0.045;
  const sc = sun.shadow.camera;
  sc.near = 12;
  sc.far = 190;
  sc.left = -42;
  sc.right = 42;
  sc.top = 42;
  sc.bottom = -42;
  sc.updateProjectionMatrix();
  scene.add(sun);
  scene.add(sun.target);
  const fill = new THREE.DirectionalLight(0xffc2a4, 1.15);
  fill.position.set(48, 22, -36);
  scene.add(fill);

  const skyMat = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    depthWrite: false,
    uniforms: {
      topColor: { value: new THREE.Color(0x243656) },
      horizonColor: { value: new THREE.Color(0xf0a56e) },
      bottomColor: { value: new THREE.Color(0xd97858) },
      sunDir: { value: new THREE.Vector3(-0.62, 0.28, 0.22).normalize() },
    },
    vertexShader: `
      varying vec3 vPos;
      void main() {
        vPos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      varying vec3 vPos;
      uniform vec3 topColor;
      uniform vec3 horizonColor;
      uniform vec3 bottomColor;
      uniform vec3 sunDir;
      void main() {
        vec3 dir = normalize(vPos);
        float h = dir.y;
        vec3 col = mix(horizonColor, topColor, smoothstep(0.0, 0.62, h));
        col = mix(bottomColor, col, smoothstep(-0.35, 0.08, h));
        float glow = pow(clamp(1.0 - abs(h * 1.25), 0.0, 1.0), 6.0);
        col += vec3(1.0, 0.45, 0.18) * glow * 0.32;
        float sunDisc = pow(max(dot(dir, sunDir), 0.0), 180.0);
        col += vec3(1.0, 0.86, 0.55) * sunDisc * 1.5;
        gl_FragColor = vec4(col, 1.0);
      }
    `,
  });
  sky = new THREE.Mesh(new THREE.SphereGeometry(820, 20, 14), skyMat);
  sky.frustumCulled = false;
  sky.renderOrder = -1;
  scene.add(sky);

  const built = buildTrack();
  samples = built.samples;
  trackLength = built.length;
  minLap = Math.max(7, trackLength / 78);
  computeBounds();

  const sand = makeSandTexture();
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(1900, 1900),
    new THREE.MeshStandardMaterial({ map: sand, roughness: 1, metalness: 0 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  buildCircuit();
  addDressing();
  addGantry();
  addStands();
  addLogo();
  cars = ROSTER.map((spec) => buildCar(spec));
  player = cars.find((car) => !car.ai);
  createPads();
  initParticles();
  buildMapCache();

  resize();
  placeCars();
  setState('title');
  updateTitleCamera(1);
  window.addEventListener('resize', resize);
  if (window.visualViewport) window.visualViewport.addEventListener('resize', resize);
  document.addEventListener('visibilitychange', () => {
    if (!audio.ctx) return;
    if (document.hidden) audio.ctx.suspend();
    else if (audio.ready && soundOn) audio.ctx.resume();
  });

  startBtn.disabled = false;
  startBtn.textContent = 'START RACE';
  app.classList.add('is-ready');
  window.__sunline = true;
  lastTime = performance.now();
  requestAnimationFrame(loop);
}

let lastTime = 0;
function loop() {
  requestAnimationFrame(loop);
  const now = performance.now();
  const dt = Math.min((now - lastTime) / 1000, 0.05);
  lastTime = now;
  if (state === 'paused') updateEngine(0, false);
  else step(dt);
  renderer.render(scene, camera);
}

function step(dt) {
  if (state === 'title') {
    titleT = wrap01(titleT + dt * 0.02);
    updateTitleCamera(dt);
    animatePads(dt);
    syncCars(dt);
    updateEngine(0, false);
  } else if (state === 'countdown') {
    stepCountdown(dt);
    followCamera(dt);
    animatePads(dt);
    syncCars(dt);
    updateEngine(6, true);
    updateHud();
    drawMap();
  } else if (state === 'race') {
    stepRace(dt);
    followCamera(dt);
    animatePads(dt);
    syncCars(dt);
    updateParticles(dt);
    updateEngine(player.sp, state === 'race');
    if (state === 'race') {
      updateHud();
      drawMap();
    }
  } else if (state === 'finish') {
    followCamera(dt);
    animatePads(dt);
    syncCars(dt);
    updateParticles(dt);
    updateEngine(0, false);
  }
  if (sky) sky.position.copy(camera.position);
  updateSun();
}

function resize() {
  const vv = window.visualViewport;
  const w = vv ? vv.width : window.innerWidth;
  const h = vv ? vv.height : window.innerHeight;
  app.style.width = w + 'px';
  app.style.height = h + 'px';
  const coarse = window.matchMedia('(pointer: coarse)').matches;
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, coarse ? 1.35 : 1.75));
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(1, h);
  fovBase = w > h ? CAM.fovLandscape : CAM.fovPortrait;
  camera.fov = fovBase;
  camera.updateProjectionMatrix();
}

function updateSun() {
  const x = lookPos.x;
  const y = lookPos.y;
  const z = lookPos.z;
  sun.position.set(x + SUN_DIR.x, y + SUN_DIR.y, z + SUN_DIR.z);
  sun.target.position.set(x, y, z);
  sun.target.updateMatrixWorld();
}

function updateTitleCamera(dt) {
  const f = frameAt(titleT);
  const ahead = frameAt(titleT + 0.03);
  desiredPos.set(f.x - f.tx * 12 + f.rx * 7, f.y + 30, f.z - f.tz * 12 + f.rz * 7);
  desiredLook.set(ahead.x, ahead.y + 1.2, ahead.z);
  const k = camSnapped ? 1 - Math.exp(-1.6 * dt) : 1;
  camSnapped = true;
  camPos.lerp(desiredPos, k);
  lookPos.lerp(desiredLook, k);
  camera.position.copy(camPos);
  camera.up.set(0, 1, 0);
  camera.lookAt(lookPos);
  if (Math.abs(camera.fov - fovBase) > 0.05) {
    camera.fov = damp(camera.fov, fovBase, 3, dt);
    camera.updateProjectionMatrix();
  }
}

function followCamera(dt) {
  const f = frameAt(player.t);
  const aheadDist = CAM.ahead / trackLength;
  const ahead = frameAt(player.t + aheadDist);
  const mix = CAM.carMix;
  desiredLook.set(
    ahead.x * (1 - mix) + player.x * mix,
    ahead.y * 0.55 + player.surfaceY * 0.45 + 0.6,
    ahead.z * (1 - mix) + player.z * mix
  );
  desiredPos.set(
    player.x - f.tx * CAM.back + f.rx * steerSmooth * 1.6,
    player.surfaceY + CAM.height,
    player.z - f.tz * CAM.back + f.rz * steerSmooth * 1.6
  );
  const k = 1 - Math.exp(-CAM.lambda * dt);
  camPos.lerp(desiredPos, k);
  lookPos.lerp(desiredLook, k);
  camera.position.copy(camPos);
  if (shake > 0) {
    if (!reduceMotion) {
      camera.position.x += (Math.random() - 0.5) * shake;
      camera.position.z += (Math.random() - 0.5) * shake;
    }
    shake = Math.max(0, shake - dt * 1.6);
  }
  camera.up.set(0, 1, 0);
  camera.lookAt(lookPos);
  const kick = (player.boost > 0 ? 3.4 : 0) + Math.min(player.sp, 56) * 0.04;
  const nextFov = damp(camera.fov, fovBase + kick, 4, dt);
  if (Math.abs(nextFov - camera.fov) > 0.01) {
    camera.fov = nextFov;
    camera.updateProjectionMatrix();
  }
}

function canvasTexture(canvas, repeat) {
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = repeat ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
  tex.wrapT = repeat ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
  tex.anisotropy = maxAniso;
  return tex;
}

function makeSandTexture() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#c48962';
  g.fillRect(0, 0, 512, 512);
  const img = g.getImageData(0, 0, 512, 512);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (Math.random() - 0.5) * 24;
    img.data[i] = clamp(img.data[i] + n, 0, 255);
    img.data[i + 1] = clamp(img.data[i + 1] + n * 0.62, 0, 255);
    img.data[i + 2] = clamp(img.data[i + 2] + n * 0.3, 0, 255);
  }
  g.putImageData(img, 0, 0);
  for (let i = 0; i < 34; i++) {
    g.fillStyle = 'rgba(92, 48, 34, ' + (0.035 + Math.random() * 0.05) + ')';
    g.beginPath();
    g.arc(Math.random() * 512, Math.random() * 512, 20 + Math.random() * 64, 0, Math.PI * 2);
    g.fill();
  }
  const tex = canvasTexture(c, true);
  tex.repeat.set(18, 18);
  return tex;
}

function makeRoadTexture() {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#34383e';
  g.fillRect(0, 0, 256, 256);
  const img = g.getImageData(0, 0, 256, 256);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (Math.random() - 0.5) * 18;
    img.data[i] = clamp(img.data[i] + n, 0, 255);
    img.data[i + 1] = clamp(img.data[i + 1] + n, 0, 255);
    img.data[i + 2] = clamp(img.data[i + 2] + n, 0, 255);
  }
  g.putImageData(img, 0, 0);
  g.fillStyle = 'rgba(16, 14, 12, 0.22)';
  g.fillRect(116, 0, 24, 256);
  g.fillStyle = '#f3eee4';
  g.fillRect(12, 0, 6, 256);
  g.fillRect(238, 0, 6, 256);
  for (let i = 0; i < 8; i++) g.fillRect(123, i * 32 + 2, 10, 14);
  return canvasTexture(c, true);
}

function makeCurbTexture() {
  const c = document.createElement('canvas');
  c.width = 32;
  c.height = 128;
  const g = c.getContext('2d');
  for (let i = 0; i < 8; i++) {
    g.fillStyle = i % 2 ? '#f4efe6' : '#e4372b';
    g.fillRect(0, i * 16, 32, 16);
  }
  return canvasTexture(c, true);
}

function makeCheckTexture() {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 64;
  const g = c.getContext('2d');
  const s = 32;
  for (let y = 0; y < 2; y++) {
    for (let x = 0; x < 8; x++) {
      g.fillStyle = (x + y) % 2 ? '#f4efe6' : '#1c100e';
      g.fillRect(x * s, y * s, s, s);
    }
  }
  return canvasTexture(c, false);
}

function makeBannerTexture() {
  const c = document.createElement('canvas');
  c.width = 1024;
  c.height = 180;
  const g = c.getContext('2d');
  g.fillStyle = '#1c100e';
  g.fillRect(0, 0, 1024, 180);
  const s = 22;
  for (let x = 0; x < 1024; x += s) {
    g.fillStyle = (x / s) % 2 ? '#f4efe6' : '#1c100e';
    g.fillRect(x, 0, s, 28);
  }
  g.fillStyle = '#ff4d2e';
  g.fillRect(0, 28, 1024, 8);
  g.fillStyle = '#f4efe6';
  g.font = '800 92px Arial Black, Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('SUNLINE', 512, 108);
  return canvasTexture(c, false);
}

function bankUp(s) {
  const sn = Math.sin(s.bank);
  const cs = Math.cos(s.bank);
  return [-s.rx * sn, cs, -s.rz * sn];
}

function buildStrip(slices) {
  const geo = new THREE.BufferGeometry();
  const pos = new Float32Array(slices.length * 6);
  const uv = new Float32Array(slices.length * 4);
  const norm = new Float32Array(slices.length * 6);
  const hasN = slices[0].lnx != null;
  for (let i = 0; i < slices.length; i++) {
    const s = slices[i];
    pos[i * 6] = s.lx;
    pos[i * 6 + 1] = s.ly;
    pos[i * 6 + 2] = s.lz;
    pos[i * 6 + 3] = s.rx;
    pos[i * 6 + 4] = s.ry;
    pos[i * 6 + 5] = s.rz;
    uv[i * 4] = 0;
    uv[i * 4 + 1] = s.v;
    uv[i * 4 + 2] = 1;
    uv[i * 4 + 3] = s.v;
    if (hasN) {
      norm[i * 6] = s.lnx;
      norm[i * 6 + 1] = s.lny;
      norm[i * 6 + 2] = s.lnz;
      norm[i * 6 + 3] = s.rnx;
      norm[i * 6 + 4] = s.rny;
      norm[i * 6 + 5] = s.rnz;
    }
  }
  const idx = [];
  for (let i = 0; i < slices.length - 1; i++) {
    const a = i * 2;
    const b = a + 1;
    const c = a + 2;
    const d = a + 3;
    idx.push(a, c, b, b, c, d);
  }
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  if (hasN) geo.setAttribute('normal', new THREE.BufferAttribute(norm, 3));
  else geo.computeVertexNormals();
  geo.setIndex(idx);
  geo.computeBoundingSphere();
  return geo;
}

function addStrip(slices, material) {
  const mesh = new THREE.Mesh(buildStrip(slices), material);
  mesh.receiveShadow = true;
  scene.add(mesh);
  return mesh;
}

function buildCircuit() {
  const roadV = trackLength / 9;
  const curbV = trackLength / 6;
  const roadMat = new THREE.MeshStandardMaterial({ map: makeRoadTexture(), roughness: 0.94, metalness: 0.02 });
  const curbMat = new THREE.MeshStandardMaterial({ map: makeCurbTexture(), roughness: 0.78 });
  const skirtMat = new THREE.MeshStandardMaterial({ color: 0x9a6248, roughness: 1, side: THREE.DoubleSide });
  const wallMat = new THREE.MeshStandardMaterial({ color: 0x5c463c, roughness: 0.92, side: THREE.DoubleSide });
  const capMat = new THREE.MeshStandardMaterial({
    color: 0xff4d2e,
    emissive: 0xff4d2e,
    emissiveIntensity: 0.28,
    roughness: 0.6,
  });
  const road = [];
  const curbL = [];
  const curbR = [];
  const skirtL = [];
  const skirtR = [];
  const wallL = [];
  const wallR = [];
  const capL = [];
  const capR = [];
  for (let i = 0; i <= N; i++) {
    const s = samples[i % N];
    const up = bankUp(s);
    const vRoad = (i / N) * roadV;
    const vCurb = (i / N) * curbV;
    road.push({
      lx: s.x - s.rx * ROAD_HALF,
      ly: s.y - Math.sin(s.bank) * ROAD_HALF + 0.08,
      lz: s.z - s.rz * ROAD_HALF,
      rx: s.x + s.rx * ROAD_HALF,
      ry: s.y + Math.sin(s.bank) * ROAD_HALF + 0.08,
      rz: s.z + s.rz * ROAD_HALF,
      v: vRoad,
      lnx: up[0], lny: up[1], lnz: up[2],
      rnx: up[0], rny: up[1], rnz: up[2],
    });
    const curb = (side, bucket) => {
      const inner = ROAD_HALF - 0.15;
      const outer = ROAD_HALF + 0.62;
      bucket.push({
        lx: s.x + s.rx * inner * side,
        ly: s.y + Math.sin(s.bank) * inner * side + 0.16,
        lz: s.z + s.rz * inner * side,
        rx: s.x + s.rx * outer * side,
        ry: s.y + Math.sin(s.bank) * outer * side + 0.16,
        rz: s.z + s.rz * outer * side,
        v: vCurb,
        lnx: up[0], lny: up[1], lnz: up[2],
        rnx: up[0], rny: up[1], rnz: up[2],
      });
    };
    curb(-1, curbL);
    curb(1, curbR);
    const skirt = (side, bucket) => {
      const outer = ROAD_HALF + 0.62;
      const x = s.x + s.rx * outer * side;
      const z = s.z + s.rz * outer * side;
      const top = s.y + Math.sin(s.bank) * outer * side + 0.16;
      const nx = s.rx * side;
      const nz = s.rz * side;
      const nl = Math.hypot(nx, 0.2, nz) || 1;
      bucket.push({
        lx: x, ly: top, lz: z,
        rx: x, ry: -0.45, rz: z,
        v: i / N,
        lnx: nx / nl, lny: 0.2 / nl, lnz: nz / nl,
        rnx: nx / nl, rny: 0.2 / nl, rnz: nz / nl,
      });
    };
    skirt(-1, skirtL);
    skirt(1, skirtR);
    const wall = (side, bucket, capBucket) => {
      const lateral = ROAD_HALF + 0.95;
      const x = s.x + s.rx * lateral * side;
      const z = s.z + s.rz * lateral * side;
      const base = s.y + Math.sin(s.bank) * ROAD_HALF * side + 0.18;
      const nx = -s.rx * side;
      const nz = -s.rz * side;
      const nl = Math.hypot(nx, 0.25, nz) || 1;
      bucket.push({
        lx: x, ly: base, lz: z,
        rx: x, ry: base + 0.78, rz: z,
        v: i / N,
        lnx: nx / nl, lny: 0.25 / nl, lnz: nz / nl,
        rnx: nx / nl, rny: 0.25 / nl, rnz: nz / nl,
      });
      const inner = ROAD_HALF + 0.7;
      const outer = ROAD_HALF + 1.25;
      const y = base + 0.82;
      capBucket.push({
        lx: s.x + s.rx * inner * side,
        ly: y,
        lz: s.z + s.rz * inner * side,
        rx: s.x + s.rx * outer * side,
        ry: y,
        rz: s.z + s.rz * outer * side,
        v: i / N,
        lnx: 0, lny: 1, lnz: 0,
        rnx: 0, rny: 1, rnz: 0,
      });
    };
    wall(-1, wallL, capL);
    wall(1, wallR, capR);
  }
  addStrip(road, roadMat);
  addStrip(curbL, curbMat);
  addStrip(curbR, curbMat);
  addStrip(skirtL, skirtMat);
  addStrip(skirtR, skirtMat);
  addStrip(wallL, wallMat);
  addStrip(wallR, wallMat);
  addStrip(capL, capMat);
  addStrip(capR, capMat);

  const s0 = samples[0];
  const s1 = samples[2];
  const up = bankUp(s0);
  const line = addStrip([
    {
      lx: s0.x - s0.rx * (ROAD_HALF - 0.3),
      ly: s0.y - Math.sin(s0.bank) * (ROAD_HALF - 0.3) + 0.2,
      lz: s0.z - s0.rz * (ROAD_HALF - 0.3),
      rx: s0.x + s0.rx * (ROAD_HALF - 0.3),
      ry: s0.y + Math.sin(s0.bank) * (ROAD_HALF - 0.3) + 0.2,
      rz: s0.z + s0.rz * (ROAD_HALF - 0.3),
      v: 0,
      lnx: up[0], lny: up[1], lnz: up[2],
      rnx: up[0], rny: up[1], rnz: up[2],
    },
    {
      lx: s1.x - s1.rx * (ROAD_HALF - 0.3),
      ly: s1.y - Math.sin(s1.bank) * (ROAD_HALF - 0.3) + 0.2,
      lz: s1.z - s1.rz * (ROAD_HALF - 0.3),
      rx: s1.x + s1.rx * (ROAD_HALF - 0.3),
      ry: s1.y + Math.sin(s1.bank) * (ROAD_HALF - 0.3) + 0.2,
      rz: s1.z + s1.rz * (ROAD_HALF - 0.3),
      v: 1,
      lnx: up[0], lny: up[1], lnz: up[2],
      rnx: up[0], rny: up[1], rnz: up[2],
    },
  ], new THREE.MeshBasicMaterial({ map: makeCheckTexture(), polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4 }));
  line.renderOrder = 2;
}

function clearOfTrack(x, z, min) {
  for (let i = 0; i < N; i += 2) {
    if (Math.hypot(samples[i].x - x, samples[i].z - z) < min) return false;
  }
  return true;
}

function addDressing() {
  const plantGeo = new THREE.ConeGeometry(0.85, 2.7, 6);
  plantGeo.translate(0, 1.35, 0);
  const plantMat = new THREE.MeshStandardMaterial({ color: 0x3e6a45, roughness: 1 });
  const plantSpots = [];
  for (let i = 0; i < N; i += 5) {
    const s = samples[i];
    const side = ((i / 5) % 2 === 0) ? 1 : -1;
    const dist = 16 + ((i * 17) % 14);
    const x = s.x + s.rx * side * dist;
    const z = s.z + s.rz * side * dist;
    if (!clearOfTrack(x, z, 13)) continue;
    plantSpots.push({ x, z, s: 0.75 + ((i * 13) % 8) / 10, yaw: i * 0.37 });
    if (plantSpots.length >= 70) break;
  }
  const plants = new THREE.InstancedMesh(plantGeo, plantMat, plantSpots.length);
  plants.frustumCulled = false;
  plantSpots.forEach((p, i) => {
    dummy.position.set(p.x, 0, p.z);
    dummy.scale.set(p.s, p.s * (0.85 + (i % 4) * 0.08), p.s);
    dummy.rotation.set(0, p.yaw, 0);
    dummy.updateMatrix();
    plants.setMatrixAt(i, dummy.matrix);
    tmpColor.set(i % 3 === 0 ? 0x4e7a3e : i % 3 === 1 ? 0x2f5a40 : 0x3e6a45);
    plants.setColorAt(i, tmpColor);
  });
  if (plants.instanceColor) plants.instanceColor.needsUpdate = true;
  scene.add(plants);

  const rockGeo = new THREE.DodecahedronGeometry(1, 0);
  const rockMat = new THREE.MeshStandardMaterial({ color: 0x8d6452, roughness: 1, flatShading: true });
  const rocks = [];
  for (let i = 4; i < N; i += 9) {
    const s = samples[i];
    const side = i % 2 === 0 ? 1 : -1;
    const dist = 20 + ((i * 11) % 18);
    const x = s.x + s.rx * side * dist;
    const z = s.z + s.rz * side * dist;
    if (!clearOfTrack(x, z, 14)) continue;
    rocks.push({ x, z, s: 0.55 + ((i * 5) % 10) / 8, ry: i });
    if (rocks.length >= 36) break;
  }
  const rockMesh = new THREE.InstancedMesh(rockGeo, rockMat, Math.max(1, rocks.length));
  rockMesh.frustumCulled = false;
  rocks.forEach((p, i) => {
    dummy.position.set(p.x, p.s * 0.2, p.z);
    dummy.rotation.set(p.ry * 0.2, p.ry, p.ry * 0.4);
    dummy.scale.set(p.s, p.s * 0.72, p.s * 0.9);
    dummy.updateMatrix();
    rockMesh.setMatrixAt(i, dummy.matrix);
    tmpColor.set(i % 2 ? 0xa07860 : 0x6e5144);
    rockMesh.setColorAt(i, tmpColor);
  });
  if (!rocks.length) {
    dummy.scale.setScalar(0);
    dummy.updateMatrix();
    rockMesh.setMatrixAt(0, dummy.matrix);
  }
  if (rockMesh.instanceColor) rockMesh.instanceColor.needsUpdate = true;
  scene.add(rockMesh);

  const mesaMatA = new THREE.MeshStandardMaterial({ color: 0xb4785c, roughness: 1, flatShading: true });
  const mesaMatB = new THREE.MeshStandardMaterial({ color: 0x8d5a48, roughness: 1, flatShading: true });
  const cx = (bounds.minX + bounds.maxX) / 2;
  const cz = (bounds.minZ + bounds.maxZ) / 2;
  const radX = (bounds.maxX - bounds.minX) * 0.42 + 90;
  const radZ = (bounds.maxZ - bounds.minZ) * 0.42 + 90;
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * Math.PI * 2 + 0.35;
    const h = 16 + (i % 3) * 9;
    const mesa = new THREE.Mesh(new THREE.CylinderGeometry(7 + (i % 3) * 3, 13 + (i % 2) * 4, h, 6), i % 2 ? mesaMatA : mesaMatB);
    mesa.position.set(cx + Math.cos(a) * radX, h * 0.32, cz + Math.sin(a) * radZ);
    scene.add(mesa);
  }

  const lampGeo = new THREE.SphereGeometry(0.32, 10, 8);
  const poleGeo = new THREE.CylinderGeometry(0.05, 0.08, 1.5, 5);
  const lampMat = new THREE.MeshStandardMaterial({ color: 0xfff1c9, emissive: 0xffb15a, emissiveIntensity: 1.35, roughness: 0.4 });
  const poleMat = new THREE.MeshStandardMaterial({ color: 0x3a302c, roughness: 0.8 });
  const lamps = [];
  for (let i = 0; i < N; i += 18) {
    const s = samples[i];
    const x = s.x + s.rx * (ROAD_HALF + 1.8);
    const z = s.z + s.rz * (ROAD_HALF + 1.8);
    const y = s.y + Math.sin(s.bank) * ROAD_HALF + 1.7;
    lamps.push({ x, y, z });
  }
  const lampMesh = new THREE.InstancedMesh(lampGeo, lampMat, lamps.length);
  const poleMesh = new THREE.InstancedMesh(poleGeo, poleMat, lamps.length);
  lampMesh.frustumCulled = false;
  poleMesh.frustumCulled = false;
  lamps.forEach((p, i) => {
    dummy.position.set(p.x, p.y, p.z);
    dummy.rotation.set(0, 0, 0);
    dummy.scale.setScalar(1);
    dummy.updateMatrix();
    lampMesh.setMatrixAt(i, dummy.matrix);
    dummy.position.set(p.x, p.y - 0.95, p.z);
    dummy.updateMatrix();
    poleMesh.setMatrixAt(i, dummy.matrix);
  });
  scene.add(lampMesh);
  scene.add(poleMesh);
}

function addGantry() {
  const s = samples[0];
  const yaw = Math.atan2(s.tx, s.tz);
  const g = new THREE.Group();
  g.position.set(s.x, s.y, s.z);
  g.rotation.y = yaw;
  const span = (ROAD_HALF + 1.45) * 2;
  const pillarMat = new THREE.MeshStandardMaterial({ color: 0x4e433e, roughness: 0.84 });
  for (const x of [-(ROAD_HALF + 1.45), ROAD_HALF + 1.45]) {
    const p = new THREE.Mesh(new THREE.BoxGeometry(0.55, 5.3, 0.55), pillarMat);
    p.position.set(x, 2.65, 0);
    p.castShadow = true;
    g.add(p);
  }
  const beam = new THREE.Mesh(new THREE.BoxGeometry(span, 0.42, 0.58), pillarMat);
  beam.position.set(0, 5.45, 0);
  beam.castShadow = true;
  g.add(beam);
  const banner = new THREE.Mesh(
    new THREE.PlaneGeometry(span - 1.4, 1.85),
    new THREE.MeshBasicMaterial({ map: makeBannerTexture(), side: THREE.DoubleSide })
  );
  banner.position.set(0, 4.35, -0.2);
  banner.rotation.y = Math.PI;
  g.add(banner);
  startLights = [-1.2, 0, 1.2].map((x) => {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.26, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0x3a2424, emissive: 0x000000, emissiveIntensity: 0, roughness: 0.4 })
    );
    mesh.position.set(x, 5.12, -0.46);
    g.add(mesh);
    return mesh;
  });
  scene.add(g);
}

function addStands() {
  const s = frameAt(0.012);
  const steel = new THREE.MeshStandardMaterial({ color: 0x6d8494, roughness: 0.74, metalness: 0.08 });
  const seat = new THREE.MeshStandardMaterial({ color: 0xd85a42, roughness: 0.82 });
  for (const side of [-1, 1]) {
    const g = new THREE.Group();
    const dist = ROAD_HALF + 9;
    g.position.set(s.x + s.rx * side * dist, 0, s.z + s.rz * side * dist);
    g.rotation.y = Math.atan2(s.tx, s.tz);
    for (let i = 0; i < 3; i++) {
      const h = 1.05 + i * 0.72;
      const box = new THREE.Mesh(new THREE.BoxGeometry(10, h, 2.5), steel);
      box.position.set(side * (1.4 + i * 2.3), h / 2, 0);
      box.receiveShadow = true;
      g.add(box);
      const top = new THREE.Mesh(new THREE.BoxGeometry(10, 0.16, 2.5), seat);
      top.position.set(side * (1.4 + i * 2.3), h + 0.08, 0);
      g.add(top);
    }
    scene.add(g);
  }
}

function insideLoop(x, z) {
  let inside = false;
  for (let i = 0; i < N; i++) {
    const a = samples[i];
    const b = samples[(i + 1) % N];
    if ((a.z > z) !== (b.z > z)) {
      const xInt = a.x + ((b.x - a.x) * (z - a.z)) / ((b.z - a.z) || 1e-9);
      if (x < xInt) inside = !inside;
    }
  }
  return inside;
}

function findLogoPoint() {
  let cx = 0;
  let cz = 0;
  for (const s of samples) { cx += s.x; cz += s.z; }
  cx /= N;
  cz /= N;
  const candidates = [{ x: cx, z: cz }];
  for (let k = 0; k < 8; k++) {
    const a = (k / 8) * Math.PI * 2;
    candidates.push({ x: cx + Math.cos(a) * 28, z: cz + Math.sin(a) * 28 });
  }
  for (const c of candidates) {
    if (!insideLoop(c.x, c.z)) continue;
    let min = Infinity;
    for (let i = 0; i < N; i += 2) min = Math.min(min, Math.hypot(samples[i].x - c.x, samples[i].z - c.z));
    if (min > 26) return c;
  }
  return null;
}

function addLogo() {
  const spot = findLogoPoint();
  if (!spot) return;
  const c = document.createElement('canvas');
  c.width = 1024;
  c.height = 256;
  const g = c.getContext('2d');
  g.clearRect(0, 0, 1024, 256);
  g.fillStyle = 'rgba(92, 42, 28, 0.55)';
  g.font = '800 168px Arial Black, Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('SUNLINE', 512, 132);
  const tex = canvasTexture(c, false);
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(86, 21.5),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false })
  );
  const f = samples[0];
  mesh.position.set(spot.x, 0.18, spot.z);
  mesh.rotation.order = 'YXZ';
  mesh.rotation.set(-Math.PI / 2, Math.atan2(f.tx, f.tz), 0);
  mesh.renderOrder = 1;
  scene.add(mesh);
}

function numberTexture(n) {
  const c = document.createElement('canvas');
  c.width = 128;
  c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = '#f4efe6';
  g.beginPath();
  g.arc(64, 64, 58, 0, Math.PI * 2);
  g.fill();
  g.fillStyle = '#1c100e';
  g.font = '800 78px Arial Black, Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(String(n), 64, 70);
  return canvasTexture(c, false);
}

function buildCar(spec) {
  const root = new THREE.Group();
  const body = new THREE.Group();
  root.add(body);
  const paint = new THREE.MeshStandardMaterial({ color: spec.color, roughness: 0.46, metalness: 0.1 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x1a1e24, roughness: 0.4, metalness: 0.35 });
  const glass = new THREE.MeshStandardMaterial({ color: 0x141a22, roughness: 0.16, metalness: 0.5 });
  const rubber = new THREE.MeshStandardMaterial({ color: 0x171717, roughness: 0.92 });
  const hubMat = new THREE.MeshStandardMaterial({ color: 0xd7d1c5, roughness: 0.42, metalness: 0.28 });
  const cream = new THREE.MeshStandardMaterial({ color: 0xf4efe6, roughness: 0.5 });
  const lamp = new THREE.MeshStandardMaterial({ color: 0xfff1d2, emissive: 0xffe2a8, emissiveIntensity: 0.85, roughness: 0.4 });
  const tailMat = new THREE.MeshStandardMaterial({ color: 0x3a1214, emissive: 0xff2430, emissiveIntensity: 0.35, roughness: 0.4 });
  const add = (w, h, d, x, y, z, mat, shadow) => {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
    mesh.position.set(x, y, z);
    mesh.castShadow = !!shadow;
    mesh.receiveShadow = true;
    body.add(mesh);
  };
  add(1.95, 0.4, 3.85, 0, 0.52, 0.05, paint, true);
  add(1.55, 0.36, 1.5, 0, 0.9, -0.22, paint, true);
  add(1.4, 0.12, 0.48, 0, 1.08, 0.42, glass, false);
  add(1.72, 0.12, 1.15, 0, 0.76, 1.08, paint, false);
  add(0.2, 0.04, 0.95, 0, 0.84, 1.08, cream, false);
  add(1.15, 0.14, 0.1, 0, 0.5, 1.96, dark, false);
  add(0.32, 0.1, 0.08, -0.55, 0.58, 1.9, lamp, false);
  add(0.32, 0.1, 0.08, 0.55, 0.58, 1.9, lamp, false);
  add(0.34, 0.1, 0.08, -0.62, 0.56, -1.9, tailMat, false);
  add(0.34, 0.1, 0.08, 0.62, 0.56, -1.9, tailMat, false);
  add(1.78, 0.08, 0.38, 0, 1.18, -1.72, paint, false);
  add(0.08, 0.28, 0.1, -0.55, 1.0, -1.72, dark, false);
  add(0.08, 0.28, 0.1, 0.55, 1.0, -1.72, dark, false);

  const num = new THREE.Mesh(
    new THREE.PlaneGeometry(1.12, 1.12),
    new THREE.MeshBasicMaterial({ map: numberTexture(spec.number), transparent: true, depthWrite: false })
  );
  num.rotation.order = 'XYZ';
  num.rotation.set(-Math.PI / 2, 0, Math.PI);
  num.position.set(0, 1.12, -0.38);
  num.renderOrder = 2;
  body.add(num);

  const wheels = [];
  const wheelGeo = new THREE.CylinderGeometry(0.36, 0.36, 0.28, 12);
  const hubGeo = new THREE.CylinderGeometry(0.16, 0.16, 0.3, 8);
  const makeWheel = (x, z, front) => {
    const pivot = new THREE.Group();
    pivot.position.set(x, 0.36, z);
    const spin = new THREE.Group();
    const tire = new THREE.Mesh(wheelGeo, rubber);
    tire.rotation.z = Math.PI / 2;
    const hub = new THREE.Mesh(hubGeo, hubMat);
    hub.rotation.z = Math.PI / 2;
    spin.add(tire);
    spin.add(hub);
    pivot.add(spin);
    body.add(pivot);
    wheels.push({ pivot, spin, front });
  };
  makeWheel(-1.08, 1.18, true);
  makeWheel(1.08, 1.18, true);
  makeWheel(-1.08, -1.22, false);
  makeWheel(1.08, -1.22, false);

  const glow = new THREE.Mesh(
    new THREE.PlaneGeometry(2.35, 3.8),
    new THREE.MeshBasicMaterial({ color: 0x3ef0c8, transparent: true, opacity: 0.55, depthWrite: false })
  );
  glow.rotation.x = -Math.PI / 2;
  glow.position.y = 0.1;
  glow.visible = false;
  glow.renderOrder = 2;
  root.add(glow);

  const blob = new THREE.Mesh(
    new THREE.CircleGeometry(1.45, 18),
    new THREE.MeshBasicMaterial({ color: 0x140c0a, transparent: true, opacity: 0.22, depthWrite: false, side: THREE.DoubleSide })
  );
  blob.renderOrder = 1;

  scene.add(root);
  scene.add(blob);
  return {
    ...spec,
    root, body, wheels, glow, blob, tailMat,
    spin: 0, yaw: 0, yawRate: 0, x: 0, z: 0, vx: 0, vz: 0,
    t: spec.spawnT, prevT: spec.spawnT, lap: 0, lapStamp: 0, lane: spec.baseLane,
    aimLane: spec.baseLane, cmdSteer: 0,
    speed: 0, fSpeed: 0, lat: 0, sp: 0, boost: 0,
    finished: false, finishTime: null, surfaceY: 0,
    braking: false, visualSteer: 0, curv: 0, seg: 0,
  };
}

function createPads() {
  const picks = [];
  for (let i = 10; i < N - 10; i++) {
    const t = i / N;
    if (t < 0.1 || t > 0.9) continue;
    if (Math.abs(samples[i].curv) > 0.009) continue;
    if (picks.length && t - picks[picks.length - 1] < 0.18) continue;
    let calm = true;
    for (let k = -2; k <= 2; k++) {
      if (Math.abs(samples[(i + k + N) % N].curv) > 0.016) calm = false;
    }
    if (!calm) continue;
    picks.push(t);
    if (picks.length === 3) break;
  }
  while (picks.length < 3) picks.push([0.18, 0.45, 0.7][picks.length]);
  const geo = new THREE.BoxGeometry(ROAD_HALF * 1.65, 0.12, 8);
  pads = picks.slice(0, 3).map((t, i) => {
    const f = frameAt(t);
    const mat = new THREE.MeshStandardMaterial({
      color: 0x08352f,
      emissive: 0x3ef0c8,
      emissiveIntensity: 0.9,
      roughness: 0.32,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(f.x, f.y + 0.3, f.z);
    mesh.rotation.y = Math.atan2(f.tx, f.tz);
    scene.add(mesh);
    return { t, half: 4.2 / trackLength, mesh, mat, flash: 0, armed: false, phase: i * 1.4 };
  });
}

function initParticles() {
  const c = document.createElement('canvas');
  c.width = 64;
  c.height = 64;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 2, 32, 32, 32);
  grd.addColorStop(0, 'rgba(255,255,255,0.9)');
  grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd;
  g.fillRect(0, 0, 64, 64);
  const mat = new THREE.MeshBasicMaterial({
    map: canvasTexture(c, false),
    transparent: true,
    depthWrite: false,
    color: 0xffffff,
  });
  puffMesh = new THREE.InstancedMesh(new THREE.PlaneGeometry(1, 1), mat, PCOUNT);
  puffMesh.frustumCulled = false;
  puffMesh.renderOrder = 3;
  scene.add(puffMesh);
  puffs = [];
  for (let i = 0; i < PCOUNT; i++) {
    puffs.push({ on: false, life: 0, max: 1, size: 1, x: 0, y: -40, z: 0, vx: 0, vy: 0, vz: 0, hex: 0xffffff });
    puffMesh.setColorAt(i, new THREE.Color(1, 1, 1));
  }
}

function spawnPuff(x, y, z, hex, size, life, vx, vy, vz) {
  const p = puffs[pCursor];
  pCursor = (pCursor + 1) % PCOUNT;
  p.on = true;
  p.life = life;
  p.max = life;
  p.size = size;
  p.x = x;
  p.y = y;
  p.z = z;
  p.vx = vx;
  p.vy = vy;
  p.vz = vz;
  p.hex = hex;
}

function updateParticles(dt) {
  for (let i = 0; i < PCOUNT; i++) {
    const p = puffs[i];
    if (p.on) {
      p.life -= dt;
      if (p.life <= 0) p.on = false;
      else {
        p.x += p.vx * dt;
        p.y += p.vy * dt;
        p.z += p.vz * dt;
        p.vy += 0.8 * dt;
      }
    }
    if (!p.on) {
      dummy.position.set(0, -40, 0);
      dummy.scale.setScalar(0);
      dummy.rotation.set(0, 0, 0);
    } else {
      const k = p.life / p.max;
      dummy.position.set(p.x, p.y, p.z);
      dummy.scale.setScalar(p.size * (0.4 + k));
      dummy.quaternion.copy(camera.quaternion);
    }
    dummy.updateMatrix();
    puffMesh.setMatrixAt(i, dummy.matrix);
    const fade = p.on ? p.life / p.max : 0;
    tmpColor.setHex(p.hex || 0xffffff);
    tmpColor.multiplyScalar(0.3 + 0.7 * fade);
    puffMesh.setColorAt(i, tmpColor);
  }
  puffMesh.instanceMatrix.needsUpdate = true;
  if (puffMesh.instanceColor) puffMesh.instanceColor.needsUpdate = true;
}

function computeBounds() {
  let minX = Infinity;
  let maxX = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (const s of samples) {
    minX = Math.min(minX, s.x);
    maxX = Math.max(maxX, s.x);
    minZ = Math.min(minZ, s.z);
    maxZ = Math.max(maxZ, s.z);
  }
  const px = (maxX - minX) * 0.16 || 1;
  const pz = (maxZ - minZ) * 0.16 || 1;
  bounds = { minX: minX - px, maxX: maxX + px, minZ: minZ - pz, maxZ: maxZ + pz };
}

function mapPoint(x, z) {
  const w = 180;
  const pad = 16;
  const u = (x - bounds.minX) / (bounds.maxX - bounds.minX);
  const v = (bounds.maxZ - z) / (bounds.maxZ - bounds.minZ);
  return [pad + u * (w - pad * 2), pad + v * (w - pad * 2)];
}

function buildMapCache() {
  mapCache = document.createElement('canvas');
  mapCache.width = 180;
  mapCache.height = 180;
  const ctx = mapCache.getContext('2d');
  ctx.beginPath();
  samples.forEach((s, i) => {
    const p = mapPoint(s.x, s.z);
    if (i === 0) ctx.moveTo(p[0], p[1]);
    else ctx.lineTo(p[0], p[1]);
  });
  ctx.closePath();
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.strokeStyle = '#1c100e';
  ctx.lineWidth = 12;
  ctx.stroke();
  ctx.strokeStyle = '#f4efe6';
  ctx.lineWidth = 6;
  ctx.stroke();
  const s0 = samples[0];
  const a = mapPoint(s0.x - s0.rx * 7, s0.z - s0.rz * 7);
  const b = mapPoint(s0.x + s0.rx * 7, s0.z + s0.rz * 7);
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.strokeStyle = '#ff4d2e';
  ctx.lineWidth = 3;
  ctx.stroke();
}

function drawMap() {
  const ctx = mapEl.getContext('2d');
  const w = mapEl.width;
  ctx.clearRect(0, 0, w, w);
  ctx.save();
  ctx.beginPath();
  ctx.arc(w / 2, w / 2, w / 2 - 2, 0, Math.PI * 2);
  ctx.clip();
  ctx.drawImage(mapCache, 0, 0);
  for (const car of cars) {
    if (!car.ai) continue;
    const p = mapPoint(car.x, car.z);
    ctx.beginPath();
    ctx.arc(p[0], p[1], 4.5, 0, Math.PI * 2);
    ctx.fillStyle = car.css;
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = '#1c100e';
    ctx.stroke();
  }
  const p = mapPoint(player.x, player.z);
  ctx.translate(p[0], p[1]);
  ctx.rotate(player.yaw);
  ctx.fillStyle = '#ff4d2e';
  ctx.strokeStyle = '#1c100e';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, -8);
  ctx.lineTo(5.5, 6);
  ctx.lineTo(-5.5, 6);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function placeCars() {
  raceTime = 0;
  lapStamp = 0;
  wrongT = 0;
  hintLeft = 0;
  bannerLeft = 0;
  goLeft = 0;
  dustAcc = 0;
  shake = 0;
  steerSmooth = 0;
  hintEl.classList.remove('show');
  bannerEl.classList.remove('show');
  wrongEl.classList.remove('show');
  boostFlag.classList.remove('show');
  for (const car of cars) {
    const f = frameAt(car.spawnT);
    car.t = car.spawnT;
    car.prevT = car.spawnT;
    car.lap = 0;
    car.finished = false;
    car.finishTime = null;
    car.boost = 0;
    car.lane = car.baseLane;
    car.speed = 0;
    car.vx = 0;
    car.vz = 0;
    car.yawRate = 0;
    car.fSpeed = 0;
    car.lat = 0;
    car.sp = 0;
    car.aimLane = car.baseLane;
    car.cmdSteer = 0;
    car.lapStamp = 0;
    car.yaw = Math.atan2(f.tx, f.tz);
    car.x = f.x + f.rx * car.baseLane;
    car.z = f.z + f.rz * car.baseLane;
    car.surfaceY = f.y + Math.sin(f.bank) * car.baseLane + 0.08;
    car.visualSteer = 0;
    car.braking = false;
    car.seg = Math.floor(car.spawnT * N) % N;
    car.body.rotation.x = 0;
    car.body.rotation.z = 0;
  }
  for (const pad of pads) pad.armed = false;
  syncCars(0);
}

function beginCountdown() {
  placeCars();
  countLeft = 3;
  countShown = -1;
  hideCount();
  updateLights(0);
  setState('countdown');
}

function stepCountdown(dt) {
  countLeft -= dt;
  const step = countLeft > 0 ? Math.ceil(countLeft) : 0;
  if (step !== countShown) {
    countShown = step;
    if (step === 0) {
      updateLights(4);
      launch();
    } else {
      updateLights(4 - step);
      showCount(String(step));
      beep(500, 0.1, 'square', 0.048);
    }
  }
}

function launch() {
  raceTime = 0;
  lapStamp = 0;
  for (const car of cars) {
    const kick = car.ai ? 11 : 12;
    car.vx = Math.sin(car.yaw) * kick;
    car.vz = Math.cos(car.yaw) * kick;
    car.yawRate = 0;
    car.fSpeed = kick;
    car.sp = kick;
  }
  setState('race');
  showCount('GO');
  goLeft = 0.55;
  hintLeft = 3.2;
  hintEl.textContent = 'DRAG THE STICK TO STEER';
  hintEl.classList.add('show');
  beep(880, 0.22, 'square', 0.06);
  buzz(18);
}

function stepRace(dt) {
  if (hitCd > 0) hitCd -= dt;
  if (scrapeLeft > 0) scrapeLeft -= dt;
  raceTime += dt;
  steerSmooth = damp(steerSmooth, readSteer(), 14, dt);
  const brakeAmt = readBrakeAmount();
  stepVehicle(player, {
    steer: steerSmooth,
    throttle: brakeAmt > 0.08 ? 0 : 1,
    brake: brakeAmt,
    grade: gradeAt(player),
  }, dt);
  for (const car of cars) {
    if (!car.ai) continue;
    stepVehicle(car, driveAI(car, dt), dt);
  }
  collideAll();
  for (const car of cars) resolveWalls(car, dt);
  for (const car of cars) {
    closest(car.x, car.z, qC, car.seg);
    if (qC.dist > 20) continue;
    car.seg = qC.seg;
    car.surfaceY = qC.y + Math.sin(qC.bank) * qC.signed + 0.08;
    const along = car.vx * qC.tx + car.vz * qC.tz;
    const nextT = qC.t;
    noteCarProgress(car, nextT);
    if (car === player) {
      if (state !== 'race') return;
      if (along < -4) wrongT += dt;
      else wrongT = Math.max(0, wrongT - dt * 2);
    }
  }
  if (state !== 'race') return;
  checkPads();
  if (hintLeft > 0) {
    hintLeft -= dt;
    if (hintLeft <= 0) hintEl.classList.remove('show');
  }
  if (bannerLeft > 0) {
    bannerLeft -= dt;
    if (bannerLeft <= 0) bannerEl.classList.remove('show');
  }
  if (goLeft > 0) {
    goLeft -= dt;
    if (goLeft <= 0) hideCount();
  }
  wrongEl.classList.toggle('show', wrongT > 0.32);
  dustAcc += dt;
  if (player.sliding && dustAcc > 0.04) {
    dustAcc = 0;
    spawnPuff(
      player.x - Math.sin(player.yaw) * 1.55,
      player.surfaceY + 0.4,
      player.z - Math.cos(player.yaw) * 1.55,
      0xd7b09a,
      0.85,
      0.42,
      -player.vx * 0.04,
      1.5,
      -player.vz * 0.04
    );
  }
  if (player.boost > 0 && Math.random() < 0.35) {
    spawnPuff(player.x, player.surfaceY + 0.3, player.z, 0x8fffe4, 0.55, 0.28, (Math.random() - 0.5) * 2, 0.8, (Math.random() - 0.5) * 2);
  }
  player.braking = readBrakeAmount() > 0.2;
}

function gradeAt(car) {
  const here = frameAt(car.t);
  const ahead = frameAt(car.t + 0.012);
  return (ahead.y - here.y) / Math.max(1, trackLength * 0.012);
}

export function stepVehicle(car, ctrl, dt) {
  const steps = dt > 0.02 ? 2 : 1;
  const h = dt / steps;
  for (let i = 0; i < steps; i++) integrateVehicle(car, ctrl, h);
  if (car.boost > 0) car.boost = Math.max(0, car.boost - dt);
}

function integrateVehicle(car, ctrl, dt) {
  const steer = clamp(ctrl.steer || 0, -1, 1);
  const throttle = clamp(ctrl.throttle || 0, 0, 1);
  const brake = clamp(ctrl.brake || 0, 0, 1);
  // Positive steer is a left turn. In this frame, increasing yaw rotates toward +right.
  const delta = -steer * VEHICLE.maxSteer;
  const fwdX = Math.sin(car.yaw);
  const fwdZ = Math.cos(car.yaw);
  const rightX = Math.cos(car.yaw);
  const rightZ = -Math.sin(car.yaw);
  let longS = car.vx * fwdX + car.vz * fwdZ;
  let latS = car.vx * rightX + car.vz * rightZ;

  let drive = throttle * VEHICLE.accel * (car.boost > 0 ? 1.38 : 1);
  drive -= brake * VEHICLE.brake;
  drive -= Math.sign(longS) * VEHICLE.drag * longS * longS;
  drive -= longS * VEHICLE.rollDrag;
  drive -= (ctrl.grade || 0) * VEHICLE.grade;

  // One grip budget: acceleration and cornering cannot both be maxed.
  const longUse = clamp(drive / VEHICLE.mu, -1, 1);
  const latMax = VEHICLE.mu * Math.sqrt(Math.max(0, 1 - longUse * longUse));
  const yawDemand = (longS / VEHICLE.wheelbase) * Math.tan(delta);
  const latDemand = Math.abs(yawDemand * longS);
  const asking = Math.abs(delta) > 0.02 && Math.abs(longS) > 2;
  const grip = asking ? latMax / Math.max(latDemand, 0.25) : 1;
  const yawTarget = grip >= 1 ? yawDemand : yawDemand * grip;
  car.yawRate = damp(car.yawRate || 0, yawTarget, 7, dt);
  car.yaw += car.yawRate * dt;
  longS += drive * dt;
  // Tires pull the slide back. Past the limit they cannot, so the car pushes wide.
  const align = grip >= 1 ? 9 : 1.6 + grip * 3;
  latS = damp(latS, 0, align, dt);
  if (grip < 1) longS *= Math.exp(-1.4 * (1 - grip) * dt);
  if (longS < -1.2) longS = damp(longS, 0, 7, dt);

  const nfx = Math.sin(car.yaw);
  const nfz = Math.cos(car.yaw);
  const nrx = Math.cos(car.yaw);
  const nrz = -Math.sin(car.yaw);
  car.vx = nfx * longS + nrx * latS;
  car.vz = nfz * longS + nrz * latS;
  car.x += car.vx * dt;
  car.z += car.vz * dt;
  if (!Number.isFinite(car.x) || !Number.isFinite(car.yawRate)) {
    car.vx = 0;
    car.vz = 0;
    car.yawRate = 0;
  }
  car.fSpeed = longS;
  car.lat = latS;
  car.sp = Math.hypot(car.vx, car.vz);
  car.visualSteer = steer;
  car.braking = brake > 0.2;
  car.sliding = grip < 0.9 && Math.abs(longS) > 8;
}

function trackGap(a, b) {
  let gap = (a.lap + a.t) - (b.lap + b.t);
  if (gap > 0.5) gap -= 1;
  if (gap < -0.5) gap += 1;
  return gap;
}

function driveAI(car, dt) {
  const here = frameAt(car.t);
  const dist = clamp(9 + car.sp * 0.62, 11, 38);
  const look = frameAt(car.t + dist / trackLength);
  const tight = Math.min(here.brakeRadius, look.brakeRadius);
  const vCap = Math.sqrt(Math.max(12, VEHICLE.mu * 0.9 * car.skill * tight));
  const vTarget = Math.min(car.top, vCap);
  let throttle = car.finished ? 0.4 : 1;
  let brake = 0;
  if (!car.finished && car.sp > vTarget + 0.6) {
    brake = clamp((car.sp - vTarget) / 9, 0, 1);
    throttle = 0;
  } else if (car.sp > vTarget - 0.4) {
    throttle *= 0.45;
  }

  const inside = -Math.sign(look.curv || 0) * clamp(Math.abs(look.curv) * 16, 0, 3.2);
  let lane = car.finished ? Math.sign(car.baseLane || 1) * 4.6 : inside;
  if (!car.finished && player) {
    const gap = trackGap(player, car);
    if (Math.abs(gap) < 0.018) {
      const side = (player.x - car.x) * here.rx + (player.z - car.z) * here.rz;
      lane += clamp(side, -1, 1) * 2.4 * car.aggression;
    }
  }
  lane = clamp(lane, -5.1, 5.1);
  car.aimLane = damp(car.aimLane || 0, lane, 4.2, dt);
  const tx = look.x + look.rx * car.aimLane;
  const tz = look.z + look.rz * car.aimLane;
  const dx = tx - car.x;
  const dz = tz - car.z;
  const fwdX = Math.sin(car.yaw);
  const fwdZ = Math.cos(car.yaw);
  const rightX = Math.cos(car.yaw);
  const rightZ = -Math.sin(car.yaw);
  const ahead = dx * fwdX + dz * fwdZ;
  const side = dx * rightX + dz * rightZ;
  let steer;
  if (ahead < 1.5) {
    let diff = Math.atan2(here.tx, here.tz) - car.yaw;
    while (diff > Math.PI) diff -= Math.PI * 2;
    while (diff < -Math.PI) diff += Math.PI * 2;
    steer = clamp(diff * 1.8, -1, 1);
  } else {
    steer = clamp(-side / Math.max(6, ahead) * 2.1, -1, 1);
  }
  const signed = (car.x - here.x) * here.rx + (car.z - here.z) * here.rz;
  if (signed > 4.2) steer += 0.75;
  if (signed < -4.2) steer -= 0.75;
  car.cmdSteer = damp(car.cmdSteer || 0, clamp(steer, -1, 1), 9, dt);
  return { steer: car.cmdSteer, throttle, brake, grade: gradeAt(car) };
}

function collideAll() {
  for (let i = 0; i < cars.length; i++) {
    for (let j = i + 1; j < cars.length; j++) {
      const a = cars[i];
      const b = cars[j];
      const dx = b.x - a.x;
      const dz = b.z - a.z;
      const d = Math.hypot(dx, dz) || 0.0001;
      const min = 2.2;
      if (d >= min) continue;
      const nx = dx / d;
      const nz = dz / d;
      const push = min - d;
      a.x -= nx * push * 0.5;
      a.z -= nz * push * 0.5;
      b.x += nx * push * 0.5;
      b.z += nz * push * 0.5;
      const rel = (a.vx - b.vx) * nx + (a.vz - b.vz) * nz;
      if (rel > 0) {
        const impulse = rel * 0.85;
        a.vx -= nx * impulse;
        a.vz -= nz * impulse;
        b.vx += nx * impulse;
        b.vz += nz * impulse;
      }
      if (a === player || b === player) {
        if (push > 0.3 && hitCd <= 0) {
          hitCd = 0.16;
          buzz(8);
        }
      }
    }
  }
}

function resolveWalls(car, dt) {
  let scraped = 0;
  const limit = ROAD_HALF - 1.08;
  for (let pass = 0; pass < 2; pass++) {
    const axles = [1.25, -1.2];
    for (let a = 0; a < axles.length; a++) {
      const dist = axles[a];
      const x = car.x + Math.sin(car.yaw) * dist;
      const z = car.z + Math.cos(car.yaw) * dist;
      const q = a === 0 ? qA : qB;
      closest(x, z, q, car.seg);
      const over = Math.abs(q.signed) - limit;
      if (over <= 0) continue;
      const sign = Math.sign(q.signed) || 1;
      car.x -= q.rx * sign * (over + 0.03);
      car.z -= q.rz * sign * (over + 0.03);
      const outward = (car.vx * q.rx + car.vz * q.rz) * sign;
      if (outward > 0) {
        car.vx -= q.rx * sign * outward;
        car.vz -= q.rz * sign * outward;
        scraped = Math.max(scraped, outward);
      }
    }
  }
  if (scraped > 3.2) {
    const keep = Math.max(0.9, 1 - clamp(scraped, 0, 14) * 0.28 * dt);
    car.vx *= keep;
    car.vz *= keep;
    if (!reduceMotion) shake = Math.min(0.5, shake + scraped * 0.012);
    car.yawRate *= Math.max(0.45, 1 - scraped * 0.05);
    scrapeLeft = 0.14;
    if (scraped > 7 && hitCd <= 0) {
      hitCd = 0.26;
      buzz(14);
      beep(70, 0.07, 'square', 0.04);
      for (let i = 0; i < 5; i++) {
        spawnPuff(
          car.x, car.surfaceY + 0.45, car.z,
          0xffe1a8, 0.32, 0.22,
          (Math.random() - 0.5) * 8, 2 + Math.random() * 2, (Math.random() - 0.5) * 8
        );
      }
    }
  }
  const sp = Math.hypot(car.vx, car.vz);
  if (sp > 80) {
    car.vx *= 80 / sp;
    car.vz *= 80 / sp;
  }
  car.sp = Math.hypot(car.vx, car.vz);
}

function noteCarProgress(car, nextT) {
  let delta = nextT - car.prevT;
  if (delta < -0.5) delta += 1;
  if (delta > 0.5) delta -= 1;
  const crossed = delta > 0 && delta < 0.2 && car.prevT > 0.75 && nextT < 0.25;
  car.prevT = nextT;
  car.t = nextT;
  if (!crossed || state !== 'race') return;
  const lt = raceTime - car.lapStamp;
  if (lt < minLap) return;
  car.lapStamp = raceTime;
  car.lap += 1;
  if (car !== player) {
    if (car.lap >= LAPS && !car.finished) {
      car.finished = true;
      car.finishTime = raceTime;
    }
    return;
  }
  lapStamp = car.lapStamp;
  if (lt < sessionBest) sessionBest = lt;
  if (car.lap >= LAPS) {
    car.finished = true;
    car.finishTime = raceTime;
    beep(880, 0.14, 'square', 0.055);
    beep(1320, 0.26, 'triangle', 0.05);
    buzz([12, 40, 18]);
    setState('finish');
    return;
  }
  flash('LAP ' + car.lap + '  ' + formatTime(lt));
  beep(680, 0.09, 'square', 0.05);
  buzz(10);
}

function checkPads() {
  for (const pad of pads) {
    if (!pad.held) pad.held = new Set();
    for (const car of cars) {
      let d = Math.abs(car.t - pad.t);
      if (d > 0.5) d = 1 - d;
      const on = d < pad.half;
      if (on && !pad.held.has(car.name)) {
        pad.held.add(car.name);
        pad.flash = 0.35;
        car.boost = TUNING.boostTime;
        if (car === player) {
          beep(760, 0.1, 'sawtooth', 0.04);
          buzz(10);
        }
      } else if (!on) pad.held.delete(car.name);
    }
  }
}

function syncCars(dt) {
  for (const car of cars) {
    car.root.position.set(car.x, car.surfaceY, car.z);
    car.root.rotation.y = car.yaw;
    const steerVis = car.ai ? car.visualSteer : steerSmooth;
    if (dt > 0) {
      const ahead = frameAt(car.t + 0.012);
      const here = frameAt(car.t);
      const grade = (ahead.y - here.y) / Math.max(1, trackLength * 0.012);
      car.body.rotation.x = damp(car.body.rotation.x, clamp(-grade * 0.85, -0.2, 0.16), 6, dt);
      car.body.rotation.z = damp(car.body.rotation.z, steerVis * 0.22, 8, dt);
      car.spin += (car.sp || 0) * dt * 2.4;
    }
    for (const w of car.wheels) {
      w.pivot.rotation.y = w.front ? -steerVis * 0.5 : 0;
      w.spin.rotation.x = car.spin;
    }
    car.tailMat.emissiveIntensity = car.braking ? 2.2 : 0.32;
    const boosting = !car.ai && car.boost > 0;
    car.glow.visible = boosting;
    if (boosting) car.glow.material.opacity = 0.4 + Math.sin(performance.now() * 0.018) * 0.15;
    car.blob.position.set(car.x, car.surfaceY + 0.14, car.z);
    car.blob.rotation.set(-Math.PI / 2, 0, 0);
  }
}

function animatePads(dt) {
  const t = performance.now() * 0.001;
  for (const pad of pads) {
    if (pad.flash > 0) pad.flash -= dt;
    const pulse = 0.65 + Math.sin(t * 4.2 + pad.phase) * 0.35;
    pad.mat.emissiveIntensity = pad.flash > 0 ? 3.4 : pulse;
  }
}

function updateLights(phase) {
  startLights.forEach((mesh, i) => {
    const mat = mesh.material;
    if (phase === 4) {
      mat.color.setHex(0x39ff7a);
      mat.emissive.setHex(0x39ff7a);
      mat.emissiveIntensity = 2.4;
    } else if (phase > 0 && i < phase) {
      mat.color.setHex(0xff2a2a);
      mat.emissive.setHex(0xff2a2a);
      mat.emissiveIntensity = 2.1;
    } else {
      mat.color.setHex(0x3a2424);
      mat.emissive.setHex(0x000000);
      mat.emissiveIntensity = 0;
    }
  });
}

function standings() {
  return cars.slice().sort((a, b) => rankKey(b) - rankKey(a));
}
function rankKey(car) {
  if (car.finished) return 1000 - car.finishTime;
  return car.lap + car.t;
}
function placeLabel(index) {
  return ['1ST', '2ND', '3RD', '4TH'][index] || (index + 1) + 'TH';
}
function formatTime(s) {
  if (!Number.isFinite(s)) return '—';
  const m = Math.floor(s / 60);
  const rem = s - m * 60;
  const whole = Math.floor(rem);
  const ms = Math.floor((rem - whole) * 1000);
  return m + ':' + String(whole).padStart(2, '0') + '.' + String(ms).padStart(3, '0');
}
function setText(el, text) {
  if (el.textContent !== text) el.textContent = text;
}

function updateHud() {
  const rows = standings();
  const place = Math.max(0, rows.indexOf(player));
  setText(posEl, placeLabel(place));
  posEl.dataset.place = String(place + 1);
  setText(lapEl, 'LAP ' + Math.min(player.lap + 1, LAPS) + '/' + LAPS);
  setText(bestEl, 'BEST ' + (Number.isFinite(sessionBest) ? formatTime(sessionBest) : '—'));
  timeEl.textContent = formatTime(raceTime);
  speedEl.textContent = String(Math.max(0, Math.round(player.sp * 3.15)));
  boostFlag.classList.toggle('show', player.boost > 0);
}

function fillResults() {
  const rows = standings();
  const place = Math.max(0, rows.indexOf(player));
  finishTitle.textContent = ['YOU WIN', '2ND PLACE', '3RD PLACE', '4TH PLACE'][place] || 'FINISHED';
  finishTitle.classList.toggle('win', place === 0);
  finishTime.textContent = formatTime(player.finishTime ?? raceTime);
  const eyebrow = document.querySelector('#panel-finish .eyebrow');
  if (eyebrow) eyebrow.textContent = Number.isFinite(sessionBest) ? 'BEST LAP  ' + formatTime(sessionBest) : 'CHECKERED FLAG';
  resultsEl.replaceChildren();
  rows.forEach((car, i) => {
    const li = document.createElement('li');
    if (!car.ai) li.classList.add('is-you');
    const left = document.createElement('span');
    left.textContent = placeLabel(i) + '   ' + car.name;
    const right = document.createElement('span');
    right.textContent = car.finished ? formatTime(car.finishTime) : 'RUNNING';
    li.append(left, right);
    resultsEl.appendChild(li);
  });
}

function flash(text) {
  bannerEl.textContent = text;
  bannerEl.classList.add('show');
  bannerLeft = 1.25;
}
function showCount(text) {
  countEl.textContent = text;
  countEl.classList.add('show');
}
function hideCount() {
  countEl.classList.remove('show');
  countEl.textContent = '';
}

function setState(next) {
  state = next;
  app.classList.remove('is-title', 'is-countdown', 'is-race', 'is-paused', 'is-finish');
  app.classList.add('is-' + next);
  if (next === 'paused' || next === 'finish' || next === 'title') clearInput();
  if (next !== 'countdown' && next !== 'race') hideCount();
  if (next === 'paused') {
    const place = Math.max(0, standings().indexOf(player));
    pauseStat.textContent = placeLabel(place) + '    ·    ' + formatTime(raceTime);
  }
  if (next === 'finish') {
    fillResults();
    againBtn.focus();
  }
}

function clearInput() {
  brakeHeld = false;
  brakeBtn.classList.remove('is-hot');
  resetStick();
}

function resetStick() {
  stickX = 0;
  stickY = 0;
  stickId = null;
  if (stickKnob) stickKnob.style.transform = 'translate(0px, 0px)';
}

function readSteer() {
  if (state !== 'race') return 0;
  let s = stickX;
  if (keys.has('ArrowLeft') || keys.has('KeyA')) s -= 1;
  if (keys.has('ArrowRight') || keys.has('KeyD')) s += 1;
  if (Math.abs(s) < 0.04 && tiltOn) s = tiltSteer;
  return clamp(s, -1, 1);
}
function readBrakeAmount() {
  if (state !== 'race') return 0;
  let amount = 0;
  if (brakeHeld || keys.has('Space')) amount = 1;
  amount = Math.max(amount, clamp(-stickY - 0.18, 0, 1) / 0.82);
  return clamp(amount, 0, 1);
}

function startGame() {
  unlockAudio();
  if (tiltWanted && !tiltOn) {
    enableTilt().then((ok) => {
      tiltOn = !!ok;
      if (!tiltOn) tiltWanted = false;
      savePrefs();
      renderToggles();
    });
  }
  beginCountdown();
}

function bindUI() {
  const moveStick = (e) => {
    const rect = stick.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = e.clientX - cx;
    let dy = e.clientY - cy;
    const max = rect.width * 0.36;
    const mag = Math.hypot(dx, dy) || 1;
    if (mag > max) {
      dx *= max / mag;
      dy *= max / mag;
    }
    stickKnob.style.transform = 'translate(' + dx + 'px, ' + dy + 'px)';
    stickX = dx / max;
    stickY = -dy / max;
  };
  stick.addEventListener('pointerdown', (e) => {
    if (state !== 'race' && state !== 'countdown') return;
    e.preventDefault();
    stick.setPointerCapture(e.pointerId);
    stickId = e.pointerId;
    moveStick(e);
  }, { passive: false });
  stick.addEventListener('pointermove', (e) => {
    if (e.pointerId !== stickId) return;
    moveStick(e);
  });
  const endStick = (e) => {
    if (e.pointerId !== stickId) return;
    resetStick();
  };
  stick.addEventListener('pointerup', endStick);
  stick.addEventListener('pointercancel', endStick);
  const brakeDown = (e) => {
    if (state !== 'race' && state !== 'countdown') return;
    e.preventDefault();
    brakeBtn.setPointerCapture(e.pointerId);
    brakeHeld = true;
    brakeBtn.classList.add('is-hot');
  };
  const brakeUp = () => {
    brakeHeld = false;
    brakeBtn.classList.remove('is-hot');
  };
  brakeBtn.addEventListener('pointerdown', brakeDown, { passive: false });
  brakeBtn.addEventListener('pointerup', brakeUp);
  brakeBtn.addEventListener('pointercancel', brakeUp);

  startBtn.addEventListener('click', startGame);
  resumeBtn.addEventListener('click', () => {
    unlockAudio();
    if (state === 'paused') setState('race');
  });
  retryBtn.addEventListener('click', () => { unlockAudio(); startGame(); });
  againBtn.addEventListener('click', () => { unlockAudio(); startGame(); });
  pauseBtn.addEventListener('click', () => {
    if (state === 'race') setState('paused');
  });
  soundBtn.addEventListener('click', () => {
    unlockAudio();
    soundOn = !soundOn;
    savePrefs();
    renderToggles();
    if (!soundOn) updateEngine(0, false);
  });
  tiltBtn.addEventListener('click', async () => {
    unlockAudio();
    if (tiltWanted) {
      tiltWanted = false;
      tiltOn = false;
      tiltSteer = 0;
      window.removeEventListener('deviceorientation', onTilt);
      savePrefs();
      renderToggles();
      return;
    }
    tiltWanted = true;
    tiltOn = await enableTilt();
    if (!tiltOn) {
      tiltWanted = false;
      tiltBtn.textContent = 'TILT BLOCKED';
      savePrefs();
      setTimeout(renderToggles, 1000);
      return;
    }
    savePrefs();
    renderToggles();
  });

  window.addEventListener('keydown', (e) => {
    keys.add(e.code);
    if (['Space', 'ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(e.code)) e.preventDefault();
    if (e.repeat) return;
    if ((e.code === 'Enter' || e.code === 'Space') && state === 'title') startGame();
    if ((e.code === 'Enter' || e.code === 'KeyR') && (state === 'finish' || state === 'paused')) startGame();
    if (e.code === 'KeyR' && state === 'race') startGame();
    if (e.code === 'KeyP') {
      if (state === 'race') setState('paused');
      else if (state === 'paused') setState('race');
    }
  });
  window.addEventListener('keyup', (e) => keys.delete(e.code));
  window.addEventListener('blur', () => { keys.clear(); clearInput(); });
  window.addEventListener('contextmenu', (e) => e.preventDefault());
  app.addEventListener('touchmove', (e) => e.preventDefault(), { passive: false });
}

function onTilt(e) {
  const angle = (screen.orientation && screen.orientation.angle) || window.orientation || 0;
  let axis;
  if (angle === 90) axis = e.beta || 0;
  else if (angle === 270 || angle === -90) axis = -(e.beta || 0);
  else axis = e.gamma || 0;
  const raw = clamp(axis / 26, -1, 1);
  const dead = 0.08;
  tiltSteer = Math.abs(raw) < dead ? 0 : Math.sign(raw) * (Math.abs(raw) - dead) / (1 - dead);
}

async function enableTilt() {
  try {
    const Ev = window.DeviceOrientationEvent;
    if (!Ev) return false;
    if (typeof Ev.requestPermission === 'function') {
      const res = await Ev.requestPermission();
      if (res !== 'granted') return false;
    }
    window.removeEventListener('deviceorientation', onTilt);
    window.addEventListener('deviceorientation', onTilt);
    return true;
  } catch (err) {
    return false;
  }
}

function renderToggles() {
  soundBtn.textContent = soundOn ? 'SOUND ON' : 'SOUND OFF';
  soundBtn.classList.toggle('is-on', soundOn);
  soundBtn.setAttribute('aria-pressed', soundOn ? 'true' : 'false');
  tiltBtn.textContent = tiltWanted ? 'TILT ON' : 'TILT OFF';
  tiltBtn.classList.toggle('is-on', tiltWanted);
  tiltBtn.setAttribute('aria-pressed', tiltWanted ? 'true' : 'false');
}

function loadPref(key, fallback) {
  try {
    const v = localStorage.getItem(key);
    if (v === null) return fallback;
    return v === '1';
  } catch (err) {
    return fallback;
  }
}
function savePrefs() {
  try {
    localStorage.setItem('sunline-sound', soundOn ? '1' : '0');
    localStorage.setItem('sunline-tilt', tiltWanted ? '1' : '0');
  } catch (err) { /* private mode */ }
}

function unlockAudio() {
  const AC = window.AudioContext || window.webkitAudioContext;
  if (!AC) return;
  if (!audio.ready) {
    const ctx = new AC();
    audio.ctx = ctx;
    const gain = ctx.createGain();
    gain.gain.value = 0;
    const filter = ctx.createBiquadFilter();
    filter.type = 'lowpass';
    filter.frequency.value = 220;
    const oscA = ctx.createOscillator();
    oscA.type = 'sawtooth';
    oscA.frequency.value = 48;
    const oscB = ctx.createOscillator();
    oscB.type = 'triangle';
    oscB.frequency.value = 96;
    const mixA = ctx.createGain();
    mixA.gain.value = 0.7;
    const mixB = ctx.createGain();
    mixB.gain.value = 0.22;
    oscA.connect(mixA).connect(filter);
    oscB.connect(mixB).connect(filter);
    filter.connect(gain).connect(ctx.destination);
    oscA.start();
    oscB.start();
    const len = Math.floor(ctx.sampleRate * 0.25);
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const data = buf.getChannelData(0);
    for (let i = 0; i < len; i++) data[i] = Math.random() * 2 - 1;
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.loop = true;
    const sg = ctx.createGain();
    sg.gain.value = 0;
    const sf = ctx.createBiquadFilter();
    sf.type = 'bandpass';
    sf.frequency.value = 1000;
    src.connect(sf).connect(sg).connect(ctx.destination);
    src.start();
    audio.gain = gain;
    audio.filter = filter;
    audio.oscA = oscA;
    audio.oscB = oscB;
    audio.scrape = sg;
    audio.ready = true;
  }
  if (audio.ctx.state === 'suspended') audio.ctx.resume();
}

function beep(freq, dur, type, vol) {
  if (!audio.ready || !soundOn) return;
  const ctx = audio.ctx;
  const o = ctx.createOscillator();
  const g = ctx.createGain();
  o.type = type || 'square';
  o.frequency.value = freq;
  g.gain.setValueAtTime(vol, ctx.currentTime);
  g.gain.exponentialRampToValueAtTime(0.0001, ctx.currentTime + dur);
  o.connect(g).connect(ctx.destination);
  o.start();
  o.stop(ctx.currentTime + dur + 0.02);
}

function updateEngine(speed, active) {
  if (!audio.ready) return;
  const ctx = audio.ctx;
  const vol = soundOn && active ? 0.022 + Math.min(speed, 55) * 0.00045 : 0;
  audio.gain.gain.setTargetAtTime(vol, ctx.currentTime, 0.06);
  const freq = 40 + Math.max(0, speed) * 2.35;
  audio.oscA.frequency.setTargetAtTime(freq, ctx.currentTime, 0.05);
  audio.oscB.frequency.setTargetAtTime(freq * 2.01, ctx.currentTime, 0.05);
  audio.filter.frequency.setTargetAtTime(150 + Math.max(0, speed) * 40, ctx.currentTime, 0.05);
  const scrapeVol = soundOn && scrapeLeft > 0 ? 0.028 : 0;
  audio.scrape.gain.setTargetAtTime(scrapeVol, ctx.currentTime, 0.03);
}

function buzz(pattern) {
  if (navigator.vibrate) navigator.vibrate(pattern);
}

if (typeof document !== 'undefined' && document.getElementById('app')) {
  try {
    boot();
  } catch (err) {
    console.error(err);
    window.__sunline = 'error';
    const lede = document.querySelector('#panel-title .lede');
    if (lede) lede.textContent = 'The track failed to build. Reload the page.';
  }
}

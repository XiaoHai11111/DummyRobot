import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const $ = (id) => document.getElementById(id);
const API = '/api';
const app = {
  config: null,
  robot: { connected: false, enabled: false, joints_deg: [0, 0, 90, 0, 0, 0], gripper_deg: 0 },
  camera: { connected: false },
  targetJoints: [0, 0, 90, 0, 0, 0],
  lastCommandedTargets: null,
  sliderActive: new Set(),
  jointSendTimer: null,
  statusPolling: false,
  pendingMove: false,
  jointCommandQueued: false,
  commandBusy: false,
  localPoseBusy: false,
  localPoseKind: '',
  modelJoints: new Map(),
  robotVisuals: [],
  robotRoot: null,
};

async function api(path, body) {
  const response = await fetch(`${API}${path}`, {
    method: body === undefined ? 'GET' : 'POST',
    headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function log(message, kind = '') {
  const list = $('logList');
  const row = document.createElement('div');
  row.className = `log-entry ${kind}`;
  const time = document.createElement('time');
  time.textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  const text = document.createElement('span');
  text.textContent = message;
  row.append(time, text);
  list.prepend(row);
  while (list.children.length > 30) list.lastElementChild.remove();
}

function appendCommandHistory(command, reply, kind = '') {
  const history = $('commandHistory');
  history.querySelector('.command-placeholder')?.remove();
  const row = document.createElement('div');
  row.className = `command-history-row ${kind}`;
  const sent = document.createElement('code');
  sent.textContent = `> ${command}`;
  const received = document.createElement('span');
  received.textContent = reply;
  row.append(sent, received);
  history.append(row);
  while (history.children.length > 12) history.firstElementChild.remove();
  history.scrollTop = history.scrollHeight;
}

function commandNeedsConfirmation(command) {
  return /^[>@&]/.test(command)
    || /^!START$/.test(command)
    || /^#GRIP(?:\s|_CURRENT\s)/.test(command)
    || /^#CMDMODE\s/.test(command);
}

async function sendAsciiCommand() {
  const input = $('asciiCommand');
  const command = input.value.trim();
  if (!command || !app.robot.connected || app.commandBusy) return;
  if (commandNeedsConfirmation(command)
      && !window.confirm(`即将发送机械臂指令：\n${command}\n\n请确认机械臂周围安全，是否继续？`)) return;

  app.commandBusy = true;
  setControlAvailability();
  try {
    const result = await api('/robot/command', { command });
    const reply = [result.reply, result.note].filter(Boolean).join(' · ');
    appendCommandHistory(command, reply || '已发送', command === '!STOP' ? 'warn' : '');
    log(`ASCII ${command} → ${reply || '已发送'}`, command === '!STOP' ? 'warn' : 'success');
    if (typeof result.enabled === 'boolean') app.robot.enabled = result.enabled;
    await pollStatus();
  } catch (error) {
    appendCommandHistory(command, error.message, 'error');
    log(`ASCII 指令失败：${error.message}`, 'error');
    toast(`指令发送失败：${error.message}`, true);
  } finally {
    app.commandBusy = false;
    setControlAvailability();
  }
}

function toast(message, isError = false) {
  const item = document.createElement('div');
  item.className = `toast${isError ? ' error' : ''}`;
  item.textContent = message;
  $('toastStack').append(item);
  setTimeout(() => item.remove(), 3600);
}

function setLed(id, online, cyan = false) {
  const led = $(id);
  led.classList.toggle('led-muted', !online);
  led.classList.toggle('led-cyan', online && cyan);
}

function setControlAvailability() {
  const connected = app.robot.connected;
  const enabled = connected && app.robot.enabled;
  const poseOperationBusy = Boolean(app.robot.busy || app.localPoseBusy);
  $('connectRobot').disabled = poseOperationBusy;
  $('enableRobot').disabled = !connected || poseOperationBusy;
  $('enableRobot').classList.toggle('is-enabled', app.robot.enabled);
  $('enableRobot').querySelector('span:last-child').textContent = app.robot.enabled ? '失能' : '使能';
  $('enableRobot').title = app.robot.enabled ? '点击失能机械臂' : '点击使能机械臂';
  $('stopRobot').disabled = !connected;
  $('asciiCommand').disabled = !connected;
  $('sendAsciiCommand').disabled = !connected || app.commandBusy;
  $('homeRobot').disabled = !enabled || Boolean(app.robot.busy);
  $('restRobot').disabled = !enabled || Boolean(app.robot.busy);
  $('gripperSlider').disabled = !enabled || poseOperationBusy;
  $('gripperOpen').disabled = !enabled || poseOperationBusy;
  $('gripperClose').disabled = !enabled || poseOperationBusy;
  document.querySelectorAll('.joint-slider').forEach((el) => { el.disabled = !enabled || poseOperationBusy; });
  const state = $('controlState');
  state.textContent = enabled ? `READY · ${app.config?.robot?.joint_speed_deg_s ?? 100}°/s` : 'LOCKED';
  state.classList.toggle('ready', enabled);
}

function renderPorts(ports) {
  const select = $('portSelect');
  const preferred = app.robot.port || app.config?.serial?.default_port || '';
  select.replaceChildren();
  if (!ports.length) {
    const opt = new Option('未发现串口', '');
    select.add(opt);
    return;
  }
  ports.forEach((p) => {
    const label = `${p.device} · ${p.description || 'serial device'}${p.vid ? ` · ${Number(p.vid).toString(16).padStart(4, '0')}:${Number(p.pid).toString(16).padStart(4, '0')}` : ''}`;
    select.add(new Option(label, p.device));
  });
  if (ports.some((p) => p.device === preferred)) select.value = preferred;
  else select.selectedIndex = 0;
}

async function scanPorts() {
  try {
    const data = await api('/serial/scan');
    renderPorts(data.ports || []);
    log(`串口扫描完成：${data.ports.length} 个设备`, 'success');
  } catch (error) { log(`串口扫描失败：${error.message}`, 'error'); toast(error.message, true); }
}

function createJointControls() {
  const container = $('jointControls');
  container.replaceChildren();
  const specs = app.config.robot.joints;
  app.targetJoints = [...app.config.robot.resting_pose_deg];
  specs.forEach((spec, index) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    const head = document.createElement('div');
    head.className = 'joint-head';
    const name = document.createElement('span');
    name.className = 'joint-name';
    name.innerHTML = `<span class="joint-index">0${index + 1}</span><span>${spec.name}</span>`;
    const value = document.createElement('span');
    value.className = 'joint-value';
    value.id = `jointValue${index}`;
    value.innerHTML = `<span class="joint-target-value">${Number(app.targetJoints[index]).toFixed(1)}°</span><small class="joint-actual-value" id="jointActual${index}">ACT —°</small>`;
    head.append(name, value);
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'joint-slider';
    slider.min = spec.min_deg;
    slider.max = spec.max_deg;
    slider.step = index === 5 ? 1 : 0.5;
    slider.value = app.targetJoints[index];
    slider.disabled = true;
    slider.dataset.index = String(index);
    slider.setAttribute('aria-label', `${spec.name} target angle`);
    slider.addEventListener('pointerdown', () => app.sliderActive.add(index));
    slider.addEventListener('input', () => {
      app.sliderActive.add(index);
      app.targetJoints[index] = Number(slider.value);
      updateJointReadout(index);
      scheduleJointTargets();
    });
    slider.addEventListener('change', async () => {
      app.sliderActive.delete(index);
      if (app.jointSendTimer) {
        window.clearTimeout(app.jointSendTimer);
        app.jointSendTimer = null;
      }
      await sendJointTargets();
    });
    row.append(head, slider);
    container.append(row);
  });
}

function scheduleJointTargets() {
  if (app.jointSendTimer) return;
  app.jointSendTimer = window.setTimeout(() => {
    app.jointSendTimer = null;
    sendJointTargets();
  }, 100);
}

function updateJointReadout(index) {
  const target = app.targetJoints[index];
  const actual = app.robot.joints_deg?.[index];
  const value = $(`jointValue${index}`);
  if (!value || target === undefined) return;
  const actualText = Number.isFinite(Number(actual)) ? `${Number(actual).toFixed(1)}°` : '—°';
  value.innerHTML = `<span class="joint-target-value">${Number(target).toFixed(1)}°</span><small class="joint-actual-value">ACT ${actualText}</small>`;
}

function setJointTargets(angles) {
  app.targetJoints = angles.slice(0, 6).map(Number);
  app.targetJoints.forEach((angle, index) => {
    const slider = document.querySelector(`.joint-slider[data-index="${index}"]`);
    if (slider) slider.value = String(angle);
    updateJointReadout(index);
  });
}

function setPoseReadout(pose) {
  if (!pose || pose.length < 6) return;
  ['poseX', 'poseY', 'poseZ', 'poseA', 'poseB', 'poseC'].forEach((id, i) => {
    $(id).textContent = Number(pose[i]).toFixed(i < 3 ? 1 : 1);
  });
  $('poseReadout').textContent = `TCP X ${Number(pose[0]).toFixed(0)} · Y ${Number(pose[1]).toFixed(0)} · Z ${Number(pose[2]).toFixed(0)} mm`;
}

function applyRobotStatus(robot) {
  const wasConnected = app.robot.connected;
  app.robot = { ...app.robot, ...robot };
  if (!wasConnected && app.robot.connected && app.robot.joints_deg?.length >= 6) {
    // Seed the targets from the live pose once on connection; telemetry must
    // not keep overwriting the user's slider targets after this point.
    setJointTargets(app.robot.joints_deg);
    app.lastCommandedTargets = [...app.robot.joints_deg];
  }
  setLed('robotLed', app.robot.connected, true);
  $('robotStatus').textContent = app.robot.connected ? (app.robot.enabled ? 'ENABLED' : 'CONNECTED') : 'OFFLINE';
  $('robotStatus').style.color = app.robot.enabled ? 'var(--green)' : '';
  $('connectRobot').querySelector('span:last-child').textContent = app.robot.connected ? '断开机械臂' : '连接机械臂';
  $('connectRobot').classList.toggle('button-primary', !app.robot.connected);
  $('connectRobot').classList.toggle('button-outline', app.robot.connected);
  if (app.robot.pose_mm_deg) setPoseReadout(app.robot.pose_mm_deg);
  if (app.robot.joints_deg) {
    app.robot.joints_deg.forEach((angle, index) => {
      updateJointReadout(index);
    });
  }
  if (app.robot.gripper_deg !== undefined) {
    $('gripperSlider').value = app.robot.gripper_deg;
    $('gripperValue').textContent = Number(app.robot.gripper_deg).toFixed(1);
  }
  if (app.robot.error) {
    $('telemetryMessage').textContent = app.robot.error;
    $('telemetryMessage').style.color = 'var(--amber)';
  } else {
    $('telemetryMessage').textContent = app.robot.connected ? '串口遥测正常 · #GETJPOS / #GETLPOS' : '等待机械臂反馈';
    $('telemetryMessage').style.color = '';
  }
  const activePose = app.robot.busy || app.localPoseKind;
  if (activePose) {
    $('homeRobot').innerHTML = activePose === 'homing' ? '<span>◌</span> Resetting…' : '<span>⌂</span> Homing';
    $('restRobot').innerHTML = activePose === 'resting' ? '<span>◌</span> Resetting…' : '<span>↺</span> Resting';
  } else {
    $('homeRobot').innerHTML = '<span>⌂</span> Homing';
    $('restRobot').innerHTML = '<span>↺</span> Resting';
  }
  setControlAvailability();
  updateRobotModel(app.robot.joints_deg || app.targetJoints, app.robot.gripper_deg || 0);
}

function applyCameraStatus(camera) {
  app.camera = { ...camera };
  setLed('cameraLed', camera.connected, true);
  $('cameraStatus').textContent = camera.connected ? 'STREAMING' : 'OFFLINE';
  $('cameraStatus').style.color = camera.connected ? 'var(--cyan)' : '';
  $('cameraBox').classList.toggle('connected', camera.connected);
  $('connectCamera').querySelector('span:last-child').textContent = camera.connected ? '断开相机' : '连接相机';
  if (camera.name) $('cameraDevice').textContent = camera.name;
  if (camera.intrinsics) {
    $('cameraResolution').textContent = `${camera.intrinsics.width} × ${camera.intrinsics.height}`;
    $('cameraDepth').textContent = `${Number(camera.intrinsics.depth_scale).toFixed(3)} m/unit`;
  }
}

async function pollStatus() {
  if (app.statusPolling) return;
  app.statusPolling = true;
  try {
    const data = await api('/status');
    app.config = data.config;
    applyRobotStatus(data.robot);
    applyCameraStatus(data.camera);
  } catch (error) {
    $('hostStatus').textContent = 'BRIDGE LOST';
    setLed('serverLed', false);
  } finally {
    app.statusPolling = false;
  }
}

async function sendJointTargets() {
  if (!app.robot.connected || !app.robot.enabled) return;
  if (app.pendingMove) {
    app.jointCommandQueued = true;
    return;
  }
  const actual = app.robot.joints_deg;
  if (!actual || actual.length < 6) {
    toast('尚未收到六个关节的实际角度，暂不能发送目标', true);
    return;
  }
  const deadband = Number(app.config.robot.joint_command_deadband_deg ?? 0.2);
  const differsFromActual = app.targetJoints.some((angle, i) => Math.abs(Number(angle) - Number(actual[i])) > deadband);
  const differsFromLastCommand = app.lastCommandedTargets?.some((angle, i) => Math.abs(Number(app.targetJoints[i]) - Number(angle)) > deadband) ?? false;
  if (!differsFromActual && !differsFromLastCommand) {
    log('滑块目标与机械臂实际角度一致，无需发送', 'muted');
    return;
  }
  app.pendingMove = true;
  app.jointCommandQueued = false;
  const requestedTargets = [...app.targetJoints];
  let commandAccepted = false;
  try {
    const result = await api('/robot/joints', { angles: requestedTargets });
    app.lastCommandedTargets = [...(result.joints_deg || requestedTargets)];
    commandAccepted = true;
    log(`关节目标已发送 · ${result.joints_deg.map((v) => Number(v).toFixed(1)).join(' / ')}°`, 'success');
    $('telemetryMessage').textContent = `Joint command sent · ${app.config.robot.joint_speed_deg_s}°/s`;
  } catch (error) {
    toast(`关节指令失败：${error.message}`, true);
    log(`关节指令失败：${error.message}`, 'error');
    await pollStatus();
  } finally {
    app.pendingMove = false;
    const queued = app.jointCommandQueued;
    app.jointCommandQueued = false;
    if (commandAccepted && queued && app.targetJoints.some((angle, i) => Math.abs(Number(angle) - Number(requestedTargets[i])) > deadband)) {
      queueMicrotask(sendJointTargets);
    }
  }
}

async function moveToPose(kind) {
  const label = kind === 'homing' ? 'Homing 姿态 [0, 0, 90, 0, 0, 0]' : 'Resting 姿态 [0, -75, 180, 0, 0, 0]';
  if (!window.confirm(`将机械臂移动到${label}？请确认工作区域安全。`)) return;
  const button = kind === 'homing' ? $('homeRobot') : $('restRobot');
  const original = button.innerHTML;
  app.localPoseBusy = true;
  app.localPoseKind = kind;
  const poseTargets = kind === 'homing' ? app.config.robot.homing_pose_deg : app.config.robot.resting_pose_deg;
  setJointTargets(poseTargets);
  setControlAvailability();
  button.disabled = true;
  button.innerHTML = '<span>◌</span> Resetting…';
  try {
    await api(`/robot/${kind}`, {});
    // Keep the controls at the configured preset; telemetry can differ by a
    // small amount because the pose operation accepts a tolerance.
    setJointTargets(poseTargets);
    app.lastCommandedTargets = [...poseTargets];
    log(`${kind === 'homing' ? 'Homing' : 'Resting'} 关节姿态到达`, 'success');
    toast(`${kind === 'homing' ? 'Homing' : 'Resting'} 姿态完成`);
  } catch (error) {
    toast(error.message, true);
    log(`姿态操作失败：${error.message}`, 'error');
  } finally {
    app.localPoseBusy = false;
    app.localPoseKind = '';
    button.innerHTML = original;
    await pollStatus();
  }
}

async function initSerial() {
  await scanPorts();
  $('scanPorts').addEventListener('click', scanPorts);
  $('connectRobot').addEventListener('click', async () => {
    const button = $('connectRobot');
    button.disabled = true;
    try {
      if (app.robot.connected) {
        await api('/serial/disconnect', {});
        app.robot.connected = false;
        app.robot.enabled = false;
        log('机械臂串口已断开', 'warn');
      } else {
        const result = await api('/serial/connect', {
          port: $('portSelect').value,
          baudrate: Number($('baudrate').value),
          bytesize: Number($('bytesize').value),
          parity: $('parity').value,
          stopbits: Number($('stopbits').value),
        });
        app.robot.port = result.port;
        log(`已连接 ${result.port}`, 'success');
        toast(`机械臂已连接：${result.port}`);
      }
      await pollStatus();
    } catch (error) { log(`串口操作失败：${error.message}`, 'error'); toast(error.message, true); }
    finally { button.disabled = false; }
  });
  $('enableRobot').addEventListener('click', async () => {
    try {
      const enabled = !app.robot.enabled;
      const result = await api('/robot/enable', { enabled });
      app.robot.enabled = result.enabled;
      applyRobotStatus(app.robot);
      log(enabled ? '机械臂已使能' : '机械臂已失能', enabled ? 'success' : 'warn');
      toast(enabled ? '机械臂已使能' : '机械臂已失能');
    } catch (error) { toast(error.message, true); log(error.message, 'error'); }
  });
  $('stopRobot').addEventListener('click', async () => {
    if (!window.confirm('发送 STOP 指令以停止机械臂运动？')) return;
    try { const result = await api('/robot/stop', {}); log(`STOP · ${result.reply}`, 'warn'); toast('STOP 指令已发送'); }
    catch (error) { toast(error.message, true); }
  });
  $('homeRobot').addEventListener('click', () => moveToPose('homing'));
  $('restRobot').addEventListener('click', () => moveToPose('resting'));
  $('sendAsciiCommand').addEventListener('click', sendAsciiCommand);
  $('asciiCommand').addEventListener('keydown', (event) => {
    if (event.key === 'Enter') {
      event.preventDefault();
      sendAsciiCommand();
    }
  });
  $('clearLog').addEventListener('click', () => $('logList').replaceChildren());
}

async function setGripper(angle, actionLabel) {
  if (!app.robot.enabled) return;
  try {
    const result = await api('/robot/gripper', { angle });
    app.robot.gripper_deg = result.gripper_deg;
    $('gripperSlider').value = result.gripper_deg;
    $('gripperValue').textContent = Number(result.gripper_deg).toFixed(1);
    updateRobotModel(app.robot.joints_deg || app.targetJoints, result.gripper_deg);
    log(`夹爪${actionLabel} · ${Number(result.gripper_deg).toFixed(1)}°`, 'success');
  } catch (error) { toast(error.message, true); log(`夹爪控制失败：${error.message}`, 'error'); }
}

function initGripper() {
  $('gripperSlider').addEventListener('input', (event) => {
    $('gripperValue').textContent = Number(event.target.value).toFixed(1);
    updateRobotModel(app.robot.joints_deg || app.targetJoints, Number(event.target.value));
  });
  $('gripperSlider').addEventListener('change', (event) => setGripper(Number(event.target.value), '目标'));
  $('gripperOpen').addEventListener('click', () => setGripper(app.config.robot.gripper.open_angle_deg, '打开'));
  $('gripperClose').addEventListener('click', () => setGripper(app.config.robot.gripper.closed_angle_deg, '关闭'));
}

async function initCamera() {
  $('connectCamera').addEventListener('click', async () => {
    const button = $('connectCamera');
    button.disabled = true;
    try {
      if (app.camera.connected) {
        $('cameraFeed').removeAttribute('src');
        await api('/camera/disconnect', {});
        applyCameraStatus({ connected: false });
        log('D435 相机已断开', 'warn');
      } else {
        const result = await api('/camera/connect', {});
        applyCameraStatus(result);
        $('cameraFeed').src = `/api/camera/stream?t=${Date.now()}`;
        log(`相机已连接 · ${result.name}`, 'success');
        toast(`相机已连接：${result.name}`);
      }
    } catch (error) { applyCameraStatus({ connected: false }); log(`相机连接失败：${error.message}`, 'error'); toast(error.message, true); }
    finally { button.disabled = false; }
  });
  $('cameraFeed').addEventListener('error', () => {
    if (app.camera.connected) { $('cameraDepth').textContent = 'stream error'; }
  });
}

// -------- Three.js robot viewport: URDF/MJCF XML is normalized by server.py. --------
let scene, camera3d, renderer, controls, stlLoader, renderFrame;

function rpyQuaternion(rpy = [0, 0, 0]) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Number(rpy[0] || 0));
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Number(rpy[1] || 0));
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Number(rpy[2] || 0));
  return qz.multiply(qy).multiply(qx);
}

function applyOrigin(group, origin = {}) {
  const xyz = origin.xyz || [0, 0, 0];
  group.position.set(Number(xyz[0]) || 0, Number(xyz[1]) || 0, Number(xyz[2]) || 0);
  if (origin.quat) {
    const [w, x, y, z] = origin.quat.map(Number);
    group.quaternion.set(x, y, z, w);
  } else group.quaternion.copy(rpyQuaternion(origin.rpy || [0, 0, 0]));
}

function colorFromRgba(rgba = [0.78, 0.84, 0.9, 1]) {
  return new THREE.Color(Number(rgba[0]) || 0, Number(rgba[1]) || 0, Number(rgba[2]) || 0);
}

function addVisual(parent, visual) {
  const frame = new THREE.Group();
  applyOrigin(frame, visual.origin);
  parent.add(frame);
  if (visual.kind === 'mesh') {
    stlLoader.load(`/api/assets/${encodeURIComponent(visual.file)}`, (geometry) => {
      geometry.computeVertexNormals();
      const rgba = visual.color || [0.8, 0.86, 0.92, 1];
      const material = new THREE.MeshStandardMaterial({
        color: colorFromRgba(rgba), metalness: 0.26, roughness: 0.53,
        transparent: Number(rgba[3] ?? 1) < 1, opacity: Number(rgba[3] ?? 1),
      });
      const mesh = new THREE.Mesh(geometry, material);
      const scale = visual.scale || [1, 1, 1];
      mesh.scale.set(Number(scale[0]) || 1, Number(scale[1]) || 1, Number(scale[2]) || 1);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      frame.add(mesh);
      app.robotVisuals.push(mesh);
    }, undefined, (error) => {
      console.error('STL load failed', visual.file, error);
      $('modelError').textContent = `网格加载失败：${visual.file}`;
      $('modelError').classList.remove('hidden');
    });
  } else {
    const d = visual.dimensions || {};
    let geometry;
    if (visual.kind === 'box') {
      const size = (d.size || '0.05 0.05 0.05').split(/\s+/).map(Number);
      geometry = new THREE.BoxGeometry(...size);
    } else if (visual.kind === 'cylinder') {
      geometry = new THREE.CylinderGeometry(Number(d.radius || 0.025), Number(d.radius || 0.025), Number(d.length || 0.05), 24);
      geometry.rotateX(Math.PI / 2);
    } else if (visual.kind === 'sphere') geometry = new THREE.SphereGeometry(Number(d.radius || 0.025), 24, 16);
    if (geometry) frame.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: colorFromRgba(visual.color) })));
  }
}

function buildModelNode(node, parent) {
  const origin = new THREE.Group();
  origin.name = `${node.name || 'node'}_origin`;
  applyOrigin(origin, node.origin);
  parent.add(origin);
  const motion = new THREE.Group();
  motion.name = `${node.name || 'node'}_motion`;
  origin.add(motion);
  if (node.joint && node.joint.type !== 'fixed') {
    const axis = new THREE.Vector3(...(node.joint.axis || [0, 0, 1])).normalize();
    const record = { group: motion, axis, type: node.joint.type, name: node.joint.name };
    app.modelJoints.set(node.joint.name, record);
  }
  (node.visuals || []).forEach((visual) => addVisual(motion, visual));
  (node.children || []).forEach((child) => buildModelNode(child, motion));
  return origin;
}

function updateRobotModel(jointsDeg = app.targetJoints, gripperDeg = app.robot.gripper_deg || 0) {
  if (!app.config) return;
  for (let i = 0; i < app.config.robot.joints.length; i++) {
    const spec = app.config.robot.joints[i];
    const joint = app.modelJoints.get(spec.model_joint);
    if (!joint) continue;
    const mappedDeg = Number(jointsDeg[i] || 0) * Number(spec.model_sign ?? 1) + Number(spec.model_offset_deg || 0);
    joint.group.quaternion.setFromAxisAngle(joint.axis, THREE.MathUtils.degToRad(mappedDeg));
  }
  const gripper = app.config.robot.gripper;
  const ratio = THREE.MathUtils.clamp((Number(gripperDeg) - gripper.min_deg) / Math.max(0.001, gripper.max_deg - gripper.min_deg), 0, 1);
  for (const name of gripper.model_joints) {
    const joint = app.modelJoints.get(name);
    if (joint) joint.group.quaternion.setFromAxisAngle(joint.axis, ratio * Number(gripper.model_open_rad));
  }
}

async function loadRobotModel(format) {
  $('modelLoading').classList.remove('hidden');
  $('modelError').classList.add('hidden');
  $('modelState').textContent = 'MODEL LOADING';
  app.modelJoints.clear();
  app.robotVisuals = [];
  if (app.robotRoot) scene.remove(app.robotRoot);
  app.robotRoot = new THREE.Group();
  scene.add(app.robotRoot);
  try {
    const model = await api(`/model?format=${format}`);
    buildModelNode(model.root, app.robotRoot);
    $('modelState').textContent = format.toUpperCase() + ' · LOADED';
    $('modelState').style.color = 'var(--green)';
    $('modelLoading').classList.add('hidden');
    updateRobotModel(app.robot.joints_deg || app.targetJoints, app.robot.gripper_deg || 0);
  } catch (error) {
    $('modelLoading').classList.add('hidden');
    $('modelState').textContent = 'MODEL ERROR';
    $('modelError').textContent = error.message;
    $('modelError').classList.remove('hidden');
    log(`模型加载失败：${error.message}`, 'error');
  }
}

function initViewport() {
  const canvas = $('robotCanvas');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.setClearColor(0x000000, 0);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  scene = new THREE.Scene();
  camera3d = new THREE.PerspectiveCamera(38, 1, 0.01, 25);
  camera3d.up.set(0, 0, 1);
  camera3d.position.set(0.92, -1.35, 0.95);
  controls = new OrbitControls(camera3d, canvas);
  controls.target.set(0, 0, 0.24);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.35;
  controls.maxDistance = 3.5;
  scene.add(new THREE.HemisphereLight(0xb6e5f4, 0x101820, 2.1));
  const keyLight = new THREE.DirectionalLight(0xc5efff, 2.7);
  keyLight.position.set(1.2, -1.2, 2.1); keyLight.castShadow = true; scene.add(keyLight);
  const rim = new THREE.DirectionalLight(0x3ca7ff, 1.35); rim.position.set(-1.2, 1.1, 1.3); scene.add(rim);
  const grid = new THREE.GridHelper(1.1, 22, 0x26576c, 0x173344);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.002;
  grid.material.transparent = true; grid.material.opacity = 0.42;
  scene.add(grid);
  const axes = new THREE.AxesHelper(0.17); axes.position.set(0, 0, 0.003); scene.add(axes);
  stlLoader = new STLLoader();
  const resize = () => {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    renderer.setSize(rect.width, rect.height, false);
    camera3d.aspect = rect.width / rect.height;
    camera3d.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(canvas.parentElement);
  resize();
  const render = () => {
    renderFrame = requestAnimationFrame(render);
    controls.update();
    renderer.render(scene, camera3d);
  };
  render();
  $('resetView').addEventListener('click', () => {
    camera3d.position.set(0.92, -1.35, 0.95);
    controls.target.set(0, 0, 0.24);
    controls.update();
  });
}

async function boot() {
  $('clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false }); }, 1000);
  try {
    const data = await api('/status');
    app.config = data.config;
    app.robot.joints_deg = [...app.config.robot.resting_pose_deg];
    app.robot.gripper_deg = Number(app.config.robot.initial_gripper_angle_deg);
    app.targetJoints = [...app.config.robot.resting_pose_deg];
    $('baudrate').value = String(app.config.serial.baudrate);
    $('bytesize').value = String(app.config.serial.bytesize);
    $('parity').value = app.config.serial.parity;
    $('stopbits').value = String(app.config.serial.stopbits);
    $('cameraResolution').textContent = `${app.config.camera.width} × ${app.config.camera.height}`;
    createJointControls();
    initViewport();
    await loadRobotModel('mjcf');
    await initSerial();
    initGripper();
    initCamera();
    await pollStatus();
    setInterval(pollStatus, Math.max(80, Number(app.config.serial.poll_interval_ms)));
    $('hostStatus').textContent = 'LOCAL';
    setLed('serverLed', true);
    log(`控制台就绪 · 关节速度 ${app.config.robot.joint_speed_deg_s}°/s`, 'success');
  } catch (error) {
    $('hostStatus').textContent = 'BRIDGE ERROR';
    $('modelError').textContent = error.message;
    $('modelError').classList.remove('hidden');
    log(`初始化失败：${error.message}`, 'error');
  }
}

window.addEventListener('beforeunload', () => cancelAnimationFrame(renderFrame));
boot();

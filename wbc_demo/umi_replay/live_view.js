const stageElement = document.getElementById('stage');
const phaseElement = document.getElementById('phase');
const errorElement = document.getElementById('error');
const actionElement = document.getElementById('action');
const startButton = document.getElementById('start');
const policyElement = document.getElementById('policy');
const buttons = [...document.querySelectorAll('[data-command]')];
const images = [...document.querySelectorAll('[data-view]')];
let connected = false;
let commandError = '';

function disconnected(message) {
  connected = false;
  stageElement.textContent = '离线：没有连接到运行中的仿真服务';
  errorElement.textContent = message;
  startButton.disabled = true;
  buttons.forEach(button => { button.disabled = true; });
  document.getElementById('connection').hidden = false;
}

async function sendCommand(value) {
  if (!connected) return;
  commandError = '';
  try {
    const response = await fetch('/api/control', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(value)
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || response.statusText);
    actionElement.textContent = '已发送 ' + value.command + '，等待后端处理';
  } catch (error) {
    commandError = error.message;
    errorElement.textContent = commandError;
  }
}

async function refreshStatus() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('状态接口返回 ' + response.status);
    const state = await response.json();
    connected = true;
    document.getElementById('connection').hidden = true;
    stageElement.textContent = state.stage;
    phaseElement.textContent = Number(state.source_time_s).toFixed(3) + ' s';
    startButton.disabled = !state.ready || state.busy;
    policyElement.disabled = state.busy;
    buttons.forEach(button => {
      const kind = button.dataset.command;
      button.disabled = kind === 'pause' ? state.stage !== 'replaying' :
        kind === 'resume' ? state.stage !== 'paused' : kind === 'stop' ? !state.busy : kind === 'shutdown' ? false : !state.ready;
    });
    document.getElementById('ready').textContent = state.ready ?
      '三路相机就绪 · 已记录 ' + state.record_frames + ' 帧' : '等待三路相机';
    errorElement.textContent = commandError || state.error || state.camera_warning || '';
    if (state.command_count) actionElement.textContent =
      '后端已处理 ' + state.command_count + ' 次操作 · ' + state.last_command;
  } catch (error) {
    disconnected('服务未启动或已关闭。图片/离线 HTML 不能控制仿真，请按下方命令启动。' + error.message);
  }
  window.setTimeout(refreshStatus, 300);
}

function refreshImage(image) {
  if (connected) {
    image.onload = image.onerror = () => window.setTimeout(() => refreshImage(image), 100);
    image.src = '/frame/' + image.dataset.view + '?t=' + Date.now();
  } else {
    window.setTimeout(() => refreshImage(image), 500);
  }
}

startButton.addEventListener('click', () => sendCommand({command: 'start', policy: policyElement.value}));
buttons.forEach(button => button.addEventListener('click', () => {
  const kind = button.dataset.command;
  if (kind === 'shutdown' && !window.confirm('结束本次回放并关闭本次启动的后端？')) return;
  sendCommand({command: kind});
}));

document.querySelectorAll('.orbit').forEach(image => {
  let previous = null;
  let lastSent = 0;
  image.draggable = false;
  image.addEventListener('dragstart', event => event.preventDefault());
  image.addEventListener('pointerdown', event => {
    if (!connected || event.button !== 0) return;
    event.preventDefault();
    previous = [event.clientX, event.clientY];
    image.setPointerCapture(event.pointerId);
  });
  image.addEventListener('pointermove', event => {
    if (!previous || performance.now() - lastSent < 40) return;
    event.preventDefault();
    const horizontal = Math.max(-1, Math.min(1, (event.clientX - previous[0]) * 0.008));
    const vertical = Math.max(-1, Math.min(1, (event.clientY - previous[1]) * 0.008));
    previous = [event.clientX, event.clientY];
    lastSent = performance.now();
    sendCommand({command: 'orbit', view: Number(image.dataset.orbit), horizontal, vertical});
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(kind =>
    image.addEventListener(kind, () => { previous = null; }));
  image.addEventListener('wheel', event => {
    event.preventDefault();
    sendCommand({command: 'zoom', view: Number(image.dataset.orbit),
      factor: Math.exp(Math.max(-1, Math.min(1, event.deltaY / 600)))});
  }, {passive: false});
});

if (location.protocol === 'file:') {
  disconnected('这是本地界面文件，不是在线服务。运行 bash wbc_demo/run.sh umi-web 后打开 http://127.0.0.1:8765。');
} else {
  refreshStatus();
  images.forEach(refreshImage);
}

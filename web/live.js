'use strict';
const $ = s => document.querySelector(s);
let latest = null, connected = false, paused = false, logs = [], busy = false, logFilter = 'all';
let cameraBusy = false, cameraURL = null, selectedFrame = 'color';
let placesKey = '', pendingConfirm = null;

const HISTORY_MAX = 120;
const history = {cpu: [], temp: [], mem: [], batt: [], nodes: [], dist: []};
const pushHistory = (key, value) => {
  if (!Number.isFinite(value)) return;
  const list = history[key];
  list.push(value);
  if (list.length > HISTORY_MAX) list.shift();
};

const viewTitles = {overview: '总览', status: '设备状态', maps: '地图配置'};

function goView(name) {
  document.querySelectorAll('.view').forEach(el => el.hidden = el.id !== name);
  document.querySelectorAll('.nav button').forEach(el => el.classList.toggle('active', el.dataset.page === name));
  $('#view-title').textContent = viewTitles[name] || '';
}
document.querySelectorAll('.nav button').forEach(b => b.addEventListener('click', () => goView(b.dataset.page)));

function age(at) { return at ? Math.max(0, Date.now() / 1000 - at) : Infinity; }
function fresh(sample, seconds = 8) { return connected && sample && age(sample.at) < seconds; }
function num(v, d = 1) { return Number.isFinite(v) ? Number(v.toFixed(d)).toString() : '—'; }
function stat(key, text) { document.querySelectorAll('[data-stat="' + key + '"]').forEach(el => el.textContent = text); }
function toast(text) {
  const el = $('#toast');
  el.textContent = text;
  el.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => el.hidden = true, 4200);
}

function ask(title, text) {
  return new Promise(resolve => {
    $('#confirm-title').textContent = title;
    $('#confirm-text').textContent = text;
    $('#confirm-dialog').showModal();
    pendingConfirm = resolve;
  });
}
$('#confirm-cancel').addEventListener('click', () => { $('#confirm-dialog').close(); pendingConfirm && pendingConfirm(false); pendingConfirm = null; });
$('#confirm-ok').addEventListener('click', () => { $('#confirm-dialog').close(); pendingConfirm && pendingConfirm(true); pendingConfirm = null; });

async function post(path, body) {
  const response = await fetch(path, {
    method: 'POST', cache: 'no-store',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({confirm: true, ...body}),
    signal: AbortSignal.timeout(10000)
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok || !data.ok) throw Error(data.message || ('HTTP ' + response.status));
  return data;
}

function batteryValue(samples) {
  const chassis = samples.chassis_battery;
  if (fresh(chassis, 12) && Number.isFinite(chassis.percent)) return {value: chassis.percent, sample: chassis, name: '底盘电池'};
  const b = samples.battery, p = samples.power;
  if (fresh(b, 12)) {
    const main = Boolean(b.installed & 2), small = Boolean(b.installed & 1);
    if (main && Number.isFinite(b.master_percent)) return {value: b.master_percent, sample: b, name: '大电池'};
    if (small && Number.isFinite(b.little_percent)) return {value: b.little_percent, sample: b, name: '小电池'};
  }
  if (fresh(p, 12) && p.voltage > 0 && Number.isFinite(p.percent)) return {value: p.percent, sample: p, name: '电源板'};
  return null;
}

function renderTask() {
  const task = latest?.task || {state: 'idle', route: []};
  const node = latest?.node || {};
  const routeEl = $('#task-route'), noteEl = $('#task-note');
  const stepsEl = $('#task-steps'), guideEl = $('#task-guide'), pill = $('#task-pill');

  const setPill = (text, tone) => {
    pill.className = 'stat-pill ' + tone;
    pill.replaceChildren();
    const dot = document.createElement('span');
    dot.className = 'dot';
    pill.append(dot, text);
  };
  const showGuide = () => {
    stepsEl.replaceChildren();
    if (guideEl.childElementCount) return;
    [
      ['启动节点', '语音链路未运行'],
      ['选择目的地', '地图点位 或 任意指令'],
      ['下发导航', '确认后开始执行']
    ].forEach(([title, note], index) => {
      const step = document.createElement('div');
      step.className = 'step';
      const no = document.createElement('span');
      no.className = 'step-no';
      no.textContent = String(index + 1);
      const body = document.createElement('div');
      body.className = 'step-body';
      const strong = document.createElement('b');
      strong.textContent = title;
      const hint = document.createElement('span');
      hint.className = 'step-note';
      hint.textContent = note;
      body.append(strong, hint);
      step.append(no, body);
      guideEl.append(step);
    });
  };
  const showRoute = (route, done) => {
    guideEl.replaceChildren();
    stepsEl.replaceChildren();
    (route || []).forEach(name => {
      const chip = document.createElement('span');
      chip.className = 'chip' + (done ? ' done' : '');
      chip.textContent = name;
      stepsEl.append(chip);
    });
  };

  $('#task-updated').textContent = task.at
    ? new Date(task.at * 1000).toLocaleTimeString('zh-CN', {hour12: false, timeZone: 'Asia/Shanghai'})
    : '—';
  $('#task-target').textContent = (task.route && task.route.length
    ? task.route[task.route.length - 1]
    : task.to) || '—';

  if (!connected) {
    setPill('连接中断', 'off');
    routeEl.firstChild.textContent = '—';
    noteEl.textContent = '正在自动重连监测服务';
    showGuide();
    return;
  }
  if (!node.running) {
    setPill('语音节点未运行', 'warn');
    routeEl.firstChild.textContent = '待命中';
    noteEl.textContent = '先在左侧启动节点，再选择目的地';
    showGuide();
    return;
  }
  if (task.state === 'running') {
    setPill('执行中', 'ok');
    routeEl.firstChild.textContent = task.route?.length > 1 ? task.route.join(' → ') : (task.to || '—');
    noteEl.textContent = task.from ? '从 ' + task.from + ' 出发' : '正在前往目的地';
    showRoute(task.route, false);
    return;
  }
  if (task.state === 'done') {
    setPill('已完成', 'ok');
    routeEl.firstChild.textContent = task.route?.length ? task.route.join(' → ') : (task.to || '—');
    noteEl.textContent = '可以继续下发新的目的地';
    showRoute(task.route, true);
    return;
  }
  if (task.state === 'failed') {
    setPill('未完成', 'off');
    routeEl.firstChild.textContent = task.route?.length ? task.route.join(' → ') : (task.to || '—');
    noteEl.textContent = '查看日志或重新下发';
    showRoute(task.route, false);
    return;
  }
  setPill('待命', 'ok');
  routeEl.firstChild.textContent = '待命中';
  noteEl.textContent = '当前没有执行任务';
  showGuide();
}

function renderNode() {
  const node = latest?.node || {};
  const button = $('#node-button');
  const label = button.querySelector('span');
  const use = button.querySelector('use');
  button.disabled = false;
  button.classList.remove('primary', 'danger');
  if (!connected) {
    $('#node-state').textContent = '连接中断';
    label.textContent = '启动节点';
    button.disabled = true;
    button.classList.add('primary');
  } else if (node.running && node.managed) {
    $('#node-state').textContent = '运行中';
    label.textContent = '停止节点';
    button.classList.add('danger');
    use.setAttribute('href', '#i-stop');
  } else if (node.running) {
    $('#node-state').textContent = '外部启动 · 运行中';
    label.textContent = '节点运行中';
    button.disabled = true;
    button.classList.add('primary');
    use.setAttribute('href', '#i-play');
  } else {
    $('#node-state').textContent = '未运行';
    label.textContent = '启动节点';
    button.classList.add('primary');
    use.setAttribute('href', '#i-play');
  }
}

function renderPlaces() {
  const places = latest?.places || [];
  const key = places.map(p => p.name).join('|');
  if (key !== placesKey) {
    placesKey = key;
    const select = $('#place-select');
    select.replaceChildren();
    places.forEach(p => {
      const option = document.createElement('option');
      option.value = p.name;
      option.textContent = p.name;
      select.append(option);
    });
    const list = $('#place-list');
    list.replaceChildren();
    places.forEach((p, i) => {
      const card = document.createElement('div');
      card.className = 'place-item';
      const strong = document.createElement('strong');
      strong.textContent = p.name;
      const poi = document.createElement('span');
      poi.textContent = '地图点位：' + p.poi;
      const order = document.createElement('span');
      order.textContent = '点位顺序：第 ' + (i + 1) + ' 站';
      card.append(strong, poi, order);
      list.append(card);
    });
    $('#map-note').textContent = places.length
      ? '共 ' + places.length + ' 个地点，默认点对点直达；配置来源 learn3_d/config/places.yaml。'
      : '未读到 places.yaml。';
  }
}

const PLAN = {scale: 22, x0: -2.5, y0: 9.9, w: 980, h: 372};
// 抽象平面图：一条主走廊，左上一个上分支、右下一个下分支，三段连通为一个整体
const PLAN_BRANCHES = [
  [4.5, 3.15, 3.0, 5.95],
  [29.0, -6.2, 2.2, 7.35]
];
const CORRIDOR = {x0: -1.2, x1: 39.4, y0: 1.15, y1: 3.15, yc: 2.15};

function px(x) { return (x - PLAN.x0) * PLAN.scale; }
function py(y) { return (PLAN.y0 - y) * PLAN.scale; }

function escapeXml(text) {
  return String(text).replace(/[&<>]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;'}[c]));
}

function stationPoints() {
  const places = latest?.places || [];
  const pois = latest?.pois || {};
  return places.map((place, index) => {
    const world = pois[place.poi] || {};
    const wx = Number(world.x), wy = Number(world.y);
    const fallbackX = 2.9 + index * 7.2;
    const x = Number.isFinite(wx) ? wx : fallbackX;
    return {
      name: place.name, poi: place.poi,
      wx: x, wy: Number.isFinite(wy) ? wy : CORRIDOR.yc,
      sx: px(x), sy: py(CORRIDOR.yc)
    };
  });
}

// 走廊是直线，用 x 投影算进度，避免定位 y 漂移把标记甩出走廊
function corridorFraction(points, x) {
  if (points.length < 2) return null;
  const first = points[0].wx, last = points[points.length - 1].wx;
  if (!Number.isFinite(first) || !Number.isFinite(last) || Math.abs(last - first) < 0.1) return null;
  return Math.max(0, Math.min(1, (x - first) / (last - first)));
}

function schematicMarkup(id, points, frac, task, opts = {}) {
  const firstX = points[0].wx, lastX = points[points.length - 1].wx;
  const pose = latest?.samples?.pose;
  const fresh = pose && connected && age(pose.at) < 5;
  const percent = frac == null ? null : Math.round(frac * 100);
  const segmentIndex = frac == null ? -1 : Math.min(points.length - 1, Math.floor(frac * (points.length - 1) + 1e-6));
  const nextIndex = Math.min(points.length - 1, segmentIndex + 1);
  const running = task.state === 'running';
  const progress = running && percent != null ? percent : 0;

  // 走廊与两个分支共用一条外轮廓，接缝处没有墙线
  const [ub, lb] = PLAN_BRANCHES;
  const fx = v => px(v).toFixed(1), fy = v => py(v).toFixed(1);
  const xa = ub[0], xb = ub[0] + ub[2], yt = ub[1] + ub[3];
  const xc = lb[0], xd = lb[0] + lb[2], yb = lb[1];
  const shape = `M${fx(CORRIDOR.x0)} ${fy(CORRIDOR.y0)} H${fx(xc)} V${fy(yb)} H${fx(xd)} V${fy(CORRIDOR.y0)} ` +
    `H${fx(CORRIDOR.x1)} V${fy(CORRIDOR.y1)} H${fx(xb)} V${fy(yt)} H${fx(xa)} V${fy(CORRIDOR.y1)} H${fx(CORRIDOR.x0)} Z`;
  const corridor = `<path d="${shape}" fill="#111b1e" stroke="#3c4a4e" stroke-width="1" stroke-linejoin="round"/>`;
  const rooms = '';
  const blocks = '';

  const routeY = py(CORRIDOR.yc);
  const route = `M${px(firstX).toFixed(1)} ${routeY.toFixed(1)} L${px(lastX).toFixed(1)} ${routeY.toFixed(1)}`;

  const stations = points.map((p, i) => {
    const passed = frac != null && i < segmentIndex;
    const next = frac != null && i === nextIndex && running;
    const fill = passed ? '#2f7ae0' : '#3f8cff';
    const opacity = frac == null ? .6 : (i <= nextIndex ? 1 : .45);
    const label = escapeXml(p.name);
    const w = label.length * 12 + 14;
    const chipY = p.sy + 22;
    const ring = next ? `<circle cx="${p.sx}" cy="${p.sy}" r="11" fill="none" stroke="#bfe3ff" stroke-opacity=".6"/>` : '';
    return `<g opacity="${opacity}">` +
      `<rect x="${(p.sx - w / 2).toFixed(1)}" y="${chipY.toFixed(1)}" width="${w}" height="20" rx="5" fill="#0f191c" stroke="${passed || next ? '#3f8cff88' : '#ffffff16'}"/>` +
      `<text x="${p.sx.toFixed(1)}" y="${(chipY + 14).toFixed(1)}" text-anchor="middle" font-size="11" fill="#d5e6e4">${label}</text>` +
      `<circle cx="${p.sx.toFixed(1)}" cy="${p.sy.toFixed(1)}" r="5.5" fill="${fill}" stroke="#ffffff" stroke-width="2"/></g>` + ring;
  }).join('');

  let robot = '';
  if (frac != null && pose) {
    const rx = px(firstX + frac * (lastX - firstX));
    const opacity = fresh ? 1 : .4;
    robot = `<g transform="translate(${rx.toFixed(1)} ${routeY.toFixed(1)})" opacity="${opacity}">` +
      `<rect x="-15" y="-8" width="30" height="16" rx="8" fill="#4ee08e" filter="url(#glow-${id})"/>` +
      `<rect x="-8" y="-3.5" width="16" height="7" rx="3.5" fill="#0e2318" fill-opacity=".55"/></g>`;
  }

  let tip = '';
  if (frac != null && opts.tip !== false) {
    const rx = px(firstX + frac * (lastX - firstX));
    const nearest = points[Math.min(points.length - 1, Math.max(0, Math.round(frac * (points.length - 1))))].name;
    const target = running ? (task.to || points[nextIndex]?.name || '—') : nearest;
    const title = running ? '任务 ' + progress + '%' : '待命';
    const line = running ? '前往 ' + target : '位于 ' + target + ' 附近';
    const tx = Math.max(8, Math.min(PLAN.w - 176, rx - 84));
    const ty = Math.max(8, Math.min(PLAN.h - 74, routeY - 88));
    tip = `<g transform="translate(${tx.toFixed(1)} ${ty.toFixed(1)})">` +
      `<rect width="168" height="62" rx="8" fill="#141d20" stroke="#ffffff1f"/>` +
      `<circle cx="13" cy="17" r="3.2" fill="#4ee08e"/>` +
      `<text x="23" y="21" font-size="12" fill="#e6f3ec">小静</text>` +
      `<text x="156" y="21" font-size="11" text-anchor="end" fill="#9fb6b0">${title}</text>` +
      `<rect x="12" y="30" width="144" height="4" rx="2" fill="#ffffff14"/>` +
      `<rect x="12" y="30" width="${(144 * progress / 100).toFixed(1)}" height="4" rx="2" fill="#3f8cff"/>` +
      `<text x="12" y="50" font-size="11" fill="#9fb6b0">${escapeXml(line)}</text></g>`;
  }

  return `<svg viewBox="0 0 ${PLAN.w} ${PLAN.h}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="机器人位置与任务进度">` +
    `<defs><pattern id="grid-${id}" width="44" height="44" patternUnits="userSpaceOnUse">` +
    `<path d="M44 0H0V44" fill="none" stroke="#ffffff05"/></pattern>` +
    `<pattern id="hatch-${id}" width="10" height="10" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">` +
    `<line x1="0" y1="0" x2="0" y2="10" stroke="#ffffff09" stroke-width="1"/></pattern>` +
    `<filter id="glow-${id}" x="-80%" y="-80%" width="260%" height="260%">` +
    `<feGaussianBlur stdDeviation="4" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter></defs>` +
    `<rect width="${PLAN.w}" height="${PLAN.h}" fill="url(#grid-${id})"/>` +
    corridor + rooms + blocks +
    `<path d="${route}" fill="none" stroke="#3f8cff" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>` +
    stations + robot + tip + '</svg>';
}

// 缩略图只显示当前位置附近一圈，点击后打开全貌
function zoomMini(box, points, frac) {
  const svg = box.querySelector('svg');
  if (!svg) return;
  const w = 340, h = 216;
  const last = points[points.length - 1];
  const rx = points[0].sx + frac * (last.sx - points[0].sx);
  const ry = points[0].sy;
  const x0 = Math.max(0, Math.min(PLAN.w - w, rx - w / 2));
  const y0 = Math.max(0, Math.min(PLAN.h - h, ry - h / 2));
  svg.setAttribute('viewBox', `${x0.toFixed(1)} ${y0.toFixed(1)} ${w} ${h}`);
  svg.setAttribute('preserveAspectRatio', 'xMidYMid slice');
}

function renderMap() {
  const points = stationPoints();
  const frac = (() => {
    const pose = latest?.samples?.pose;
    if (!pose || !connected || !Number.isFinite(pose.x)) return null;
    return corridorFraction(points, pose.x);
  })();
  const task = latest?.task || {state: 'idle'};
  ['schematic-map', 'schematic-home', 'schematic-full'].forEach(id => {
    const box = document.getElementById(id);
    if (!box) return;
    const mini = id === 'schematic-home';
    box.innerHTML = points.length
      ? schematicMarkup(id, points, frac, task, {tip: !mini})
      : '<p class="empty">未读到 places.yaml 或底盘点位坐标</p>';
    if (mini && points.length && frac != null) zoomMini(box, points, frac);
  });
  const pose = latest?.samples?.pose;
  const noteEl = document.getElementById('pose-note');
  document.getElementById('map-note').textContent = points.length
    ? '抽象平面图只保留走廊、分支与障碍块；位置按 X 方向投影，仅表示大致进度。'
    : '未读到点位配置。';
  if (!noteEl) return;
  if (!pose || !connected) {
    document.getElementById('pose-readout').textContent = '';
    noteEl.textContent = connected ? '等待底盘位姿数据' : '监测连接中断';
    return;
  }
  const degrees = (pose.yaw * 180 / Math.PI).toFixed(0);
  document.getElementById('pose-readout').textContent = 'x ' + num(pose.x, 2) + ' · y ' + num(pose.y, 2) + ' · ' + degrees + '°';
  noteEl.textContent = '位姿 ' + num(age(pose.at), 1) + ' 秒前 · 置信度 ' + (Number.isFinite(pose.confidence) ? num(pose.confidence, 2) : '—') + (frac == null ? ' · 无点位坐标' : '');
}

function sparkline(values) {
  if (!values.length) return '';
  const w = 260, h = 30, pad = 3;
  const max = Math.max(...values), min = Math.min(...values);
  const span = max - min || Math.max(1, Math.abs(max) * 0.1);
  const step = values.length > 1 ? w / (values.length - 1) : w;
  const pts = values.map((v, i) => {
    const x = i * step;
    const y = h - pad - ((v - min) / span) * (h - pad * 2 - 2);
    return [x, y];
  });
  const line = pts.map((p, i) => (i ? 'L' : 'M') + p[0].toFixed(1) + ' ' + p[1].toFixed(1)).join(' ');
  const area = line + ' L' + w + ' ' + h + ' L0 ' + h + ' Z';
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('class', 'spark-svg');
  svg.setAttribute('viewBox', '0 0 ' + w + ' ' + h);
  svg.setAttribute('preserveAspectRatio', 'none');
  svg.setAttribute('aria-hidden', 'true');
  const fill = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  fill.setAttribute('d', area);
  fill.setAttribute('fill', '#4fd1a5');
  fill.setAttribute('opacity', '.16');
  const stroke = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  stroke.setAttribute('d', line);
  stroke.setAttribute('fill', 'none');
  stroke.setAttribute('stroke', '#6fdcae');
  stroke.setAttribute('stroke-width', '1.6');
  stroke.setAttribute('stroke-linejoin', 'round');
  stroke.setAttribute('stroke-linecap', 'round');
  stroke.setAttribute('vector-effect', 'non-scaling-stroke');
  svg.append(fill, stroke);
  return svg;
}

function bigChart(values) {
  const w = 320, h = 110, top = 8, bottom = 6;
  const max = Math.max(...values), min = Math.min(...values);
  const span = max - min || Math.max(1, Math.abs(max) * 0.1);
  const lo = min - span * 0.3, hi = max + span * 0.3;
  const step = values.length > 1 ? w / (values.length - 1) : w;
  const y = v => h - bottom - ((v - lo) / (hi - lo)) * (h - top - bottom);
  const pts = values.map((v, i) => [i * step, y(v)]);
  const line = pts.map((p, i) => (i ? 'L' : 'M') + p[0].toFixed(1) + ' ' + p[1].toFixed(1)).join(' ');
  const area = line + ' L' + w + ' ' + (h - bottom) + ' L0 ' + (h - bottom) + ' Z';
  const ns = 'http://www.w3.org/2000/svg';
  const svg = document.createElementNS(ns, 'svg');
  svg.setAttribute('viewBox', '0 0 ' + w + ' ' + h);
  svg.setAttribute('preserveAspectRatio', 'none');
  svg.setAttribute('aria-hidden', 'true');
  for (let i = 0; i <= 3; i++) {
    const gy = top + i * (h - top - bottom) / 3;
    const grid = document.createElementNS(ns, 'line');
    grid.setAttribute('x1', '0');
    grid.setAttribute('y1', gy.toFixed(1));
    grid.setAttribute('x2', String(w));
    grid.setAttribute('y2', gy.toFixed(1));
    grid.setAttribute('stroke', '#ffffff12');
    grid.setAttribute('stroke-dasharray', '4 5');
    grid.setAttribute('vector-effect', 'non-scaling-stroke');
    svg.append(grid);
  }
  const fill = document.createElementNS(ns, 'path');
  fill.setAttribute('d', area);
  fill.setAttribute('fill', '#4fd1a5');
  fill.setAttribute('opacity', '.12');
  const stroke = document.createElementNS(ns, 'path');
  stroke.setAttribute('d', line);
  stroke.setAttribute('fill', 'none');
  stroke.setAttribute('stroke', '#6fdcae');
  stroke.setAttribute('stroke-width', '1.8');
  stroke.setAttribute('stroke-linejoin', 'round');
  stroke.setAttribute('stroke-linecap', 'round');
  stroke.setAttribute('vector-effect', 'non-scaling-stroke');
  svg.append(fill, stroke);
  return svg;
}

function renderCharts() {
  const rows = [
    ['#chart-cpu', history.cpu],
    ['#chart-mem', history.mem],
    ['#chart-node', history.nodes]
  ];
  rows.forEach(([selector, values]) => {
    const box = $(selector);
    if (!box) return;
    box.replaceChildren();
    if (values.length < 2) {
      const p = document.createElement('p');
      p.className = 'empty';
      p.textContent = '积累数据中…';
      box.append(p);
      return;
    }
    box.append(bigChart(values));
  });
  const h = connected && fresh(latest?.host, 8) ? latest.host : null;
  $('#chart-cpu-note').textContent = h ? 'CPU ' + num(h.cpu_percent) + '%' : 'CPU 负载';
  $('#chart-mem-note').textContent = h
    ? '内存 ' + num(h.memory_used_gb) + ' / ' + num(h.memory_total_gb) + ' GB'
    : '主机内存';
  const ros = latest?.ros;
  const graph = connected && ros?.available && age(ros.graph_at) < 12;
  $('#chart-node-note').textContent = graph ? 'ROS ' + ros.nodes.length + ' 个节点' : 'ROS 节点';
}

function renderService() {
  const ros = latest?.ros;
  const graph = connected && ros?.available && age(ros.graph_at) < 12;
  const nodes = graph ? ros.nodes : [], services = graph ? ros.services : [];
  const voice = nodes.some(n => n.endsWith('/tour_chat'));
  const tts = services.includes('/intelligent_interaction/tts/play');
  const chassis = nodes.some(n => n.includes('knewbots_chassis_control'));
  const mic = fresh(latest?.samples?.microphone, 4);
  const items = [
    ['带路语音节点', voice ? '运行中' : '未运行', voice, voice ? 'tour_chat' : '等待启动节点'],
    ['语音播报服务', tts ? '可用' : '未发现', tts, '/intelligent_interaction/tts/play'],
    ['麦克风采集', mic ? '采集中' : '静默', mic, mic ? '最近 4 秒内收到音频帧' : '未收到音频帧'],
    ['移动底盘', chassis ? '已发现' : '未发现', chassis, 'knewbots_chassis_control']
  ];
  const box = $('#service-chips');
  box.replaceChildren();
  items.forEach(([label, value, ok, note]) => {
    const item = document.createElement('div');
    item.className = 'chip-card';
    const cap = document.createElement('div');
    cap.className = 'cap';
    const name = document.createElement('span');
    name.textContent = label;
    const pill = document.createElement('span');
    pill.className = 'stat-pill ' + (ok ? 'ok' : 'warn');
    const dot = document.createElement('span');
    dot.className = 'dot';
    pill.append(dot, value);
    cap.append(name, pill);
    const strong = document.createElement('b');
    strong.textContent = note;
    item.append(cap, strong);
    box.append(item);
  });
}

function card({label, pill, tone, value, unit, sub, bar, barTone, text, spark}) {
  const box = document.createElement('div');
  box.className = 'stat-card';

  const cap = document.createElement('div');
  cap.className = 'cap';
  const name = document.createElement('span');
  name.textContent = label;
  cap.append(name);
  if (pill) {
    const tag = document.createElement('span');
    tag.className = 'stat-pill ' + (tone || 'ok');
    const dot = document.createElement('span');
    dot.className = 'dot';
    tag.append(dot, pill);
    cap.append(tag);
  }

  const figure = document.createElement('div');
  figure.className = 'stat-value' + (text ? ' text' : '');
  figure.textContent = value;
  if (unit) {
    const small = document.createElement('small');
    small.textContent = unit;
    figure.append(small);
  }
  box.append(cap, figure);

  if (sub) {
    const note = document.createElement('div');
    note.className = 'stat-sub';
    note.textContent = sub;
    box.append(note);
  }

  if (typeof bar === 'number' && Number.isFinite(bar)) {
    const track = document.createElement('div');
    track.className = 'stat-bar';
    const fillBar = document.createElement('i');
    fillBar.style.width = Math.min(100, Math.max(2, bar * 100)).toFixed(0) + '%';
    if (barTone) fillBar.className = barTone;
    track.append(fillBar);
    box.append(track);
  }

  if (Array.isArray(spark) && spark.length > 1) {
    const wrap = document.createElement('div');
    wrap.className = 'spark';
    wrap.append(sparkline(spark));
    box.append(wrap);
  }
  return box;
}

function mount(selector, cards) {
  const el = $(selector);
  el.replaceChildren();
  cards.forEach(item => el.append(item));
}

function renderStatusLists() {
  const s = latest?.samples || {}, ros = latest?.ros;
  const graph = connected && ros?.available && age(ros.graph_at) < 12;
  const nodes = graph ? ros.nodes : [], services = graph ? ros.services : [];
  const h = connected && fresh(latest?.host, 8) ? latest.host : null;
  const b = batteryValue(s);
  const color = connected && latest?.cameras?.color && age(latest.cameras.color.at) < 3;
  const depth = fresh(s.depth, 3);
  const mic = fresh(s.microphone, 4);
  const tts = services.includes('/intelligent_interaction/tts/play');
  const voice = nodes.some(n => n.endsWith('/tour_chat'));
  const chassis = nodes.some(n => n.includes('knewbots_chassis_control'));

  pushHistory('cpu', h ? h.cpu_percent : null);
  pushHistory('temp', h && h.cpu_temp_c !== null ? h.cpu_temp_c : null);
  pushHistory('mem', h ? h.memory_used_gb : null);
  pushHistory('batt', b ? b.value : null);
  pushHistory('nodes', graph ? nodes.length : null);
  pushHistory('dist', depth ? s.depth.distance_m : null);

  const charge = b?.sample.is_charging === true ? '充电中'
    : b?.sample.is_charging === false ? '未充电' : '未知';
  const level = (v, warn, hot) => (v == null ? 'off' : v >= hot ? 'off' : v >= warn ? 'warn' : 'ok');
  const levelText = (v, warn, hot, labels) => (v == null ? labels.none
    : v >= hot ? labels.hot : v >= warn ? labels.warn : labels.ok);

  const hostCards = [
    card({
      label: 'CPU 负载',
      pill: h ? levelText(h.cpu_percent, 65, 85, {ok: '正常', warn: '较忙', hot: '偏高', none: '无数据'}) : '无数据',
      tone: h ? level(h.cpu_percent, 65, 85) : 'off',
      value: h ? num(h.cpu_percent) : '—', unit: '%',
      bar: h ? h.cpu_percent / 100 : null,
      barTone: h ? (h.cpu_percent >= 85 ? 'hot' : h.cpu_percent >= 65 ? 'warn' : '') : '',
      spark: history.cpu
    }),
    card({
      label: 'CPU 温度',
      pill: h && h.cpu_temp_c !== null ? levelText(h.cpu_temp_c, 70, 82, {ok: '正常', warn: '偏高', hot: '过热', none: '无数据'}) : '无数据',
      tone: h && h.cpu_temp_c !== null ? level(h.cpu_temp_c, 70, 82) : 'off',
      value: h && h.cpu_temp_c !== null ? num(h.cpu_temp_c) : '—', unit: '°C',
      bar: h && h.cpu_temp_c !== null ? (h.cpu_temp_c - 20) / 60 : null,
      barTone: h && h.cpu_temp_c >= 82 ? 'hot' : h && h.cpu_temp_c >= 70 ? 'warn' : '',
      spark: history.temp
    }),
    card({
      label: '主机内存',
      pill: h ? levelText(h.memory_used_gb / h.memory_total_gb, 0.75, 0.9, {ok: '充裕', warn: '偏紧', hot: '紧张', none: '无数据'}) : '无数据',
      tone: h ? level(h.memory_used_gb / h.memory_total_gb, 0.75, 0.9) : 'off',
      value: h ? num(h.memory_used_gb) : '—',
      unit: h ? '/ ' + num(h.memory_total_gb) + ' GB' : '',
      bar: h ? h.memory_used_gb / h.memory_total_gb : null,
      barTone: h && h.memory_used_gb / h.memory_total_gb >= 0.9 ? 'hot' : h && h.memory_used_gb / h.memory_total_gb >= 0.75 ? 'warn' : '',
      spark: history.mem
    }),
    card({
      label: '电池电量',
      pill: b ? charge : '无数据',
      tone: b ? (b.value < 20 ? 'off' : b.value < 40 ? 'warn' : 'ok') : 'off',
      value: b ? num(b.value, 0) : '—', unit: b ? '%' : '',
      sub: b ? b.name + ' · ' + charge : '等待电量数据',
      bar: b ? b.value / 100 : null,
      barTone: b ? (b.value < 20 ? 'hot' : b.value < 40 ? 'warn' : '') : '',
      spark: history.batt
    }),
    card({
      label: '开机时长',
      pill: h ? '运行中' : '无数据', tone: h ? 'ok' : 'off',
      value: h ? String(Math.floor(h.uptime_s / 3600)) : '—',
      unit: h ? '小时 ' + Math.floor(h.uptime_s % 3600 / 60) + ' 分' : '',
      sub: h ? '主机连续运行' : '等待主机数据'
    })
  ];

  const hwCards = [
    card({
      label: 'ROS 图',
      pill: graph ? '已连接' : '未连接', tone: graph ? 'ok' : 'off',
      value: graph ? String(nodes.length) : '—', unit: graph ? '节点' : '',
      sub: graph ? ros.topics.length + ' 个话题 · 只读订阅' : 'ROS 监测不可用',
      spark: history.nodes
    }),
    card({
      label: '头部彩色相机',
      pill: color ? 'LIVE' : '无信号', tone: color ? 'ok' : 'off',
      value: color ? num(age(latest.cameras.color.at)) : '—', unit: color ? '秒前' : '',
      sub: color ? '图像持续更新' : '等待相机画面'
    }),
    card({
      label: '深度图 / 前向距离',
      pill: depth ? '实时' : '无数据', tone: depth ? 'ok' : 'off',
      value: depth ? num(s.depth.distance_m, 2) : '—', unit: depth ? 'm' : '',
      sub: depth ? '头部深度图中央区域' : '等待深度数据',
      spark: history.dist
    }),
    card({
      label: '麦克风',
      pill: mic ? '采集中' : '静默', tone: mic ? 'ok' : 'warn', text: true,
      value: mic ? '音频更新中' : '无新音频',
      sub: mic ? '最近 4 秒内收到音频帧' : '未收到音频帧'
    }),
    card({
      label: '移动底盘',
      pill: chassis ? '已发现' : '未发现', tone: chassis ? 'ok' : 'warn', text: true,
      value: chassis ? '节点已发现' : '未发现节点',
      sub: 'knewbots_chassis_control'
    }),
    card({
      label: '语音播报',
      pill: tts ? '可用' : '未发现', tone: tts ? 'ok' : 'warn', text: true,
      value: tts ? 'TTS 服务可用' : 'TTS 服务未发现',
      sub: '/intelligent_interaction/tts/play'
    }),
    card({
      label: '带路语音节点',
      pill: voice ? '运行中' : '未运行', tone: voice ? 'ok' : 'warn', text: true,
      value: voice ? '运行中' : '未运行',
      sub: 'tour_chat'
    })
  ];

  mount('#host-cards', hostCards);
  mount('#hw-cards', hwCards);
  $('#ros-summary').textContent = graph ? nodes.length + ' 个节点 / ' + ros.topics.length + ' 个话题' : 'ROS 未连接';
  renderCharts();
  renderService();
}

function update() {
  const s = latest?.samples || {}, ros = latest?.ros;
  const graph = connected && ros?.available && age(ros.graph_at) < 12;
  const nodes = graph ? ros.nodes : [], services = graph ? ros.services : [];
  const h = connected && fresh(latest?.host, 8) ? latest.host : null;

  stat('cpu', h ? num(h.cpu_percent) + ' %' : '—');
  stat('temp', h && h.cpu_temp_c !== null ? num(h.cpu_temp_c) + ' °C' : '—');
  stat('memory', h ? num(h.memory_used_gb) + ' / ' + num(h.memory_total_gb) + ' GB' : '—');
  stat('uptime', h ? Math.floor(h.uptime_s / 3600) + ' 小时 ' + Math.floor(h.uptime_s % 3600 / 60) + ' 分' : '—');

  const b = batteryValue(s);
  stat('battery', b && b.value >= 0 && b.value <= 100 ? num(b.value, 0) : '—');
  $('#battery-note').textContent = b
    ? b.name + ' · ' + (b.sample.is_charging === true ? '正在充电' : b.sample.is_charging === false ? '未充电' : '充电状态未知')
    : '未收到有效电量';
  $('#battery-ring').style.background = b && b.value >= 0 && b.value <= 100
    ? 'conic-gradient(#badbc9 0 ' + b.value + '%,#3a4942 ' + b.value + '%)' : '#3a4942';

  const voice = nodes.some(n => n.endsWith('/tour_chat'));
  const tts = services.includes('/intelligent_interaction/tts/play');
  const mic = fresh(s.microphone, 4);
  $('#voice-link').textContent = graph ? (voice ? '节点运行中' : '未运行') : 'ROS 未连接';
  $('#tts-link').textContent = tts ? '服务可用' : '未发现';
  $('#mic-link').textContent = mic ? '有音频数据' : '无新音频';

  const connection = $('#connection');
  connection.innerHTML = '';
  const dot = document.createElement('span');
  dot.className = 'dot' + (connected ? '' : ' off');
  connection.append(dot, connected ? '实时监测 · 已连接' : '监测连接中断 · 正在重连');

  renderTask();
  renderNode();
  renderPlaces();
  renderMap();
  renderStatusLists();

  const nodeRunning = Boolean(latest?.node?.running);
  $('#send-button').disabled = !connected || !nodeRunning || !(latest?.places || []).length;
  $('#command-button').disabled = !connected || !nodeRunning;
  $('#command-hint').textContent = !connected
    ? '监测连接中断，暂不可下发'
    : !nodeRunning
      ? '先启动语音节点，再下发导航'
      : '确认后立即下发：去「当前选择的地点」';

  const frame = connected && latest?.cameras?.[selectedFrame];
  if (!frame || age(frame.at) >= 3) {
    $('#camera-image').hidden = true;
    $('#camera-empty').hidden = false;
  }
}

function logHeadline(text) {
  const t = String(text || '');
  return t.length > 34 ? t.slice(0, 34) + '…' : t;
}

function renderLogs(force) {
  if (paused && !force) return;
  const panel = $('#event-stream');
  panel.replaceChildren();
  const rows = logs
    .filter(e => logFilter === 'all' || String(e.level).toLowerCase() === logFilter)
    .slice(-60);
  if (!rows.length) {
    const p = document.createElement('div');
    p.className = 'empty';
    p.textContent = logFilter === 'all' ? '暂无日志' : '没有' + (logFilter === 'warn' ? '告警' : '错误') + '记录';
    panel.append(p);
    return;
  }
  rows.forEach(e => {
    const level = String(e.level || 'INFO').toLowerCase();
    const stamp = new Date(e.at * 1000).toLocaleTimeString('zh-CN', {hour12: false, timeZone: 'Asia/Shanghai'});

    const row = document.createElement('div');
    row.className = 'event-row ' + level;

    const time = document.createElement('span');
    time.className = 'event-time';
    time.textContent = stamp;

    const rail = document.createElement('span');
    rail.className = 'event-rail';
    const dot = document.createElement('span');
    dot.className = 'event-dot';
    rail.append(dot);

    const body = document.createElement('div');
    body.className = 'event-body';
    const line = document.createElement('div');
    line.className = 'event-line';
    line.textContent = level === 'info' ? e.text : logHeadline(e.text);
    const src = document.createElement('span');
    src.className = 'event-src';
    src.textContent = ' - ' + e.source;
    line.append(src);
    body.append(line);

    if (level === 'warn' || level === 'error') {
      const card = document.createElement('div');
      card.className = 'event-card ' + level;
      const mark = document.createElement('span');
      mark.className = 'mark';
      mark.textContent = level === 'error' ? '×' : '⚠';
      const msg = document.createElement('span');
      msg.className = 'msg';
      msg.textContent = e.text;
      const act = document.createElement('button');
      act.type = 'button';
      act.className = 'act';
      act.textContent = '查看详情';
      const meta = document.createElement('div');
      meta.className = 'event-meta';
      meta.textContent = `${stamp} · ${e.level} · ${e.source}`;
      act.addEventListener('click', () => meta.classList.toggle('open'));
      card.append(mark, msg, act);
      body.append(card, meta);
    }

    row.append(time, rail, body);
    panel.append(row);
  });
  panel.scrollTop = panel.scrollHeight;
}

async function getJSON(path) {
  const response = await fetch(path, {cache: 'no-store', signal: AbortSignal.timeout(3500)});
  if (!response.ok) throw Error('HTTP ' + response.status);
  return response.json();
}

async function poll() {
  if (busy) return;
  busy = true;
  try {
    latest = await getJSON('/api/status');
    connected = true;
    logs = (await getJSON('/api/logs')).logs;
    renderLogs();
  } catch {
    connected = false;
  } finally {
    update();
    busy = false;
    setTimeout(poll, 2000);
  }
}

async function cameraPoll() {
  if (cameraBusy) return;
  cameraBusy = true;
  try {
    const kind = selectedFrame;
    const frame = latest?.cameras?.[kind];
    if (!document.hidden && connected && frame && age(frame.at) < 3 && !$('#overview').hidden) {
      const response = await fetch('/api/frame?kind=' + kind, {cache: 'no-store', signal: AbortSignal.timeout(2000)});
      if (!response.ok) throw Error('frame unavailable');
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      const image = $('#camera-image');
      const previous = cameraURL;
      cameraURL = url;
      image.src = url;
      image.hidden = false;
      $('#camera-empty').hidden = true;
      $('#camera-label').textContent = (kind === 'color' ? '彩色画面' : '深度画面') + ' · ' + frame.width + ' × ' + frame.height;
      if (previous) URL.revokeObjectURL(previous);
    }
  } catch {
    $('#camera-image').hidden = true;
    $('#camera-empty').hidden = false;
  } finally {
    cameraBusy = false;
    setTimeout(cameraPoll, 350);
  }
}

$('#camera-select').addEventListener('change', () => {
  selectedFrame = $('#camera-select').value;
  $('#camera-image').hidden = true;
  $('#camera-empty').hidden = false;
});

$('#node-button').addEventListener('click', async () => {
  const node = latest?.node || {};
  if (node.running && node.managed) {
    if (!await ask('停止节点', '停止后会中断当前语音对话和导航任务，确认停止导览语音节点？')) return;
    try { toast((await post('/api/node/stop')).message); } catch (e) { toast('停止失败：' + e.message); }
    return;
  }
  if (!await ask('启动节点', '将启动导览语音节点（不会自动移动）。确认启动？')) return;
  try { toast((await post('/api/node/start')).message); } catch (e) { toast('启动失败：' + e.message); }
});

$('#send-button').addEventListener('click', async () => {
  const place = $('#place-select').value;
  if (!place) return;
  if (!await ask('下发导航', '让机器人从当前位置前往「' + place + '」？确认后立即下发导航。')) return;
  try { toast((await post('/api/navigate', {place})).message); } catch (e) { toast('下发失败：' + e.message); }
});

async function sendCommand() {
  const input = $('#command-input');
  const text = input.value.trim();
  if (!text) return;
  if (!await ask('下发指令', '向语音节点下发「' + text + '」？')) return;
  try {
    toast((await post('/api/command', {text})).message);
    input.value = '';
  } catch (e) { toast('下发失败：' + e.message); }
}

$('#command-button').addEventListener('click', sendCommand);
$('#command-input').addEventListener('keydown', e => {
  if (e.key === 'Enter') sendCommand();
});

$('#pause-log').addEventListener('click', () => {
  paused = !paused;
  $('#pause-log').textContent = paused ? '继续' : '暂停';
  if (!paused) renderLogs(true);
});

document.querySelectorAll('.seg [data-filter]').forEach(button => {
  button.addEventListener('click', () => {
    logFilter = button.dataset.filter;
    document.querySelectorAll('.seg [data-filter]').forEach(b => b.classList.toggle('active', b === button));
    renderLogs(true);
  });
});

$('#map-mini').addEventListener('click', () => {
  renderMap();
  const readout = $('#pose-readout');
  $('#map-dialog-pose').textContent = readout?.textContent || '当前位置';
  $('#map-dialog').showModal();
});

$('#map-dialog-close').addEventListener('click', () => $('#map-dialog').close());

$('#map-dialog').addEventListener('click', event => {
  if (event.target === $('#map-dialog')) $('#map-dialog').close();
});

function clock() {
  const now = new Date();
  $('#clock').textContent = now.toLocaleTimeString('zh-CN', {hour: '2-digit', minute: '2-digit', hour12: false, timeZone: 'Asia/Shanghai'});
  $('#date').textContent = now.toLocaleDateString('zh-CN', {year: 'numeric', month: 'long', day: 'numeric', weekday: 'long', timeZone: 'Asia/Shanghai'});
}
clock();
setInterval(clock, 10000);
poll();
cameraPoll();

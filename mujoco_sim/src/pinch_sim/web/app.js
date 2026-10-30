// pinch_sim browser monitor: draws what pinch_sim/webui.py says (GET /events) around the camera image
// (GET /stream) and sends keys, mouse on the image and button actions back (POST /api/input).
'use strict';

const token = (() => {               // from the URL the app opened; kept for this tab so a reload still works
  const t = new URLSearchParams(location.search).get('t');
  try { if (t) sessionStorage.setItem('pinch_t', t); return t || sessionStorage.getItem('pinch_t') || ''; }
  catch (e) { return t || ''; }
})();
const $ = (id) => document.getElementById(id);
const A = '/static/assets/';
const STEP_ICONS = ['route.svg', 'pen-tool.svg', 'check-square.svg', 'move-right.svg'];
const CHIP_ICONS = { loading: 'loading.svg', check: 'check.svg', pause: 'pause.svg', warning: 'warning-triangle.svg' };
const KEYS = 'NSVYHXUROEGQ';

let S = null;                       // the latest state from the app
let graphOpen = false;
let firstChat = true;
let dismissedDialog = 0;
let localDialog = null;             // a question asked by this page itself ("검증 후 이동할까요?")
const localToasts = [];
let localId = 0;

// ------------------------------------------------------------------ talking to the app
function send(msg) {
  fetch(`/api/input?t=${encodeURIComponent(token)}`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Token': token },
    body: JSON.stringify(msg), keepalive: true,
  }).catch(() => {});
}
const key = (k) => send({ type: 'key', key: k });
const act = (name, extra = {}) => send({ type: 'action', name, ...extra });

function connect() {
  const es = new EventSource(`/events?t=${encodeURIComponent(token)}`);
  es.onmessage = (e) => { S = JSON.parse(e.data); render(); };
  es.onerror = () => { S = null; renderBadges(); };
}

// ------------------------------------------------------------------ camera image + mouse edits on it
const video = $('video');
function startVideo() { video.src = `/stream?t=${encodeURIComponent(token)}&r=${Date.now()}`; }
video.addEventListener('error', () => { $('video-wait').classList.remove('hidden'); setTimeout(startVideo, 1000); });
setInterval(() => { $('video-wait').classList.toggle('hidden', video.naturalWidth > 0); }, 500);
const backdrop = $('backdrop').getContext('2d');
setInterval(() => {                 // blurred cover copy of the frame for the letterbox bands (cheap: 192 x 108)
  const nw = video.naturalWidth, nh = video.naturalHeight;
  if (!nw) return;
  const s = Math.max(192 / nw, 108 / nh);
  backdrop.filter = 'blur(6px)';
  try { backdrop.drawImage(video, (192 - nw * s) / 2, (108 - nh * s) / 2, nw * s, nh * s); } catch (e) { /* not decoded yet */ }
}, 400);

function imageUV(e) {               // pointer -> fraction of the frame (the image is letterboxed: object-fit contain)
  const r = video.getBoundingClientRect();
  const nw = video.naturalWidth || (S ? S.frame[0] : 4), nh = video.naturalHeight || (S ? S.frame[1] : 3);
  const s = Math.min(r.width / nw, r.height / nh), w = nw * s, h = nh * s;
  return { u: (e.clientX - r.left - (r.width - w) / 2) / w, v: (e.clientY - r.top - (r.height - h) / 2) / h };
}
const clampUV = (p) => ({ u: Math.min(1, Math.max(0, p.u)), v: Math.min(1, Math.max(0, p.v)) });
let dragging = false, pendingMove = null;
video.addEventListener('pointerdown', (e) => {
  if (e.button !== 0) return;
  const p = imageUV(e);
  if (p.u < 0 || p.u > 1 || p.v < 0 || p.v > 1) return;
  dragging = true;
  video.setPointerCapture(e.pointerId);
  send({ type: 'mouse', kind: 'press', ...p });
});
video.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  const first = pendingMove === null;
  pendingMove = imageUV(e);
  if (first) requestAnimationFrame(() => {
    if (pendingMove) send({ type: 'mouse', kind: 'move', ...clampUV(pendingMove) });
    pendingMove = null;
  });
});
video.addEventListener('pointerup', (e) => {
  if (!dragging) return;
  dragging = false;
  pendingMove = null;
  send({ type: 'mouse', kind: 'release', ...clampUV(imageUV(e)) });
});
video.addEventListener('contextmenu', (e) => {
  e.preventDefault();
  dragging = false;
  send({ type: 'mouse', kind: 'cancel', u: 0.5, v: 0.5 });
});

// ------------------------------------------------------------------ small 3-D simulator view: drag turns it
const simImg = $('sim'), simView = $('simview');
function startSim() { simImg.src = `/sim?t=${encodeURIComponent(token)}&r=${Date.now()}`; }
simImg.addEventListener('error', () => setTimeout(startSim, 1000));
let orbiting = false, orbitLast = null, orbitAcc = null;
simView.addEventListener('pointerdown', (e) => {
  if (e.button !== 0) return;
  orbiting = true; orbitLast = { x: e.clientX, y: e.clientY };
  simView.setPointerCapture(e.pointerId);
  simView.classList.add('dragging');
});
simView.addEventListener('pointermove', (e) => {
  if (!orbiting) return;
  const first = orbitAcc === null;
  orbitAcc = orbitAcc || { dx: 0, dy: 0 };
  orbitAcc.dx += e.clientX - orbitLast.x; orbitAcc.dy += e.clientY - orbitLast.y;
  orbitLast = { x: e.clientX, y: e.clientY };
  if (first) requestAnimationFrame(() => { act('orbit', orbitAcc); orbitAcc = null; });   // one message per frame
});
const endOrbit = () => { orbiting = false; simView.classList.remove('dragging'); };
simView.addEventListener('pointerup', endOrbit);
simView.addEventListener('pointercancel', endOrbit);

function toggleFullscreen() {         // F / the rail button; Esc (the browser's own) goes back to the window
  if (document.fullscreenElement) document.exitFullscreen().catch(() => {});
  else document.documentElement.requestFullscreen().catch(() => {});
}
$('fullscreen').addEventListener('click', toggleFullscreen);
document.addEventListener('fullscreenchange', () => setTimeout(reportSize, 300));

window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') { if (e.key === 'Escape') e.target.blur(); return; }
  if (e.key === 'Escape') { closeDialog(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey || e.repeat) return;
  const k = e.key.length === 1 ? e.key.toUpperCase() : '';
  if (k === 'F') { toggleFullscreen(); e.preventDefault(); return; }
  if (KEYS.includes(k)) { key(k); e.preventDefault(); }
});

// ------------------------------------------------------------------ side rail, sidebar, chat panel
function setSidebar(open) {
  $('sidebar').classList.toggle('hidden', !open);
  $('side-close').classList.toggle('hidden', !open);
  $('rail-open').classList.toggle('hidden', open);
  if (open) document.body.classList.remove('chat-open');
}
function setChat(open) {
  document.body.classList.toggle('chat-open', open);
  if (open) setSidebar(false);
  renderSuggest();
}
const NAV_LABEL = { new: '새 대화', saved: '저장된 경로', recent: '최근 대화', settings: '설정' };
document.querySelectorAll('[data-nav]').forEach((b) => b.addEventListener('click', () => {
  const nav = b.dataset.nav;
  if (nav === 'chat') { setChat(!document.body.classList.contains('chat-open') || !$('sidebar').classList.contains('hidden')); return; }
  toastLocal(`${NAV_LABEL[nav]} 준비 중이에요`, '지금은 쿠챗과 화면 조작으로 진행해 주세요.', false);
}));
$('rail-open').addEventListener('click', () => setSidebar(true));
$('side-close').addEventListener('click', () => setSidebar(false));
$('chat-close').addEventListener('click', () => setChat(false));
$('chat-form').addEventListener('submit', (e) => {
  e.preventDefault();
  $('chat-text').value = '';
  $('chat-text').blur();
  toastLocal('글자 입력은 준비 중이에요', '말로 해 주세요.', false);
});
$('graph-btn').addEventListener('click', () => { graphOpen = !graphOpen; renderGraph(); });
document.querySelectorAll('.step').forEach((b) => b.addEventListener('click', () => {
  if (b.classList.contains('can')) act('step', { index: Number(b.dataset.step) });
}));

// ------------------------------------------------------------------ render
function render() {
  if (!S) return;
  renderMenu();
  renderBadges();
  renderChat();
  renderSuggest();
  renderGraph();
  renderActions();
  renderToasts();
  renderDialog();
}

function renderMenu() {
  const m = S.menu, en = S.enabled;
  const can = [en.N, en.E, en.S || en.V, en.Y];
  document.querySelectorAll('.step').forEach((b) => {
    const i = Number(b.dataset.step);
    const done = i < m.step, current = i === m.step;
    b.className = 'step' + (done ? ' done' : '') + (current && !m.error ? ' current' : '') +
      (current && m.error ? ' error' : '') + (can[i] ? ' can' : '');
    const src = A + (done ? 'check-big.svg' : STEP_ICONS[i]);
    if (b.firstElementChild.getAttribute('src') !== src) b.firstElementChild.setAttribute('src', src);
  });
  const chip = $('chip'), img = chip.firstElementChild;
  const at = Math.min(m.step, 3);
  chip.style.left = `calc(${8 + at * 72 + 30} * var(--u))`;
  chip.classList.toggle('error', !!m.error);
  chip.lastElementChild.textContent = m.chip.text;
  const icon = m.chip.icon ? A + CHIP_ICONS[m.chip.icon] : null;
  if (icon) { if (img.getAttribute('src') !== icon) img.setAttribute('src', icon); } else img.removeAttribute('src');
  img.classList.toggle('spin', m.chip.icon === 'loading');
}

function chipHTML(text, icon, error) {
  return `<div class="chip${error ? ' error' : ''}">${icon ? `<img src="${A}${icon}" alt="">` : ''}<span>${esc(text)}</span></div>`;
}
function renderBadges() {
  if (!S) { $('badges').innerHTML = chipHTML('앱과 연결이 끊겼어요', 'warning-triangle.svg', true); return; }
  let h = '';
  if (S.calib) h += chipHTML(S.calib.text, S.calib.level === 'ok' ? 'check.svg' : 'warning-triangle.svg', S.calib.level !== 'ok');
  h += chipHTML(S.voice ? '쿠옹이 연결됨' : '쿠옹이 연결 대기', S.voice ? 'check.svg' : null, false);
  if (S.arm) h += chipHTML(S.arm, null, false);
  if ($('badges').innerHTML !== h) $('badges').innerHTML = h;
}

function renderSuggest() {          // chat folded: what 쿠옹이 understands now ("국그릇으로 가자" ...)
  const el = $('suggest');
  const say = (S && S.suggest) || [];
  const show = say.length > 0 && !document.body.classList.contains('chat-open');
  el.classList.toggle('hidden', !show);
  const sig = JSON.stringify(say);
  if (!show || el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  el.innerHTML = '';
  const label = document.createElement('span');
  label.className = 'say-label';
  label.textContent = '이렇게 말해 보세요';
  el.appendChild(label);
  for (const t of say) {
    const s = document.createElement('span');
    s.className = 'say';
    s.textContent = `"${t}"`;
    el.appendChild(s);
  }
}

function renderChat() {
  const list = $('chat-list');
  $('chat-intro').classList.toggle('hidden', S.chat.length > 0);
  $('intro-status').textContent = S.voice ? '쿠옹이가 듣고 있어요' : '쿠옹이 연결 대기';
  let changed = false;
  for (const m of S.chat) {
    if (list.querySelector(`[data-id="${m.id}"]`)) continue;
    const el = document.createElement('div');
    el.className = `bubble ${m.role}`;
    el.dataset.id = m.id;
    list.insertBefore(el, list.querySelector('.tail'));
    if (m.role === 'kuong' && !firstChat) typeInto(el, m.text); else el.textContent = m.text;
    changed = true;
  }
  firstChat = false;
  const bubbles = list.querySelectorAll('.bubble:not(.tail)');
  for (let i = 0; i < bubbles.length - 60; i++) bubbles[i].remove();
  // tail: answer chips under the last question, or "쿠옹이가 듣고 있어요"
  const listening = (S.hint || '').includes('듣고 있어요');
  const sig = JSON.stringify([S.chips, listening]);
  let tail = list.querySelector('.tail');
  if (!tail || tail.dataset.sig !== sig) {
    if (tail) tail.remove();
    tail = document.createElement('div');
    tail.className = 'tail';
    tail.dataset.sig = sig;
    if (S.chips.length) {
      const box = document.createElement('div');
      box.className = 'chips';
      for (const c of S.chips) {
        const b = document.createElement('button');
        b.textContent = c.label;
        b.addEventListener('click', () => act('answer', { command: c.command, fields: c.fields, rev: c.rev }));
        box.appendChild(b);
      }
      tail.appendChild(box);
    }
    if (listening) {
      const t = document.createElement('div');
      t.className = 'bubble kuong typing';
      t.style.marginTop = 'calc(11.6 * var(--u))';
      t.innerHTML = '<span class="dots"><i></i><i></i><i></i></span><span>쿠옹이가 듣고 있어요</span>';
      tail.appendChild(t);
    }
    list.appendChild(tail);
    changed = true;
  }
  if (changed) list.scrollTop = list.scrollHeight;
}
function typeInto(el, text) {
  let i = 0;
  const step = () => {
    i = Math.min(text.length, i + 2);
    el.textContent = text.slice(0, i);
    $('chat-list').scrollTop = $('chat-list').scrollHeight;
    if (i < text.length) setTimeout(step, 30);
  };
  step();
}

function renderGraph() {
  $('graph-btn').classList.toggle('on', graphOpen);
  $('graph').classList.toggle('hidden', !graphOpen);
  if (!graphOpen || !S) return;
  const h = S.heights;
  const svg = $('graph-svg');
  if (!h) {
    $('graph-now').textContent = '-';
    $('graph-route').textContent = '경로 없음';
    $('graph-min').textContent = '';
    svg.innerHTML = '';
    $('graph-max-card').classList.add('hidden');
    $('graph-min-card').classList.add('hidden');
    return;
  }
  $('graph-now').textContent = h.now === null ? '-' : `${Math.round(h.now)}cm`;
  $('graph-route').textContent = h.route;
  $('graph-min').textContent = `Min : ${Math.round(h.min)}cm`;
  const W = 664, H = 174, X0 = 34, top = 6;
  const ymax = Math.max(10, Math.ceil(h.max / 5) * 5);
  const n = h.series.length;
  const X = (f) => X0 + f * (W - X0);
  const Y = (z) => H - (Math.max(0, z) / ymax) * (H - top);
  let s = `<defs><linearGradient id="g-line" gradientUnits="userSpaceOnUse" x1="0" y1="${Y(h.max)}" x2="0" y2="${Y(h.min)}">
    <stop offset="0" stop-color="#EA2102"/><stop offset="0.48" stop-color="#9D63A9"/><stop offset="1" stop-color="#61ABD5"/></linearGradient>
    <linearGradient id="g-hair" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#BDBDBD"/><stop offset="1" stop-color="#FEFDFF" stop-opacity="0.4"/></linearGradient></defs>`;
  for (let v = ymax; v > 0; v -= 5) {
    s += `<line x1="${X0}" x2="${W}" y1="${Y(v)}" y2="${Y(v)}" stroke="#E4E4E4" stroke-dasharray="4 4"/><text x="0" y="${Y(v) + 3.5}">${v}</text>`;
  }
  const zAt = (f) => { const t = f * (n - 1), i = Math.floor(t), k = t - i; return i + 1 < n ? h.series[i] * (1 - k) + h.series[i + 1] * k : h.series[n - 1]; };
  for (let x = X0; x <= W; x += 5.2) s += `<line x1="${x}" x2="${x}" y1="${Y(zAt((x - X0) / (W - X0)))}" y2="${H}" stroke="url(#g-hair)" stroke-width="1.6"/>`;
  const pts = h.series.map((z, i) => `${X(i / (n - 1)).toFixed(1)},${Y(z).toFixed(1)}`).join(' ');
  s += `<polyline points="${pts}" fill="none" stroke="url(#g-line)" stroke-width="2.6" stroke-linejoin="round"/>`;
  const xm = X(h.min_at), xM = X(h.max_at);
  s += `<line x1="${xm}" x2="${xm}" y1="${Y(h.min)}" y2="${H}" stroke="#61ABD6" stroke-width="2.4"/>`;
  s += `<circle cx="${xm}" cy="${Y(h.min)}" r="4.5" fill="#fff" stroke="#33596F" stroke-width="2"/>`;
  s += `<circle cx="${xM}" cy="${Y(h.max)}" r="5" fill="#fff" stroke="#EA2102" stroke-width="2.4"/>`;
  if (h.now !== null && h.now_at !== null) s += `<circle cx="${X(h.now_at)}" cy="${Y(h.now)}" r="6.5" fill="#1e1e1e" stroke="#fff" stroke-width="2.5"/>`;
  svg.innerHTML = s;
  placeCard('graph-max-card', `${Math.round(h.max)} cm`, xM, Y(h.max), true);
  placeCard('graph-min-card', `${Math.round(h.min)} cm`, xm, Y(h.min), false);
}
function placeCard(id, text, x, y, above) {   // value card next to its point, kept inside the panel
  const card = $(id);
  card.classList.remove('hidden');
  card.querySelector('b').textContent = text;
  // beside its point (right of it in the left part of the graph), never over the title and the big number
  const px = 75 + x;
  const left = Math.min(813 - 235, Math.max(20, px < 330 ? px + 18 : px - 250));
  const topY = Math.max(150, Math.min(300, 195 + y - (above ? 78 : -14)));
  card.style.left = `calc(${left} * var(--u))`;
  card.style.top = `calc(${topY} * var(--u))`;
}

let actionsSig = '';
function renderActions() {
  const st = S.state, en = S.enabled, items = [];
  if (S.edit_mode) {
    items.push(['soft', '되돌리기', () => act('undo'), !S.can_undo]);
    items.push(['soft', '다시하기', () => act('redo'), !S.can_redo]);
    items.push(['soft', '처음 경로', () => act('reset'), false]);
    items.push(['run', '수정 완료', askRun, false]);
  } else if (st === 'GENERATED' || st === 'SELECTED') {
    items.push(['soft', '경로 수정', () => key('E'), !en.E]);
    items.push(['run', '검증 후 이동', askRun, false]);
  } else if (st === 'PASSED') {
    items.push(['run', '실행하기', () => act('run'), !en.Y]);
  } else if (st === 'VALIDATING') {
    items.push(['soft', '경로 검증 중', null, true, 'loading.svg']);
    items.push(['soft', '정지', () => act('stop'), false, 'pause.svg']);
  } else if (st === 'EXECUTING') {
    items.push(['run', '실행중....', null, true]);
    items.push(['soft', '정지', () => act('stop'), false, 'pause.svg']);
  } else if (st === 'DONE') {
    items.push(['run', '실행 완료', null, true, 'check-big-white.svg']);
    items.push(['soft', '처음 자리로', () => key('H'), !en.H]);
  } else if (st === 'ABORTED') {
    items.push(['soft', '계속 가기', () => act('answer', { command: 'resume', fields: { direction: 'forward' }, rev: S.rev }), false]);
    items.push(['soft', '돌아가기', () => act('answer', { command: 'resume', fields: { direction: 'back' }, rev: S.rev }), false]);
    items.push(['soft', '처음 자리로', () => key('H'), !en.H]);
  } else if (st === 'HOLD') {
    items.push(['soft', '새 목표', () => key('N'), !en.N]);
  }
  const sig = JSON.stringify(items.map((x) => [x[0], x[1], x[3], x[4]]) + S.rev);
  if (sig === actionsSig) return;
  actionsSig = sig;
  const box = $('actions');
  box.innerHTML = '';
  for (const [kind, label, fn, disabled, icon] of items) {
    const b = document.createElement('button');
    b.className = kind === 'run' ? 'btn-run' : 'btn-soft';
    b.disabled = !!disabled || !fn;
    if (icon && kind === 'soft') b.innerHTML = `<img src="${A}${icon}" alt="" class="${icon === 'loading.svg' ? 'spin-img' : ''}">`;
    b.appendChild(document.createTextNode(label));
    if (icon && kind === 'run') b.insertAdjacentHTML('beforeend', `<img src="${A}${icon}" alt="">`);
    if (fn) b.addEventListener('click', fn);
    box.appendChild(b);
  }
}
function askRun() {
  localDialog = { title: '수정 완료', lines: ['검증 후 이동할까요?'], cancel: '취소', ok: '검증 후 이동', action: 'confirm_run', rev: S.rev };
  renderDialog();
}

let toastSig = '';
function toastLocal(title, text, warn) {
  localToasts.push({ id: `L${++localId}`, title, text, warn, t: Date.now() });
  renderToasts();
}
function renderToasts() {
  const now = Date.now();
  while (localToasts.length && now - localToasts[0].t > 4000) localToasts.shift();
  const all = (S ? S.toasts : []).concat(localToasts).slice(-4);
  const sig = all.map((t) => t.id).join(',');
  if (sig === toastSig) return;
  toastSig = sig;
  const box = $('toasts');
  const keep = new Set(all.map((t) => String(t.id)));
  box.querySelectorAll('.toast').forEach((el) => { if (!keep.has(el.dataset.id)) el.remove(); });
  for (const t of all) {
    if (box.querySelector(`[data-id="${t.id}"]`)) continue;
    const el = document.createElement('div');
    el.className = 'toast';
    el.dataset.id = t.id;
    el.innerHTML = `<div><b>${esc(t.title)}</b><span>${esc(t.text)}</span></div>` +
      `<i class="${t.warn ? '' : 'info'}"><img src="${A}${t.warn ? 'warning-toast.svg' : 'check-big-white.svg'}" alt=""></i>`;
    box.appendChild(el);
  }
}
setInterval(renderToasts, 1000);

function renderDialog() {
  const d = localDialog || (S && S.dialog && S.dialog.id > dismissedDialog ? S.dialog : null);
  $('dialog-layer').classList.toggle('hidden', !d);
  if (!d) return;
  $('dialog-title').textContent = d.title;
  $('dialog-body').textContent = d.lines.join('\n');
  $('dialog-cancel').textContent = d.cancel;
  $('dialog-ok').textContent = d.ok;
  $('dialog-ok').onclick = () => {
    if (d.action === 'edit' && S && !S.edit_mode) key('E');
    else if (d.action === 'new_target') key('N');
    else if (d.action === 'confirm_run') act('confirm_run', { rev: d.rev });
    closeDialog();
  };
  $('dialog-cancel').onclick = closeDialog;
}
function closeDialog() {
  if (localDialog) localDialog = null;
  else if (S && S.dialog) dismissedDialog = S.dialog.id;
  renderDialog();
}

function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

const OPEN = (new URLSearchParams(location.search).get('open') || '').split(',');   // previews: ?open=graph,sidebar
history.replaceState(null, '', location.pathname);   // the token stays in this page only, not in the address
setChat(!OPEN.includes('sidebar'));
if (OPEN.includes('sidebar')) setSidebar(true);
graphOpen = OPEN.includes('graph');
startVideo();
startSim();
connect();
function reportSize() { act('viewport', { w: innerWidth, h: innerHeight }); }   // logged by the app (window check)
reportSize();
let resizeTimer = null;
addEventListener('resize', () => { clearTimeout(resizeTimer); resizeTimer = setTimeout(reportSize, 400); });

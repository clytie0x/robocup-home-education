// Robot Console page: polls /api/state once a second, the run's status 3 times a second while a task runs
// (only while this tab is in view: that read is the run's page heartbeat), the LiDAR while it is on screen,
// the power bank estimate and the 21V motor battery's use every 15 s, and the Data tab's charts when it is open. Plain JS, no libraries.
"use strict";

const $ = (id) => document.getElementById(id);
const TABS = ["overview", "live", "task", "data", "more", "jetson", "checks", "logs", "help"];
const PHONE_ONLY_TABS = ["live", "more"]; // on a desktop the overview shows the camera and the LiDAR
const MORE_TABS = ["jetson", "checks", "logs", "help"];
const TITLES = { overview: "HMC Robot", live: "ภาพสด", task: "งาน", data: "ข้อมูล", more: "อื่นๆ", jetson: "Jetson",
  checks: "ตรวจระบบ", logs: "Log", help: "วิธีใช้" };
const PROGRAM_TEXT = {
  detector: "ตัวตรวจจับคน (server.py) TCP 5601",
  bridge: "ตัวส่งคำสั่งจาก ROS ไป Pi (base_bridge.py)",
  flow: "ลำดับงาน รับเสียง พูด และสถานะ (hmc_flow.py)",
};
const PROGRAM_NAME = { detector: "ตรวจจับคน", bridge: "สะพาน", flow: "HmcFlow" };
const PHASE_TEXT = { idle: "รอ Follow", follow: "กำลังตามคน", return: "กำลังกลับ", done: "เสร็จแล้ว" };
const ACT_TEXT = { start: "เปิด", stop: "ปิด", restart: "เปิดใหม่" };
const STOP_TEXT = { // run_follow's stop reasons
  obstacle: "มีสิ่งกีดขวาง", "no scan": "LiDAR ไม่ส่งข้อมูล", paused: "หยุดชั่วคราวจากหน้าเว็บ", idle: "รอกด Follow",
  ambiguous: "มีคนอยู่ติดคนที่ตาม", wheels: "ล้อไม่หมุน", wait: "กด Wait", silent: "ไม่ได้ยินจาก Jetson",
};
const DOT = { ok: "", warn: "ring", bad: "bad", idle: "idle" };
function dotOf(level) { return level in DOT ? DOT[level] : "idle"; }
const DESK = window.matchMedia("(min-width: 900px)");

let state = null;      // the newest /api/state
let run = null;        // the newest /api/run while a task runs
let bank = null;       // the newest /api/bank
let tab = "overview";
let choice = { mode: "dry", speed: null, minutes: 5, virtual: false };

// ------------------------------------------------------------ helpers

function esc(s) {
  return String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
function storeGet(key) { try { return localStorage.getItem(key); } catch (e) { return null; } }
function storeSet(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* private mode */ } }

// This tab's token: the console hands it out with the first command this tab sends to a run, and only that tab's
// status reads (the newest commander's) keep the run's page heartbeat alive. Per tab, so a second tab never does.
function session() { try { return sessionStorage.getItem("hmc-session"); } catch (e) { return null; } }
function setSession(t) { try { sessionStorage.setItem("hmc-session", t); } catch (e) { /* private mode */ } }

async function getJSON(url, headers) {
  const r = await fetch(url, { cache: "no-store", headers: headers || {} });
  const body = await r.json();
  if (body && typeof body === "object") Object.defineProperty(body, "_status", { value: r.status });
  return body;
}

async function post(url, body) {
  const headers = { "Content-Type": "application/json", "X-HMC-Console": "1" }; // the console takes no other POST
  if (session()) headers["X-HMC-Session"] = session();
  const pin = storeGet("console-pin");
  if (pin) headers["X-Console-Pin"] = pin;
  let r;
  try {
    r = await fetch(url, { method: "POST", headers, body: JSON.stringify(body || {}) });
  } catch (e) {
    toast("คอนโซลไม่ตอบ", true);
    return null;
  }
  let data = {};
  try { data = await r.json(); } catch (e) { /* empty */ }
  if (data && typeof data.session === "string") setSession(data.session);
  if (r.status === 401 && data.pin) {
    if (askPin()) return post(url, body);
    return null;
  }
  if (data && data.error) toast(data.error, true);
  return data;
}

function askPin() {
  const given = window.prompt("คอนโซลนี้ต้องใช้ PIN");
  if (!given) return false;
  storeSet("console-pin", given.trim());
  render();
  return true;
}

let toastTimer = null;
function toast(text, bad) {
  const el = $("toast");
  el.textContent = text;
  el.className = bad ? "bad" : "";
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, bad ? 6000 : 3000);
}

function confirmBox(title, text, yes, drive) {
  return new Promise((resolve) => {
    $("modal-title").textContent = title;
    $("modal-text").textContent = text;
    $("modal-yes").textContent = yes || "ใช่";
    $("modal").className = drive ? "modal drive" : "modal";
    $("modal").hidden = false;
    const done = (answer) => {
      $("modal").hidden = true;
      $("modal-yes").onclick = $("modal-no").onclick = null;
      resolve(answer);
    };
    $("modal-yes").onclick = () => done(true);
    $("modal-no").onclick = () => done(false);
  });
}

function pad2(n) { return String(n).padStart(2, "0"); }
function clock(seconds) {
  seconds = Math.max(0, Math.round(seconds || 0));
  const h = Math.floor(seconds / 3600), m = Math.floor((seconds % 3600) / 60), s = seconds % 60;
  return h ? `${h}:${pad2(m)}:${pad2(s)}` : `${pad2(m)}:${pad2(s)}`;
}
function hhmm(ts) { const d = new Date(ts * 1000); return `${pad2(d.getHours())}:${pad2(d.getMinutes())}`; }
function dayTime(ts) { const d = new Date(ts * 1000); return `${pad2(d.getDate())}/${pad2(d.getMonth() + 1)} ${hhmm(ts)}`; }
function fixed(v, n) { return v == null || !isFinite(v) ? "-" : Number(v).toFixed(n); }

function setText(el, text) { if (el.textContent !== text) el.textContent = text; }
function setHTML(el, html) { if (el.innerHTML !== html) el.innerHTML = html; }

function setLog(el, lines, empty) {
  const text = lines && lines.length ? lines.join("\n") : empty;
  if (el.textContent === text) return;
  const atEnd = el.scrollTop + el.clientHeight >= el.scrollHeight - 8;
  el.textContent = text;
  if (atEnd) el.scrollTop = el.scrollHeight;
}

function rows(items) { // [title, value, level, dot, note]
  return items.map(([k, v, level, dot, note]) =>
    `<div class="row${dot == null ? " two" : ""}">${dot == null ? "" : `<i class="sq ${dot}"></i>`}<span>${esc(k)}</span>` +
    `<span class="val ${level === "bad" ? "bad" : ""}" title="${esc(v)}">${esc(v)}</span></div>` +
    (note ? `<div class="row two note-row"><span class="note">${esc(note)}</span></div>` : "")).join("");
}

function tagHTML(text, dot, bad) {
  return `<span class="tag${bad ? " bad" : ""}">${dot == null ? "" : `<i class="sq ${dot}"></i>`}${esc(text)}</span>`;
}

// ------------------------------------------------------------ tabs

function showTab(name) {
  if (!TABS.includes(name)) name = "overview";
  if (DESK.matches && PHONE_ONLY_TABS.includes(name)) name = "overview";
  tab = name;
  for (const t of TABS) $(`tab-${t}`).hidden = t !== name;
  const phoneTab = MORE_TABS.includes(name) ? "more" : name;
  for (const a of document.querySelectorAll("#tabbar a")) a.classList.toggle("here", a.dataset.tab === phoneTab);
  for (const a of document.querySelectorAll("#desk-tabs a")) a.classList.toggle("here", a.dataset.tab === name);
  if (name === "logs") loadLogs();
  if (name === "data") loadData();
  window.scrollTo(0, 0);
  render();
}
window.addEventListener("hashchange", () => showTab(location.hash.slice(1)));
DESK.addEventListener("change", () => showTab(tab));
let resizeTimer = null;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => { place(); if (tab === "data") drawCharts(); }, 150);
});

// The camera, the LiDAR, the task buttons and the arm webcam each exist once: they move to the open tab's slot,
// so only one camera stream is ever open.
const PIECES = { cam: "cam-box", lidar: "lidar-box", cmds: "cmds-box", webcam: "webcam-box" };
function place() {
  for (const [slot, id] of Object.entries(PIECES)) {
    const piece = $(id);
    const target = [...document.querySelectorAll(`.slot[data-slot="${slot}"]`)].find((s) => s.getClientRects().length > 0);
    const home = target || $("parking");
    if (piece.parentElement !== home) home.appendChild(piece);
  }
  liveStreams();
}
function onScreen(id) { return $(id).parentElement !== $("parking"); }

// ------------------------------------------------------------ the whole state

function jobOf(s) { return s && s.job && s.job.job; }
function jobRunning(s) { return !!(s && s.job && s.job.running); }
function taskRunning(s) { const j = jobOf(s); return jobRunning(s) && j && j.kind === "task"; }
function card(id) { return state && (state.cards || []).find((c) => c.id === id); }

async function pollState() {
  try {
    state = await getJSON("/api/state");
    $("offline").hidden = true;
  } catch (e) {
    $("offline").hidden = false;
  }
  render();
  setTimeout(pollState, 1000);
}

// The run's status, through the console. heartbeat=1 tells run_follow that a page is in view; a hidden tab (a phone
// in a pocket) asks nothing, so the heartbeat stops with it.
async function fetchRun() {
  const s = session();
  try { run = await getJSON("/api/run?heartbeat=1", s ? { "X-HMC-Session": s } : {}); } catch (e) { run = { error: "the console did not answer" }; }
  renderRunParts();
}

async function pollRun() {
  if (taskRunning(state) && document.visibilityState === "visible") {
    await fetchRun();
  } else if (!taskRunning(state)) {
    run = null;
  }
  setTimeout(pollRun, 333);
}

async function pollBank() {
  try { bank = await getJSON("/api/bank"); } catch (e) { bank = null; }
  renderBank();
  setTimeout(pollBank, 15000);
}

function render() {
  if (!state) return;
  renderJob();
  renderOverview();
  renderBank();
  renderRunParts();
  if (tab === "live") renderLive();
  if (tab === "task") renderTask();
  if (tab === "jetson") renderJetson();
  if (tab === "checks") renderChecks();
  place();
}

// ------------------------------------------------------------ what the run is doing

function runFigures() {
  const r = run && run.record;
  if (!r) return null;
  const pos = r.operator || r.operator_kept;
  const op = pos && r.believed ? Math.hypot(pos[0] - r.believed[0], pos[1] - r.believed[1]) : null;
  const v = Array.isArray(r.measured) && r.measured[0] != null ? r.measured[0] : r.v;
  const remote = r.remote || (run && run.remote) || {};
  return { r, op, v, near: r.nearest, locked: !!r.locked, remote, phase: r.phase };
}

function stateWord() {
  if (!state) return ["-", ""];
  const j = jobOf(state), running = jobRunning(state);
  if (running && j.stopping) return ["กำลังหยุด", "bad"];
  if (running && j.kind === "task") {
    const f = runFigures();
    if (!f) return ["กำลังเริ่มงาน", ""];
    if (f.r.stop && f.r.stop !== "idle" && f.phase !== "done") return [`หยุดอยู่ เพราะ${STOP_TEXT[f.r.stop] || f.r.stop}`, ""];
    return [PHASE_TEXT[f.phase] || f.phase || "กำลังทำงาน", ""];
  }
  if (running) return [`กำลัง${j.label}`, ""];
  const bad = (state.cards || []).some((c) => c.level === "bad");
  return [bad ? "ไม่มีงาน" : "พร้อมทำงาน", ""];
}

function timeText() {
  const j = jobOf(state);
  if (!jobRunning(state) || !j) return "-";
  const total = j.info && j.info.minutes ? ` / ${clock(j.info.minutes * 60)}` : "";
  return `${clock(j.elapsed)}${total}`;
}

function lockTag() {
  if (!taskRunning(state)) return "";
  const f = runFigures();
  if (!f) return tagHTML("รอสถานะ", "idle");
  return f.locked ? tagHTML("ล็อคคน", "") : tagHTML("ยังไม่ล็อค", "ring");
}

function renderJob() {
  const j = jobOf(state), running = jobRunning(state);
  document.body.classList.toggle("job", running);
  const bar = $("stopbar"), desk = $("btn-stop-desk");
  bar.hidden = !running;
  desk.disabled = !running;
  const stopping = running && !!j.stopping;
  bar.classList.toggle("stopping", stopping);
  desk.classList.toggle("stopping", stopping);
  setText($("stopbar-text"), stopping ? "กำลังหยุด กดอีกครั้งเพื่อปิดทันที" : "STOP");
  setText(desk, stopping ? "ปิดทันที" : "STOP");
  const pin = $("pin-tag");
  setText(pin, !state.pin ? "ไม่มี PIN" : storeGet("console-pin") ? "PIN ✓" : "PIN");
}

function renderRunParts() {
  if (!state) return;
  const task = taskRunning(state), f = task ? runFigures() : null;
  const [word, level] = stateWord();
  // desktop status strip
  setText($("sl-state"), word);
  $("sl-state").className = level;
  setText($("sl-time"), timeText());
  setText($("sl-op"), f && f.op != null ? `${fixed(f.op, 2)} m` : "-");
  setText($("sl-speed"), f && f.v != null ? `${fixed(f.v, 2)} m/s` : "-");
  setText($("sl-near"), f && f.near != null ? `${fixed(f.near, 2)} m` : "-");
  setText($("sl-bank"), bank && bank.percent != null ? `~${fixed(bank.percent, 0)}%` : "-");
  const lock = $("sl-lock");
  lock.hidden = !task;
  if (task) {
    lock.querySelector(".sq").className = `sq ${f && f.locked ? "" : f ? "ring" : "idle"}`;
    setText(lock.querySelector("span"), f ? (f.locked ? "ล็อคคน" : "ยังไม่ล็อค") : "รอสถานะ");
  }
  // phone header bar
  renderBar(word);
  // the camera's labels
  const shown = !!$("img-camera").dataset.shown;
  setText($("cam-ov"), shown ? "XTION · LIVE" : "XTION");
  const op = $("cam-op");
  op.hidden = !(f && f.locked && f.op != null);
  if (!op.hidden) setText(op, `OPERATOR ${fixed(f.op, 2)} m`);
  // the three numbers under the camera, and the task buttons
  setHTML($("task-kv3"), [["ระยะคน", f && f.op != null ? fixed(f.op, 2) : "-", "m"],
    ["ความเร็ว", f && f.v != null ? fixed(f.v, 2) : "-", "m/s"], ["ใกล้สุด", f && f.near != null ? fixed(f.near, 2) : "-", "m"]]
    .map(([k, v, u]) => `<div><span>${k}</span><b>${v}<small>${u}</small></b></div>`).join(""));
  $("cmds").hidden = !task;
  $("idle-cmds").hidden = task;
  for (const b of $("idle-cmds").querySelectorAll("button")) b.disabled = jobRunning(state);
  const ready = !!(f && f.r);
  const active = !f ? "" : f.phase === "return" ? "return" : (f.remote.waiting || f.remote.held) ? "wait" :
    f.phase === "follow" ? "follow" : "";
  for (const b of document.querySelectorAll("[data-command]")) {
    b.disabled = !ready;
    b.classList.toggle("on", b.dataset.command === active);
  }
  if (task) {
    const items = runItems();
    if (tab === "task") setHTML($("task-kv"), rows(items));
    if (tab === "live") setHTML($("live-run-kv"), rows(items));
    setText($("task-phase"), f && f.phase ? f.phase.toUpperCase() : "กำลังเริ่ม");
  }
}

function runItems() {
  if (!run || run.error || !run.record) {
    return [["สถานะ", run && run.error ? "กำลังเริ่ม " + run.error : "กำลังเริ่ม"]];
  }
  const f = runFigures(), r = f.r;
  const wheels = r.wheels != null ? (typeof r.wheels === "object" ? JSON.stringify(r.wheels) : String(r.wheels)) :
    `v ${fixed(r.v || 0, 2)} m/s · w ${fixed(r.w || 0, 2)} rad/s`;
  const held = f.remote.held ? `พักอยู่ (${STOP_TEXT[f.remote.held] || f.remote.held})` : f.remote.waiting ? "รอ" : "";
  return [
    ["ช่วง", (PHASE_TEXT[r.phase] || r.phase || "-") + (held ? ` · ${held}` : "")],
    ["คนที่ตาม", r.locked ? `ล็อกแล้ว · ${f.op == null ? "-" : fixed(f.op, 2) + " m"}` : "ยังไม่ล็อก"],
    ["ใกล้สุด", r.nearest == null ? "-" : `${fixed(r.nearest, 2)} m`],
    [r.virtual ? "กล้องเสมือน" : "ลิงก์ Jetson", r.link ? "ใช้ได้" : "ขาด", r.link ? "" : "bad"],
    ["ล้อ", run.run && run.run.drive === false ? "ปิด (ทดลอง)" : wheels],
    ["เหตุที่หยุด", r.stop ? (STOP_TEXT[r.stop] || r.stop) : "ไม่มี"],
  ].concat(r.virtual ? [virtualItem(r.virtual)] : []).concat(r.end ? [["จบ", r.end]] : [])
    .concat(r.why ? [["เหตุผล", r.why]] : []);
}

// --virtual-operator: the simulated person from the PC (virtual_operator.py's report in the record)
function virtualItem(v) {
  if (!v.person) return ["คนเสมือน", v.from ? `ไม่ได้รับจาก PC ${fixed(v.age || 0, 1)} s` : "ยังไม่ได้รับจาก PC", "bad"];
  const said = v.speaking ? ` · พูด ${v.speaking}` : "";
  return ["คนเสมือน", `${v.seen ? "กล้องเห็น" : "กล้องไม่เห็น"} · LiDAR ${v.returns} จุด${said}`];
}

function linkTag() {
  const links = state.jetson.links || [];
  const lan = links.find((l) => l.host === "10.0.0.2");
  if (links.every((l) => l.ok == null)) return tagHTML("กำลังตรวจ", "idle");
  if (lan && lan.ok) return tagHTML("LAN", "");
  if (links.some((l) => l.ok)) return tagHTML("Wi-Fi", "ring");
  return tagHTML("Jetson ขาด", "", true);
}

// the commit base/tools/deploy.sh put on this machine (VERSION.json), so everyone sees which code runs
function codeText(v) {
  return v && v.describe ? ` · โค้ด ${v.describe}` : " · โค้ดไม่ได้ deploy จาก git";
}

function renderBar(word) {
  let title = TITLES[tab], sub = location.host, tag = "";
  const task = taskRunning(state), j = jobOf(state);
  if (tab === "overview") tag = linkTag();
  if (tab === "live") {
    sub = `XTION · ${$("cam-state").textContent}`;
    tag = lockTag();
  }
  if (tab === "task") {
    if (task) {
      const f = runFigures();
      title = word;
      sub = `${f && f.phase ? f.phase.toUpperCase() : "START"} · ${timeText()}`;
      tag = lockTag();
    } else if (jobRunning(state)) {
      sub = `${j.label} · ${timeText()}`;
    } else {
      sub = "เดินตามคน แล้วกลับจุดเริ่ม";
    }
  }
  if (tab === "data") {
    sub = `บันทึกทุก 5 วินาที${dataInfo.db_bytes != null ? " · " + bytes(dataInfo.db_bytes) : ""}`;
    tag = `<a class="tag" href="${esc($("csv-link").getAttribute("href"))}" download>CSV</a>`;
  }
  if (tab === "jetson") {
    const agent = state.jetson.agent;
    sub = agent ? `agent ${state.jetson.host}${codeText(agent.version)}` : "agent ไม่ตอบ";
    if (agent) {
      const progs = Object.values(agent.programs || {});
      tag = tagHTML(`${progs.filter((p) => p.running).length}/${progs.length}`, progs.every((p) => p.running) ? "" : "ring");
    } else tag = tagHTML("ไม่ตอบ", "bad");
  }
  if (tab === "checks" || tab === "logs" || tab === "help" || tab === "more") sub = location.host + codeText(state.version);
  setText($("bar-title"), title);
  setText($("bar-sub"), sub);
  setHTML($("bar-tag"), tag);
}

// ------------------------------------------------------------ Overview

function jetsonWatts(agent) {
  const vin = agent && agent.power && agent.power.VDD_IN;
  return vin && vin.mW != null ? vin.mW / 1000 : null;
}
function jetsonTemp(agent) {
  const t = agent && agent.thermal ? Object.values(agent.thermal).filter((x) => x != null) : [];
  return t.length ? Math.max(...t) : null;
}

function renderOverview() {
  const [word, level] = stateWord();
  setText($("state-word"), word);
  $("state-word").className = `v ${level}`;
  const agent = state.jetson.agent, pi = state.pi || {};
  const w = jetsonWatts(agent);
  setHTML($("n-bank"), bank && bank.percent != null ? `${fixed(bank.percent, 0)}<small>%</small>` : "-");
  setHTML($("n-pi"), pi.temp_c != null ? `${fixed(pi.temp_c, 0)}<small>°C</small>` : "-");
  setHTML($("n-jetson"), w != null ? `${fixed(w, 1)}<small>W</small>` : "-");
  const go = $("btn-go-task");
  setText(go.querySelector("span"), taskRunning(state) ? "ดูงาน" : "เริ่มงาน");
  go.disabled = jobRunning(state) && !taskRunning(state);
  for (const b of $("ov-buttons").querySelectorAll("[data-check]")) b.disabled = jobRunning(state);
  // faults that need action: what was measured, then what to do
  const cards = state.cards || [];
  setHTML($("faults"), cards.filter((c) => c.level === "bad").map((c) => `
    <div class="fault"><div class="h"><svg width="16" height="16"><use href="#i-warn"/></svg>${esc(c.title)}</div>
      <p class="num">${esc(c.value)}</p><div class="act">${esc(c.text)}</div></div>`).join(""));
  // the system list
  setHTML($("sys-rows"), rows(cards.map((c) => [c.title, c.value, c.level, dotOf(c.level),
    c.level === "warn" ? c.text : null])));
  const ok = cards.filter((c) => c.level === "ok").length;
  setText($("sys-count"), `${ok}/${cards.length}`);
  $("sys-count").className = `tag small${cards.some((c) => c.level === "bad") ? " bad" : ""}`;
  // desktop: the Jetson and the Pi
  const t = jetsonTemp(agent);
  setText($("jet-note"), agent ? `${t == null ? "-" : fixed(t, 0) + "°C"} · ${w == null ? "-" : fixed(w, 1) + " W"}` : "ไม่ตอบ");
  renderJetsonRows(agent);
  const load = pi.load ? fixed(pi.load[0], 2) : "-";
  setHTML($("pi-rows"), rows([["โหลด", load], ["อุณหภูมิ", pi.temp_c == null ? "-" : `${fixed(pi.temp_c, 1)}°C`],
    ["ดิสก์ว่าง", pi.disk_free_gb == null ? "-" : `${fixed(pi.disk_free_gb, 1)} GB`],
    ["เปิดมา", pi.uptime_s == null ? "-" : clock(pi.uptime_s)]]));
  setText($("cam-note-desk"), $("cam-state").textContent);
}

function renderJetsonRows(agent) {
  const box = $("jet-rows");
  const names = agent ? agent.order || Object.keys(agent.programs || {}) : ["detector", "bridge", "flow"];
  const key = names.map((n) => { const p = agent && agent.programs[n]; return `${n}:${p ? p.running : "?"}`; }).join(",");
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  box.innerHTML = names.map((n) => {
    const p = agent && agent.programs[n];
    const dot = !p ? "idle" : p.running ? "" : p.code != null && p.code !== 0 ? "bad" : "ring";
    const act = p && p.running ? "stop" : "start";
    return `<div class="row"><i class="sq ${dot}"></i><span>${esc(PROGRAM_NAME[n] || n)}</span>` +
      `<button type="button" class="btn slim" data-name="${esc(n)}" data-act="${act}" ${p ? "" : "disabled"}>` +
      `${p && p.running ? "หยุด" : "เปิด"}</button></div>`;
  }).join("");
  for (const b of box.querySelectorAll("button")) b.addEventListener("click", () => jetsonAct(b, b.dataset.act, b.dataset.name));
}

async function jetsonAct(b, act, name) {
  b.disabled = true;
  const r = await post(`/api/jetson/${act}`, { name });
  if (r && !r.error) toast(`${name} ${ACT_TEXT[act] || act} แล้ว`);
  b.disabled = false;
}

// ------------------------------------------------------------ the power bank (Pi) and the 21V motor battery

function renderMotor(none) {
  const m = none ? null : bank.motor;
  $("btn-motor-full").disabled = none;
  if (!m || m.since == null) {
    setText($("motor-use"), "-");
    setText($("motor-note"), none ? "" : "ยังไม่มีจุดอ้างอิง ชาร์จแบต 21V ให้เต็มแล้วกด ชาร์จเต็มแล้ว");
    return;
  }
  setText($("motor-use"), `ใช้ไป ${fixed(m.wh_used, 1)} Wh · ขับ ${fixed(m.drive_minutes, 0)} นาที ตั้งแต่ชาร์จเต็ม`);
  setText($("motor-note"), `ชาร์จเต็มเมื่อ ${dayTime(m.since)} ยังไม่รู้ความจุแบต จึงยังบอกเปอร์เซ็นต์ไม่ได้`);
}

function renderBank() {
  const none = !bank || bank.telemetry === false || bank._status >= 400;
  $("btn-bank-full").disabled = none;
  for (const el of $("bank-form").elements) el.disabled = none;
  renderMotor(none);
  if (none) {
    setText($("bank-pct"), "-");
    setText($("bank-range"), "");
    setText($("bank-hours"), "-");
    setText($("bank-drive"), "-");
    $("bank-warn").hidden = true;
    setText($("bank-note"), bank && bank.error ? bank.error : "อ่านค่าประมาณแบตไม่ได้");
    return;
  }
  const pct = bank.percent;
  const low = pct != null && pct < bank.warn_pct;
  setText($("bank-pct"), pct == null ? "-" : `${fixed(pct, 0)}%`);
  $("bank-pct").className = `num${low ? " bad" : ""}`;
  setText($("bank-range"), pct == null ? "" : `ช่วง ${fixed(bank.low, 0)}-${fixed(bank.high, 0)}%`);
  setText($("bank-hours"), bank.full_ts == null ? "-" : `${fixed((bank.now - bank.full_ts) / 3600, 1)} ชม.`);
  setText($("bank-drive"), bank.driving_min == null ? "-" : `${fixed(bank.driving_min, 0)} นาที`);
  $("bank-warn").hidden = !low;
  if (low) setText($("bank-warn-head"), `แบตเหลือประมาณ ${fixed(pct, 0)}%`);
  let note = "";
  if (pct == null) note = "ยังไม่มีจุดอ้างอิง ชาร์จให้เต็มแล้วกด ชาร์จเต็มแล้ว";
  else if (bank.mark_kind === "reading") note = `ค่าล่าสุดจากจอ ${fixed(bank.mark_percent, 0)}% เมื่อ ${dayTime(bank.mark_ts)}`;
  else note = `ชาร์จเต็มเมื่อ ${dayTime(bank.mark_ts)} ค่านี้คำนวณจากไฟที่ใช้ ไม่ได้วัดจริง`;
  setText($("bank-note"), note);
}

$("btn-bank-full").addEventListener("click", async () => {
  if (!(await confirmBox("ชาร์จเต็มแล้วใช่ไหม", "บันทึกว่าแบต power bank เต็ม 100% ตอนนี้ ค่าประมาณจะเริ่มนับใหม่จากตอนนี้", "ใช่ เต็มแล้ว"))) return;
  const r = await post("/api/bank/full", { battery: "bank" });
  if (r && !r.error) { bank = r.bank; renderBank(); render(); toast("บันทึกว่าชาร์จเต็มแล้ว"); }
});
$("btn-motor-full").addEventListener("click", async () => {
  if (!(await confirmBox("ชาร์จแบต 21V เต็มแล้วใช่ไหม", "บันทึกว่าแบต 21V ของมอเตอร์เต็มตอนนี้ พลังงานที่ใช้ไปจะเริ่มนับใหม่จากตอนนี้", "ใช่ เต็มแล้ว"))) return;
  const r = await post("/api/bank/full", { battery: "motor" });
  if (r && !r.error) { bank = r.bank; renderBank(); render(); toast("บันทึกว่าแบต 21V ชาร์จเต็มแล้ว"); }
});
$("bank-form").addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const text = $("bank-input").value.trim();
  const percent = Number(text);
  if (!text || !isFinite(percent) || percent < 0 || percent > 100) { toast("กรอกตัวเลข 0 ถึง 100", true); return; }
  const r = await post("/api/bank/reading", { percent });
  if (r && !r.error) { bank = r.bank; $("bank-input").value = ""; renderBank(); render(); toast(`บันทึกค่าจากจอ ${percent}% แล้ว`); }
});

// ------------------------------------------------------------ the live view

// During a task the console relays run_follow's own camera and LiDAR (same URLs); during a check they are paused.
function liveFromRun(s) { return taskRunning(s); }
function liveOwn(s) { return !!(s && !s.devices.held); }

function liveStreams() {
  const cam = $("img-camera"), web = $("img-webcam");
  const open = onScreen("cam-box");
  const source = !open || !state ? "" : liveFromRun(state) ? "run" :
    liveOwn(state) && state.devices.camera.state !== "off" ? "own" : "";
  if (cam.dataset.on && cam.dataset.on !== source) { delete cam.dataset.on; delete cam.dataset.shown; cam.removeAttribute("src"); }
  const waiting = Date.now() < Number(cam.dataset.retry || 0); // after an error: try again a few seconds later
  if (source && !cam.dataset.on && !waiting) { cam.dataset.on = source; cam.src = `/stream?t=${Date.now()}`; }
  const webOpen = onScreen("webcam-box") && state;
  if (webOpen && !web.dataset.on && Date.now() >= Number(web.dataset.retry || 0)) { web.dataset.on = "1"; web.src = `/usb-stream?t=${Date.now()}`; }
  if (!webOpen && web.dataset.on) { delete web.dataset.on; delete web.dataset.shown; web.removeAttribute("src"); }
  renderCamState();
}

$("img-camera").addEventListener("error", () => {
  const cam = $("img-camera");
  if (!cam.dataset.on) return;
  delete cam.dataset.on;
  cam.removeAttribute("src");
  delete cam.dataset.shown;
  cam.dataset.retry = String(Date.now() + 3000);
  setTimeout(liveStreams, 3100);
});
$("img-camera").addEventListener("load", () => { $("img-camera").dataset.shown = "1"; });
$("img-webcam").addEventListener("error", () => {
  const web = $("img-webcam");
  if (!web.dataset.on) return;
  delete web.dataset.on;
  web.removeAttribute("src");
  web.dataset.failed = "1";
  delete web.dataset.shown;
  web.dataset.retry = String(Date.now() + 5000);
  setTimeout(liveStreams, 5100);
});
$("img-webcam").addEventListener("load", () => { delete $("img-webcam").dataset.failed; $("img-webcam").dataset.shown = "1"; });

const DEVICE_WORD = { live: "สด", stale: "ไม่มีข้อมูลใหม่", off: "ไม่พบ", held: "หยุดชั่วคราว" };

function renderCamState() {
  if (!state) return;
  const shown = !!$("img-camera").dataset.shown;
  let text;
  if (liveFromRun(state)) text = shown ? "สด จากงาน" : "รอภาพจากงาน";
  else text = DEVICE_WORD[state.devices.camera.state] || "-";
  setText($("cam-state"), text);
  const web = state.jetson.webcam, failed = $("img-webcam").dataset.failed;
  const wt = web && !failed ? "สด" : web == null ? "กำลังตรวจ" : "ปิดอยู่";
  setText($("badge-webcam"), wt);
  setText($("badge-webcam-desk"), wt);
}

let scanLive = false; // the newest /scan answered and was fresh

function renderLive() {
  const d = state.devices;
  const fromRun = liveFromRun(state);
  const held = d.held && jobRunning(state) && !fromRun;
  $("live-paused").hidden = !held;
  $("live-cam-panel").hidden = held;
  $("live-lidar-panel").hidden = held;
  $("live-run").hidden = !(d.held && jobRunning(state));
  if (held) {
    const j = jobOf(state);
    setText($("live-paused-why"), `${j.label} ใช้กล้องและ LiDAR อยู่ เปิดได้ทีละโปรแกรม จบงานแล้วภาพจะกลับมาเอง`);
  }
  if (!taskRunning(state) && jobRunning(state)) {
    const j = jobOf(state);
    setHTML($("live-run-kv"), rows([["งาน", j.label], ["ทำงานมา", clock(j.elapsed)]]));
  }
  setText($("live-run-note"), taskRunning(state) ? "งาน" : "ตรวจ");
  if (!fromRun && d.lidar.state === "off") setText($("scan-info"), `ไม่พบ LiDAR`);
}

const RANGE = 4.0; // m ahead of the robot shown
const OUTLINE = [[0.2, 0.16], [0.16, 0.2], [-0.16, 0.2], [-0.2, 0.16], [-0.2, -0.16], [-0.16, -0.2], [0.16, -0.2], [0.2, -0.16]];
const HALF_FOV = 29 * Math.PI / 180; // the Xtion's half view

function drawScan(canvas, d) {
  const dpr = window.devicePixelRatio || 1;
  const cw = canvas.clientWidth, chh = canvas.clientHeight;
  if (!cw || !chh) return;
  const w = Math.round(cw * dpr), h = Math.round(chh * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  const g = canvas.getContext("2d"), k = dpr;
  const ox = w / 2, oy = h * 0.8;
  const s = Math.min(ox / 2.5, oy / RANGE); // px per m: 2.5 m to each side, RANGE ahead
  const px = (x, y) => [ox - y * s, oy - x * s];
  const mono = `${Math.round(10.5 * k)}px ui-monospace, "SF Mono", Menlo, Consolas, monospace`;
  g.fillStyle = "#fff"; g.fillRect(0, 0, w, h);
  g.strokeStyle = "#e2e2e2"; g.lineWidth = k;
  g.beginPath(); g.moveTo(ox, 0); g.lineTo(ox, h); g.moveTo(0, oy); g.lineTo(w, oy); g.stroke();
  g.fillStyle = "#8f8f8f"; g.font = mono;
  for (let m = 1; m * s < Math.hypot(w, h); m++) {
    g.beginPath(); g.arc(ox, oy, m * s, 0, 2 * Math.PI); g.stroke();
    if (oy - m * s > 12 * k) g.fillText(`${m} m`, ox + 4 * k, oy - m * s - 4 * k);
  }
  g.setLineDash([3 * k, 3 * k]);
  g.strokeStyle = "#bdbdbd";
  g.beginPath(); // the camera's view
  for (const a of [-HALF_FOV, HALF_FOV]) { const q = px(RANGE * 1.2 * Math.cos(a), RANGE * 1.2 * Math.sin(a)); g.moveTo(ox, oy); g.lineTo(q[0], q[1]); }
  g.stroke();
  g.beginPath(); g.arc(ox, oy, 0.5 * s, 0, 2 * Math.PI); g.stroke(); // the 0.5 m zone where the robot stops for anything
  g.setLineDash([]);
  const older = (d.older || []).slice().reverse();
  older.forEach((pts) => {
    g.fillStyle = "#cfcfcf";
    for (const p of pts) { const q = px(p[0], p[1]); g.fillRect(q[0] - k, q[1] - k, 2 * k, 2 * k); }
  });
  g.fillStyle = "#000";
  let near = null;
  for (const p of d.points) {
    const q = px(p[0], p[1]);
    g.beginPath(); g.arc(q[0], q[1], 1.6 * k, 0, 2 * Math.PI); g.fill();
    const r = Math.hypot(p[0], p[1]);
    if (near == null || r < near[0]) near = [r, q];
  }
  if (near) { g.strokeStyle = "#000"; g.lineWidth = k; g.beginPath(); g.arc(near[1][0], near[1][1], 6 * k, 0, 2 * Math.PI); g.stroke(); }
  g.strokeStyle = "#000"; g.lineWidth = 1.8 * k; // the robot
  g.beginPath();
  OUTLINE.forEach((p, i) => { const q = px(p[0], p[1]); i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]); });
  g.closePath(); g.stroke();
  const nose = px(0.35, 0);
  g.beginPath(); g.moveTo(ox, oy - 0.2 * s); g.lineTo(nose[0], nose[1]); g.stroke();
  // others: hollow black circles; the person the robot follows: a red ring with its distance
  for (const p of d.people || []) {
    if (!p) continue;
    const q = px(p[0], p[1]);
    g.strokeStyle = "#000"; g.lineWidth = 1.8 * k;
    g.beginPath(); g.arc(q[0], q[1], 5 * k, 0, 2 * Math.PI); g.stroke();
  }
  if (d.operator) {
    const q = px(d.operator[0], d.operator[1]);
    g.strokeStyle = "#e3120b"; g.fillStyle = "#e3120b"; g.lineWidth = 2 * k;
    g.beginPath(); g.arc(q[0], q[1], 8 * k, 0, 2 * Math.PI); g.stroke();
    g.beginPath(); g.arc(q[0], q[1], 2.5 * k, 0, 2 * Math.PI); g.fill();
    const label = `OPERATOR ${Math.hypot(d.operator[0], d.operator[1]).toFixed(2)} m`;
    g.font = `600 ${Math.round(11 * k)}px ui-monospace, "SF Mono", Menlo, Consolas, monospace`;
    const tw = g.measureText(label).width;
    const x = q[0] + tw + 14 * k > w ? q[0] - tw - 12 * k : q[0] + 12 * k;
    g.fillText(label, x, q[1] - 4 * k);
  }
  if (d.stop) {
    const text = `หยุด ${STOP_TEXT[d.stop] || d.stop}`, padx = 7 * k;
    g.font = `700 ${Math.round(13 * k)}px "Noto Sans Thai", "Sukhumvit Set", "Leelawadee UI", Tahoma, sans-serif`;
    const tw = Math.min(g.measureText(text).width, w - 16 * k - 2 * padx);
    g.fillStyle = "#fff"; g.fillRect(8 * k, 8 * k, tw + 2 * padx, 26 * k);
    g.strokeStyle = "#000"; g.lineWidth = 1.5 * k; g.strokeRect(8 * k, 8 * k, tw + 2 * padx, 26 * k);
    g.fillStyle = "#000"; g.fillText(text, 8 * k + padx, 26 * k, tw);
  }
}

async function pollScan() {
  const fromRun = !!state && liveFromRun(state);
  const shown = onScreen("lidar-box");
  if (shown && state && (fromRun || (liveOwn(state) && state.devices.lidar.state !== "off")) && document.visibilityState === "visible") {
    try {
      const d = await getJSON("/scan");
      if (!d.points) throw new Error(d.error || "no scan");
      scanLive = d.age != null && d.age < 1.0;
      drawScan($("scan"), d);
      delete $("scan").dataset.blank;
      const info = `${d.points.length} จุด · ใกล้สุด ${d.nearest == null ? "-" : d.nearest.toFixed(2) + " m"}`;
      setText($("scan-info"), info);
      setText($("scan-info-task"), info);
    } catch (e) {
      scanLive = false;
      if (fromRun) setText($("scan-info"), "รอข้อมูลจากงาน");
    }
  } else if (shown && !$("scan").dataset.blank) { // no LiDAR to read: the empty grid, once
    drawScan($("scan"), { points: [] });
    $("scan").dataset.blank = "1";
  }
  setTimeout(pollScan, fromRun ? 200 : 100); // a run's page shares its process with the control loop: ask less often
}

// ------------------------------------------------------------ Task

function segment(id, onPick) {
  $(id).addEventListener("click", (ev) => {
    const b = ev.target.closest("button");
    if (!b || b.disabled) return;
    for (const o of $(id).querySelectorAll("button")) o.classList.toggle("on", o === b);
    onPick(b.dataset.value);
  });
}
segment("seg-mode", (v) => { choice.mode = v; updateStartButton(); });
segment("seg-speed", (v) => { choice.speed = Number(v); });
segment("seg-minutes", (v) => { choice.minutes = Number(v); });
segment("seg-person", (v) => { choice.virtual = v === "virtual"; $("virtual-fields").hidden = !choice.virtual; updateStartButton(); });
$("virtual-host").value = storeGet("virtual-host") || "";

function updateStartButton() {
  $("btn-start-task").textContent = (choice.mode === "drive" ? "เริ่มงาน (ขับจริง" : "เริ่มงาน (ทดลอง") +
    (choice.virtual ? " คนเสมือน)" : ")");
}

function renderSpeeds() {
  const el = $("seg-speed");
  if (el.dataset.built) return;
  el.dataset.built = "1";
  const offered = [0.08, 0.12];
  el.innerHTML = offered.map((s) => {
    const ok = state.speeds.includes(s);
    return `<button type="button" data-value="${s}" ${ok ? "" : "disabled title='เกิน max_speed ใน params.json'"}><span class="num">${s.toFixed(2)} m/s</span><small>${s === 0.08 ? "ระวัง" : "ปกติ"}</small></button>`;
  }).join("");
  choice.speed = state.speeds[0] || null;
  const first = el.querySelector(`button[data-value="${choice.speed}"]`);
  if (first) first.classList.add("on");
}

function renderTask() {
  renderSpeeds();
  const running = jobRunning(state), task = taskRunning(state), j = jobOf(state);
  $("task-start").hidden = running;
  $("task-running").hidden = !task;
  $("btn-start-task").disabled = running;
  const services = card("services");
  $("task-start-note").hidden = !!(services && services.level === "ok");
  if (j && j.kind === "task") {
    setText($("task-output-what"), `${task ? "กำลังทำงาน" : `จบแล้ว (code ${j.code})`} · 30 บรรทัดล่าสุด`);
    setLog($("task-output"), (j.notes || []).concat(state.job.tail), "(ยังไม่มีข้อความ)");
  } else if (running) {
    setText($("task-output-what"), `${j.label} กำลังทำงาน เริ่มงานได้เมื่อจบ`);
  }
  if (task) setText($("task-label"), j.label);
}

$("btn-start-task").addEventListener("click", async () => {
  const drive = choice.mode === "drive";
  if (!choice.speed) { toast("params.json ไม่อนุญาตความเร็วใดเลย", true); return; }
  if (drive && !(await confirmBox("เริ่มขับจริงไหม", "หุ่นจะเคลื่อนที่ พื้นที่โล่งและสายไฟปลอดภัยแล้วใช่ไหม", "ใช่ เริ่มเลย", true))) return;
  const body = { drive, speed: choice.speed, minutes: choice.minutes };
  if (choice.virtual) {
    const host = $("virtual-host").value.trim();
    body.virtual = true;
    if (host) body.virtual_host = host;
    storeSet("virtual-host", host);
  }
  const r = await post("/api/task/start", body);
  if (r && !r.error) toast(drive ? "เริ่มงานแล้ว หุ่นรอ Follow" : "เริ่มทดลองแล้ว");
});

for (const b of document.querySelectorAll("[data-command]")) {
  b.addEventListener("click", async () => {
    const r = await post("/api/run/command", { command: b.dataset.command });
    if (r && !r.error) { toast(`ส่ง ${b.textContent.trim()} แล้ว`); fetchRun(); } // this tab's heartbeat starts now
  });
}

async function stop() {
  const r = await post("/api/stop", {});
  if (r && !r.error) toast(r.note ? "ไม่มีงานของคอนโซลอยู่ ส่งหยุดไปที่หน้าของงานและส่ง S# ให้ล้อแล้ว" : "กำลังหยุด");
}
$("stopbar").addEventListener("click", stop);
$("btn-stop-desk").addEventListener("click", stop);
$("pin-tag").addEventListener("click", () => { if (state && state.pin) askPin(); });
$("btn-go-task").addEventListener("click", () => { location.hash = "#task"; });
$("btn-go-task-desk").addEventListener("click", () => { location.hash = "#task"; });

// ------------------------------------------------------------ Jetson

function renderJetson() {
  const j = state.jetson, agent = j.agent;
  let summary = agent ? `agent บน ${j.host}` : (j.agent_error || "agent ไม่ตอบ");
  if (agent) {
    const bits = [];
    const t = jetsonTemp(agent), w = jetsonWatts(agent);
    if (t != null) bits.push(`${t.toFixed(0)}°C`);
    if (agent.cpu_pct != null) bits.push(`CPU ${agent.cpu_pct.toFixed(0)}%`);
    if (agent.gpu_load_pct != null) bits.push(`GPU ${agent.gpu_load_pct.toFixed(0)}%`);
    if (agent.mem_used_mb != null) bits.push(`RAM ${(agent.mem_used_mb / 1024).toFixed(1)} จาก ${(agent.mem_total_mb / 1024).toFixed(1)} GB`);
    if (w != null) bits.push(`${w.toFixed(1)} W เข้า`);
    if (bits.length) summary += ` · ${bits.join(" · ")}`;
  }
  setText($("jetson-summary"), summary);
  $("btn-start-all").disabled = $("btn-stop-all").disabled = !agent;
  setText($("jetson-sequence"), agent && agent.sequence ? `เปิดทั้งหมดตามลำดับ ${agent.sequence}` : "");
  const outside = agent && agent.outside ? Object.entries(agent.outside) : [];
  $("jetson-outside").hidden = !outside.length;
  if (outside.length) {
    setText($("jetson-outside-text"), "คอนโซลไม่ยุ่งกับมัน " + outside.map(([n, pids]) => `${n} (pid ${pids.join(", ")})`).join(" ") +
      " ปิดเองด้วยมือ จะได้ไม่ทำงานซ้อนกันสองชุด");
  }
  const box = $("programs");
  const names = agent ? agent.order : ["detector", "bridge", "flow"];
  for (const name of names) {
    let el = box.querySelector(`[data-program="${name}"]`);
    if (!el) {
      el = document.createElement("div");
      el.className = "panel prog";
      el.dataset.program = name;
      el.innerHTML = `
        <div class="ph"><span><i class="sq idle"></i> ${esc(name)}</span><span class="state">-</span></div>
        <div class="meta">${esc(PROGRAM_TEXT[name] || "")}<br><span class="when num">-</span></div>
        <div class="buttons">
          <button type="button" class="btn" data-act="start">เปิด</button>
          <button type="button" class="btn" data-act="stop">ปิด</button>
          <button type="button" class="btn" data-act="restart">เปิดใหม่</button>
        </div>
        <pre class="log"></pre>`;
      for (const b of el.querySelectorAll("[data-act]")) b.addEventListener("click", () => jetsonAct(b, b.dataset.act, name));
      box.appendChild(el);
    }
    const p = agent && agent.programs[name];
    const dot = !p ? "idle" : p.running ? "" : p.code != null && p.code !== 0 ? "bad" : "ring";
    el.querySelector(".sq").className = `sq ${dot}`;
    const st = el.querySelector(".state");
    setText(st, !p ? "ไม่ทราบ" : p.running ? "ทำงาน" : p.code != null && p.code !== 0 ? `จบ code ${p.code}` : "หยุดอยู่");
    st.classList.toggle("bad", dot === "bad");
    setText(el.querySelector(".when"), !p ? "-" : p.running ? `ทำงานมา ${clock(p.uptime)} · pid ${p.pid}` :
      p.started ? `เปิดล่าสุด ${hhmm(p.started)}` : "ไม่ได้เปิดจากหน้านี้");
    el.querySelector('[data-act="start"]').disabled = !p || p.running;
    el.querySelector('[data-act="stop"]').disabled = !p || !p.running;
    el.querySelector('[data-act="restart"]').disabled = !p;
    setLog(el.querySelector("pre"), p && p.log, "(ยังไม่มี log)");
  }
}

$("btn-start-all").addEventListener("click", async () => {
  const r = await post("/api/jetson/start-all", {});
  if (r && !r.error) toast("กำลังเปิด detector แล้ว bridge แล้ว flow");
});
$("btn-stop-all").addEventListener("click", async () => {
  if (taskRunning(state) && !(await confirmBox("ปิดโปรแกรมบน Jetson ไหม", "งานกำลังวิ่งอยู่ ถ้าไม่มี detector และ bridge หุ่นจะตามใครไม่ได้และหยุดนิ่ง", "ปิดทั้งหมด"))) return;
  const r = await post("/api/jetson/stop-all", {});
  if (r && !r.error) toast("ปิด flow bridge และ detector แล้ว");
});

// ------------------------------------------------------------ Checks

const LEVEL_WORD = { ok: "ปกติ", warn: "ควรดู", bad: "เสีย", idle: "-" };

function renderChecks() {
  const w = state.wheel_test, c = card("wheels");
  if (c) {
    setHTML($("badge-wheel"), `<i class="sq ${dotOf(c.level)}"></i> ${esc(LEVEL_WORD[c.level])}`);
    setText($("wheel-last"), w ? `ครั้งล่าสุด ${c.value} · ${c.text}` : "ยังไม่เคยทดสอบล้อ");
  }
  const running = jobRunning(state), j = jobOf(state);
  for (const b of document.querySelectorAll("#tab-checks [data-check]")) b.disabled = running;
  if (j && j.kind.startsWith("check")) {
    setText($("check-output-what"), `${j.label} · ${running ? "กำลังทำงาน" : j.code === 0 ? "ผ่าน" : `code ${j.code}`}`);
    setLog($("check-output"), (j.notes || []).concat(state.job.tail), "(ยังไม่มีข้อความ)");
  }
}

for (const b of document.querySelectorAll("[data-check]")) {
  b.addEventListener("click", async () => {
    const check = b.dataset.check;
    if (check === "wheel" && !(await confirmBox("ทดสอบล้อไหม", "หุ่นจะเดินหน้าราว 4 ซม. พื้นที่โล่งและสายไฟปลอดภัยแล้วใช่ไหม", "ใช่ ทดสอบเลย", true))) return;
    const r = await post("/api/check/start", { check });
    if (r && !r.error) toast("เริ่มตรวจแล้ว");
  });
}

$("btn-linktest").addEventListener("click", async () => {
  const b = $("btn-linktest");
  b.disabled = true;
  setText(b, "กำลังทดสอบ");
  try {
    const r = await getJSON("/api/linktest");
    const port = { 8090: "agent 8090", 22: "ssh 22" };
    $("linktest-out").innerHTML = `<table class="links"><tr><th>ที่อยู่</th><th>พอร์ต</th><th>ms</th></tr>` +
      r.results.map((x) => `<tr><td>${esc(x.host)}</td><td>${esc(port[x.port] || x.port)}</td><td class="${x.ok ? "" : "bad"}">` +
        `${x.ms.map((m) => (m == null ? "ไม่ตอบ" : m.toFixed(1))).join(", ")}</td></tr>`).join("") + "</table>";
  } catch (e) {
    toast("ทดสอบลิงก์ไม่จบ", true);
  }
  b.disabled = false;
  setText(b, "ทดสอบลิงก์");
});

// ------------------------------------------------------------ Logs

function bytes(n) {
  if (n == null) return "-";
  return n > 1e6 ? `${(n / 1e6).toFixed(1)} MB` : `${Math.max(1, Math.round(n / 1e3))} kB`;
}

async function loadLogs() {
  let r;
  try { r = await getJSON("/api/logs"); } catch (e) { return; }
  const el = $("runs");
  if (!r.runs.length) { el.innerHTML = '<p class="note">ยังไม่มีงานที่บันทึกไว้</p>'; return; }
  el.innerHTML = r.runs.map((x) => {
    const end = x.end ? esc(x.end) : '<span class="bad">ไม่มีบันทึกตอนจบ ยังวิ่งอยู่หรือหยุดกลางคัน</span>';
    return `<div class="run-row">
      <div><div class="when">${esc(x.start)}</div>
        <div class="what"><b>${x.seconds == null ? "-" : clock(x.seconds)}</b> · ${end} · <span class="num">${bytes(x.size)}</span></div></div>
      ${x.has_log ? `<a class="btn slim" href="/logs/${encodeURIComponent(x.name)}/log.jsonl" download>log.jsonl</a>` : '<span class="note">ไม่มี log</span>'}
    </div>`;
  }).join("");
}
setInterval(() => { if (tab === "logs") loadLogs(); }, 10000);

// ------------------------------------------------------------ Data

const METRICS = ["bank_est_pct", "pi_temp_c", "jetson_temp_tj_c", "jetson_temp_cpu_c", "jetson_temp_max_c",
  "jetson_vdd_in_mw", "link_cable_ms", "link_wifi_ms"];
let range = "today";
let dataInfo = { db_bytes: null };
let dataSeries = null;   // the newest /api/data/series
let dataRuns = [];       // /api/data/runs
let pickedRun = null;    // run_id picked in the table (null: the newest)
let runDetail = null;    // /api/data/run/<id>
let dataWindow = null;   // [since, until]
let dataBusy = false;

for (const b of document.querySelectorAll("#range button")) {
  b.addEventListener("click", () => {
    range = b.dataset.range;
    for (const o of document.querySelectorAll("#range button")) o.classList.toggle("on", o === b);
    loadData();
  });
}

function windowFor(now) {
  if (range === "hour") return [now - 3600, now];
  if (range === "week") return [now - 7 * 86400, now];
  if (range === "run") {
    const r = currentRun();
    if (!r || r.started == null) return null;
    return [r.started - 5, (r.ended || r.started + (r.seconds || 60)) + 5];
  }
  const d = new Date(); d.setHours(0, 0, 0, 0);
  return [d.getTime() / 1000, now];
}

function currentRun() { return dataRuns.find((r) => r.run_id === pickedRun) || dataRuns[0] || null; }

async function loadData() {
  if (dataBusy) return;
  dataBusy = true;
  try {
    const runs = await getJSON("/api/data/runs?limit=50");
    const missing = runs._status === 503 && runs.telemetry === false;
    $("data-missing").hidden = !(missing || runs._status >= 400);
    if (runs._status >= 400) {
      setText($("data-missing-text"), missing ? "ไม่พบ telemetry.py บน Pi ข้อมูลส่วนนี้จึงว่าง" : (runs.error || "อ่านข้อมูลไม่ได้"));
      dataRuns = []; dataSeries = null; runDetail = null;
    } else {
      dataRuns = runs.runs;
      dataInfo = runs;
      setText($("sum-runs"), String(runs.total.runs));
      setText($("sum-dist"), `${fixed(runs.total.distance_m, 1)} m`);
      setText($("sum-db"), runs.db ? bytes(runs.db_bytes) : "ยังไม่มี");
      if (!runs.db) {
        $("data-missing").hidden = false;
        setText($("data-missing-text"), "ยังไม่มีไฟล์ข้อมูล (data/robot.db) ตัวบันทึกจะสร้างเองเมื่อคอนโซลทำงาน");
      }
      const now = Date.now() / 1000;
      dataWindow = windowFor(now);
      const points = Math.max(60, Math.min(400, Math.round(($("ch-bank").clientWidth || 360) / 2)));
      const q = dataWindow ? `&since=${dataWindow[0].toFixed(0)}&until=${dataWindow[1].toFixed(0)}` : "";
      dataSeries = dataWindow ? await getJSON(`/api/data/series?metrics=${METRICS.join(",")}${q}&points=${points}`) : null;
      $("csv-link").setAttribute("href", `/api/data/export.csv?metrics=${METRICS.join(",")}${q}`);
      const r = currentRun();
      runDetail = r ? await getJSON(`/api/data/run/${encodeURIComponent(r.run_id)}?points=300`) : null;
      setText($("data-window"), dataWindow ? `${dayTime(dataWindow[0])} ถึง ${dayTime(dataWindow[1])}` :
        "ยังไม่มีรอบวิ่งให้เลือก");
    }
  } catch (e) {
    $("data-missing").hidden = false;
    setText($("data-missing-text"), "คอนโซลไม่ตอบ");
  }
  dataBusy = false;
  renderRunsTable();
  drawCharts();
  if (state) renderBar(stateWord()[0]);
}
setInterval(() => { if (tab === "data" && document.visibilityState === "visible") loadData(); }, 30000);

function renderRunsTable() {
  const pick = currentRun();
  const t = $("runs-table");
  if (!dataRuns.length) { t.innerHTML = '<tr><td class="note">ยังไม่มีรอบวิ่งในฐานข้อมูล</td></tr>'; return; }
  t.innerHTML = `<tr><th>วันที่</th><th class="n">ยาว</th><th class="n">ระยะ</th><th>จบเพราะ</th></tr>` +
    dataRuns.map((r) => `<tr class="pick${pick && r.run_id === pick.run_id ? " on" : ""}" data-run="${esc(r.run_id)}">` +
      `<td class="num">${r.started == null ? esc(r.run_id) : dayTime(r.started)}</td><td class="n">${r.seconds == null ? "-" : clock(r.seconds)}</td>` +
      `<td class="n">${r.distance_m == null ? "-" : fixed(r.distance_m, 1) + " m"}</td>` +
      `<td class="why" title="${esc(r.end_detail || "")}">${esc(r.end_reason || "ไม่มีบันทึก")}</td></tr>`).join("");
  for (const tr of t.querySelectorAll("tr[data-run]")) {
    tr.addEventListener("click", () => { pickedRun = tr.dataset.run; loadData(); });
  }
}

function seriesOf(name, scale) {
  const s = dataSeries && dataSeries.series && dataSeries.series[name];
  return (s || []).map(([t, v]) => [t, v * (scale || 1)]);
}

function drawCharts() {
  // a line breaks where no sample came for longer than this (the Pi was off): 3 buckets, at least 90 s
  const xs = dataWindow, bucket = Math.max(3 * ((dataSeries && dataSeries.bucket_s) || 0), 90);
  const timeFmt = xs && xs[1] - xs[0] > 86400 ? dayTime : hhmm;
  const tj = ["jetson_temp_tj_c", "jetson_temp_cpu_c", "jetson_temp_max_c"].map((m) => seriesOf(m)).find((s) => s.length) || [];
  chart($("ch-bank"), { title: "แบต power bank (ประมาณ)", unit: "%", digits: 0, x: xs, gap: bucket, fmtX: timeFmt, y: [0, 100],
    series: [{ name: "ประมาณ", pts: seriesOf("bank_est_pct"), color: "#000" }],
    lines: [{ y: 30, color: "#e3120b", name: "เตือน 30%" }] });
  chart($("ch-temp"), { title: "อุณหภูมิ", unit: "°C", digits: 0, x: xs, gap: bucket, fmtX: timeFmt,
    series: [{ name: "Pi", pts: seriesOf("pi_temp_c"), color: "#000" }, { name: "Jetson", pts: tj, color: "#9a9a9a" }] });
  chart($("ch-power"), { title: "กำลังไฟ Jetson (VDD_IN)", unit: "W", digits: 1, x: xs, gap: bucket, fmtX: timeFmt,
    series: [{ name: "Jetson", pts: seriesOf("jetson_vdd_in_mw", 0.001), color: "#000" }] });
  chart($("ch-link"), { title: "ลิงก์ Pi ไป Jetson", unit: "ms", digits: 1, x: xs, gap: bucket, fmtX: timeFmt,
    series: [{ name: "สาย LAN", pts: seriesOf("link_cable_ms"), color: "#000" }, { name: "Wi-Fi", pts: seriesOf("link_wifi_ms"), color: "#9a9a9a" }] });
  const r = runDetail && runDetail.samples, run = runDetail && runDetail.run;
  const pts = (key) => (r ? r.t.map((t, i) => [t, r[key][i]]).filter((p) => p[1] != null) : []);
  const tmax = r && r.t.length ? r.t[r.t.length - 1] : null;
  chart($("ch-speed"), { title: run ? `ความเร็ว รอบ ${run.started ? dayTime(run.started) : run.run_id}` : "ความเร็ว (เลือกรอบวิ่ง)",
    unit: "m/s", digits: 2, x: tmax ? [0, tmax] : null, gap: Infinity, fmtX: (t) => clock(t),
    series: [{ name: "สั่ง", pts: pts("v_cmd"), color: "#9a9a9a" }, { name: "วัดได้", pts: pts("v_meas"), color: "#000" }] });
}

// One chart: thin polylines in an SVG drawn at the panel's own pixel size; a tap or hover shows the values there.
function chart(el, o) {
  const H = DESK.matches ? 120 : 84, W = Math.max(120, el.clientWidth || 320);
  const all = o.series.flatMap((s) => s.pts.map((p) => p[1])).concat((o.lines || []).map((l) => l.y));
  const any = o.series.some((s) => s.pts.length);
  const last = o.series.map((s) => s.pts.length ? s.pts[s.pts.length - 1][1] : null);
  const value = last.filter((v) => v != null).map((v) => fixed(v, o.digits)).join(" / ");
  let svg = "";
  if (any && o.x) {
    let lo = o.y ? o.y[0] : Math.min(...all), hi = o.y ? o.y[1] : Math.max(...all);
    if (!o.y) { const padv = (hi - lo) * 0.12 || Math.abs(hi) * 0.1 || 1; lo -= padv; hi += padv; }
    if (!o.y && Math.min(...all) >= 0 && lo < 0) lo = 0;
    const L = 30, R = 4, T = 6, B = 6;
    const sx = (t) => L + (t - o.x[0]) / (o.x[1] - o.x[0]) * (W - L - R);
    const sy = (v) => T + (hi - v) / (hi - lo) * (H - T - B);
    let g = "";
    for (const f of [0, 0.5, 1]) {
      const v = lo + (hi - lo) * (1 - f), y = sy(v);
      g += `<line x1="${L}" x2="${W - R}" y1="${y.toFixed(1)}" y2="${y.toFixed(1)}" stroke="#ececec"/>` +
        `<text x="${L - 4}" y="${(y + 3).toFixed(1)}" text-anchor="end">${fixed(v, o.digits === 2 ? 2 : 0)}</text>`;
    }
    for (const l of o.lines || []) {
      g += `<line x1="${L}" x2="${W - R}" y1="${sy(l.y).toFixed(1)}" y2="${sy(l.y).toFixed(1)}" stroke="${l.color}" stroke-width="1" stroke-dasharray="3 3"/>`;
    }
    for (const s of o.series) {
      const parts = [[]];
      s.pts.forEach((p, i) => {
        if (i && p[0] - s.pts[i - 1][0] > (o.gap || Infinity)) parts.push([]);
        parts[parts.length - 1].push(`${sx(p[0]).toFixed(1)},${sy(p[1]).toFixed(1)}`);
      });
      for (const part of parts) {
        if (part.length === 1) part.push(part[0]);
        if (part.length) g += `<polyline points="${part.join(" ")}" fill="none" stroke="${s.color}" stroke-width="1.6" stroke-linejoin="round"/>`;
      }
    }
    g += `<line class="cursor" x1="0" x2="0" y1="${T}" y2="${H - B}" stroke="#000" stroke-width="1" visibility="hidden"/>`;
    o.series.forEach((s, i) => { g += `<circle class="dot${i}" r="3" fill="${s.color}" visibility="hidden"/>`; });
    svg = `<svg viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" role="img" aria-label="${esc(o.title)}">${g}</svg>`;
    el._chart = { o, sx, sy, L, R, W };
  } else {
    svg = `<div class="empty">${o.x ? "ยังไม่มีข้อมูลในช่วงนี้" : "ยังไม่มีข้อมูล"}</div>`;
    el._chart = null;
  }
  const xText = o.x ? `${o.fmtX(o.x[0])} ถึง ${o.fmtX(o.x[1])}` : "";
  const legend = o.series.map((s) => `<span><i style="background:${s.color}"></i>${esc(s.name)}</span>`)
    .concat((o.lines || []).map((l) => `<span><i class="dash"></i>${esc(l.name)}</span>`)).join("");
  el.innerHTML = `<div class="top"><span>${esc(o.title)}</span><b>${value ? esc(value + (o.unit === "%" || o.unit === "°C" ? "" : " ") + o.unit) : "-"}</b></div>` +
    svg + `<div class="readout">${esc(xText)}</div><div class="leg">${legend}</div>`;
  el.dataset.xtext = xText;
  const s = el.querySelector("svg");
  if (s) {
    s.addEventListener("pointermove", (ev) => hover(el, ev));
    s.addEventListener("pointerdown", (ev) => hover(el, ev));
    s.addEventListener("pointerleave", (ev) => { if (ev.pointerType === "mouse") unhover(el); });
  }
}

function nearest(pts, t) {
  let lo = 0, hi = pts.length - 1;
  if (hi < 0) return null;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (pts[mid][0] < t) lo = mid; else hi = mid; }
  return Math.abs(pts[lo][0] - t) <= Math.abs(pts[hi][0] - t) ? pts[lo] : pts[hi];
}

function hover(el, ev) {
  const c = el._chart;
  if (!c) return;
  const box = ev.currentTarget.getBoundingClientRect();
  const x = (ev.clientX - box.left) * (c.W / box.width);
  const t = c.o.x[0] + (x - c.L) / (c.W - c.L - c.R) * (c.o.x[1] - c.o.x[0]);
  const bits = [];
  let at = null;
  c.o.series.forEach((s, i) => {
    const dot = el.querySelector(`.dot${i}`);
    const p = nearest(s.pts, t);
    if (!p || Math.abs(p[0] - t) > (c.o.x[1] - c.o.x[0]) / 20) { dot.setAttribute("visibility", "hidden"); return; }
    dot.setAttribute("cx", c.sx(p[0]).toFixed(1)); dot.setAttribute("cy", c.sy(p[1]).toFixed(1));
    dot.setAttribute("visibility", "visible");
    bits.push(`${s.name} ${fixed(p[1], c.o.digits)} ${c.o.unit}`);
    if (at == null) at = p[0];
  });
  const cur = el.querySelector(".cursor");
  if (at == null) { cur.setAttribute("visibility", "hidden"); setText(el.querySelector(".readout"), el.dataset.xtext); return; }
  cur.setAttribute("x1", c.sx(at).toFixed(1)); cur.setAttribute("x2", c.sx(at).toFixed(1));
  cur.setAttribute("visibility", "visible");
  setText(el.querySelector(".readout"), `${c.o.fmtX(at)} · ${bits.join(" · ")}`);
}

function unhover(el) {
  for (const n of el.querySelectorAll(".cursor, circle")) n.setAttribute("visibility", "hidden");
  const r = el.querySelector(".readout");
  if (r) setText(r, el.dataset.xtext || "");
}

// ------------------------------------------------------------ start

updateStartButton();
showTab(location.hash.slice(1) || "overview");
pollState();
pollRun();
pollScan();
pollBank();

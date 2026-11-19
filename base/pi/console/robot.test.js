"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const robot = require("./robot.js");

const NAMES = ["detector", "whisper", "voice", "bridge", "armstack", "webcam", "arm", "flow"];

function programs(running) {
  const out = {};
  NAMES.forEach(function (name) { out[name] = { running: running.indexOf(name) !== -1 }; });
  return out;
}

function baseState(over) {
  return Object.assign({
    cards: [
      { id: "pi", level: "ok", title: "Raspberry Pi", value: "48 °C", text: "ปกติ" },
      { id: "lidar", level: "ok", title: "LiDAR", value: "สด", text: "กำลังสแกน" },
      { id: "camera", level: "ok", title: "กล้อง Xtion", value: "สด", text: "ทำงานปกติ" },
      { id: "wheels", level: "ok", title: "ล้อและไฟมอเตอร์", value: "ผ่าน", text: "ล้อหมุนทั้งสองข้าง" },
      { id: "services", level: "ok", title: "โปรแกรมบน Jetson", value: "ทำงาน 8 จาก 8", text: "ครบ" },
      { id: "webcam", level: "ok", title: "เว็บแคมของแขน", value: "สด", text: "ทำงาน" },
    ],
    devices: {
      camera: { state: "live", age: 0.1, error: null },
      lidar: { state: "live", age: 0.1, error: null },
      held: false,
    },
    job: { running: false, job: null, tail: [] },
    jetson: {
      host: "10.0.0.2",
      webcam: true,
      agent: { order: NAMES.slice(), programs: programs(NAMES) },
    },
    version: { describe: "03bad17-dirty", commit: "03bad17" },
  }, over || {});
}

const healthyBank = { percent: 86, warning: false };
const healthyArm = { state: { state: "resting", fault: null, holding: false } };

test("a healthy robot is ready and the timeline sits on the first step", function () {
  const model = robot.viewModel(baseState(), { bank: healthyBank, arm: healthyArm });
  assert.equal(model.status.text, "พร้อมทำงาน");
  assert.equal(model.status.level, "ok");
  assert.equal(model.stepIndex, 0);
  assert.equal(model.steps[0].state, "now");
  assert.equal(model.banner, null);
  assert.equal(model.version, "03bad17");
  assert.equal(model.battery, "~86%");
  assert.deepEqual(model.lights.map(function (l) { return l.level; }), ["ok", "ok", "ok", "ok", "ok", "ok", "ok", "ok"]);
});

test("stopped Jetson programs are never called ready, even when the services card is only a warning", function () {
  const state = baseState();
  state.jetson.agent.programs = programs(["bridge", "flow"]);
  state.jetson.webcam = false;
  state.cards = state.cards.map(function (c) {
    return c.id === "services" ? Object.assign({}, c, { level: "warn", value: "ทำงาน 2 จาก 8" }) : c;
  });
  const model = robot.viewModel(state, { bank: healthyBank, arm: { error: "bag_arm.py is not answering on 127.0.0.1:5607" } });
  assert.notEqual(model.status.text, "พร้อมทำงาน");
  assert.equal(model.status.text, "ยังไม่พร้อม");
  const jetson = model.lights.find(function (l) { return l.id === "jetson"; });
  assert.equal(jetson.label, "Jetson 2/8");
  assert.equal(jetson.level, "warn");
  assert.equal(model.lights.find(function (l) { return l.id === "webcam"; }).level, "bad");
  assert.equal(model.lights.find(function (l) { return l.id === "reid"; }).level, "bad");
  assert.equal(model.lights.find(function (l) { return l.id === "arm"; }).level, "bad");
  assert.ok(model.banner);
  assert.match(model.banner.fix, /เปิดโปรแกรมหุ่น|5607|webcam/);
});

test("a stale wheel test and a low battery block ready", function () {
  const state = baseState();
  state.cards = state.cards.map(function (c) {
    return c.id === "wheels" ? Object.assign({}, c, { level: "warn", value: "ผ่าน 26 ชม.", text: "ทดสอบนานแล้ว ทดสอบใหม่ก่อนวิ่ง" }) : c;
  });
  const model = robot.viewModel(state, { bank: { percent: 22, warning: true }, arm: healthyArm });
  assert.equal(model.status.text, "ยังไม่พร้อม");
  assert.equal(model.lights.find(function (l) { return l.id === "wheels"; }).level, "warn");
  assert.equal(model.lights.find(function (l) { return l.id === "battery"; }).level, "warn");
});

test("state.mission is the only executive payload, and step_index 5 is the fifth step", function () {
  const state = baseState({
    mission: {
      active: true, state: "CONFIRM_CAR", outcome: null,
      step_index: 5, step_total: 11, elapsed_s: 75, remaining_s: 225, limit_s: 300,
      reason: { code: "ASK_CAR", en: "ask if this is the car", th: "กำลังถามว่าถึงรถหรือยัง", fix: "ตอบใช่เมื่อยืนที่รถแล้ว", detail: "", level: "warn" },
      carrying: false, car_pose: null, say: "", side: "left", log: [],
    },
    executive: { state: "FOLLOWING", step_index: 0, last_reason: { th: "ไม่ใช้" } },
    last_reason: { th: "ไม่ใช้" },
    job: { running: true, job: { kind: "task", elapsed: 10, info: { minutes: 5 }, label: "งาน" }, tail: [] },
  });
  const model = robot.viewModel(state, { bank: healthyBank, arm: healthyArm, run: { record: { phase: "idle", locked: false } } });
  assert.equal(model.status.text, "กำลังถามว่าถึงรถหรือยัง");
  assert.equal(model.mission.code, "CONFIRM_CAR");
  assert.equal(model.stepIndex, 4);
  assert.equal(model.steps[4].label, "ยืนยันรถ");
  assert.equal(model.steps[4].state, "now");
  assert.equal(model.steps[5].state, "todo");
  assert.equal(model.clock, "01:15 · เหลือ 03:45 · ขั้น 5/11");
  assert.doesNotMatch(model.clock, /ขั้น 6\/11/);
  assert.equal(model.banner.title, "กำลังถามว่าถึงรถหรือยัง");
  assert.equal(model.banner.fix, "ตอบใช่เมื่อยืนที่รถแล้ว");
  assert.equal(model.banner.level, "warn");
});

test("step_index 0 is idle and a failed reason stays until it is acknowledged", function () {
  const idle = baseState({
    mission: { active: false, state: "IDLE", outcome: null, step_index: 0, step_total: 11, elapsed_s: 0, remaining_s: 300, limit_s: 300, reason: null },
  });
  const idleModel = robot.viewModel(idle, { bank: healthyBank, arm: healthyArm });
  assert.equal(idleModel.stepIndex, -1);
  assert.ok(idleModel.steps.every(function (s) { return s.state === "todo"; }));
  assert.equal(idleModel.status.text, "ยังไม่เริ่ม");
  assert.equal(idleModel.mission.code, "IDLE");
  assert.doesNotMatch(idleModel.clock, /ขั้น /);

  const failed = baseState({
    mission: {
      active: false, state: "FAILED", outcome: "FAILED", step_index: 8, step_total: 11,
      elapsed_s: 40, remaining_s: 260, limit_s: 300,
      reason: { code: "ODOM_STALE", en: "wheel odometry went stale", th: "ค่าตำแหน่งล้อขาดช่วง การวิ่งหยุดแล้ว", fix: "ตรวจสาย encoder ของล้อ แล้วเริ่มงานใหม่", detail: "gap", level: "error" },
    },
  });
  const shown = robot.viewModel(failed, { bank: healthyBank, arm: healthyArm });
  assert.equal(shown.stepIndex, 7);
  assert.equal(shown.steps[7].label, "หยิบถุง");
  assert.equal(shown.steps[7].state, "bad");
  assert.equal(shown.banner.title, "ค่าตำแหน่งล้อขาดช่วง การวิ่งหยุดแล้ว");
  assert.equal(shown.banner.fix, "ตรวจสาย encoder ของล้อ แล้วเริ่มงานใหม่");
  assert.equal(shown.status.level, "bad");
  assert.notEqual(shown.status.text, "พร้อมทำงาน");
  const hidden = robot.viewModel(failed, { bank: healthyBank, arm: healthyArm, ack: shown.banner.key });
  assert.equal(hidden.banner, null);
  assert.equal(hidden.status.level, "bad");
});

test("guessed executive keys are ignored when mission is absent", function () {
  const state = baseState({
    executive: { state: "FOLLOWING", step_index: 2, last_reason: { th: "ไม่ใช้" } },
    task_exec: { state: "PICKING" },
    mission_state: "PICKING",
    step_index: 3,
    elapsed_s: 12,
    last_reason: { code: "PICK_MISSED", th: "คว้าไม่ถึง", fix: "จัดถุงใหม่" },
  });
  const mission = robot.readMission(state);
  assert.equal(mission.present, false);
  const model = robot.viewModel(state, { bank: healthyBank, arm: healthyArm });
  assert.equal(model.mission.code, "IDLE");
  assert.equal(model.banner, null);
  assert.equal(model.stepIndex, 0);
  assert.equal(model.status.text, "พร้อมทำงาน");
});

test("without a mission payload the run phase still moves the timeline", function () {
  const state = baseState({
    job: { running: true, job: { kind: "task", elapsed: 30, info: { minutes: 5 }, label: "โจทย์จริง" }, tail: [] },
  });
  const model = robot.viewModel(state, { bank: healthyBank, arm: healthyArm, run: { record: { phase: "bag", locked: true } } });
  assert.equal(model.status.text, "กำลังหาถุง");
  assert.equal(model.stepIndex, 6);
  assert.equal(model.steps[6].label, "ชี้ถุง");
  assert.match(model.clock, /^00:30 · เหลือ 04:30/);
  assert.doesNotMatch(model.clock, /ขั้น /);
});

test("camera labels keep the arm webcam and a Jetson snapshot apart", function () {
  const state = baseState();
  state.jetson.webcam = false;
  state.jetson.snap_url = "/usb-snap";
  state.jetson.snapshot_url = "http://10.0.0.2:8082/snap.jpg";
  let model = robot.viewModel(state, { bank: healthyBank, arm: healthyArm });
  assert.equal(model.cameras.arm.source, "webcam_view :8082 · /usb-stream");
  assert.match(model.cameras.arm.reason, /webcam/);
  assert.equal(model.cameras.jetson.url, null);
  assert.match(model.cameras.jetson.reason, /8082/);
  state.jetson.image = "/jetson-preview.jpg";
  model = robot.viewModel(state, { bank: healthyBank, arm: healthyArm });
  assert.equal(robot.jetsonSnapURL(state), "/jetson-preview.jpg");
  assert.equal(model.cameras.xtion.source, "สด /stream");
});

test("routine connection-refused lines are hidden and arm-stop failures stay in Thai", function () {
  const noise = robot.explainMessage("the run's status page does not answer yet (Connection refused)");
  assert.equal(noise.noise, true);
  const arm = robot.explainMessage("/api/arm/stop: bag_arm.py is not answering on 127.0.0.1:5607");
  assert.equal(arm.noise, false);
  assert.equal(arm.th, "หยุดแขนไม่สำเร็จ");
  assert.match(arm.fix, /5607/);
  assert.match(arm.fix, /ไม่ใช่แขนพัง/);
  const thai = robot.explainMessage("ตั้ง PIN ก่อน");
  assert.equal(thai.th, "ตั้ง PIN ก่อน");
});

test("the page files wire Operate, the 800px breakpoint, and motion-only STOP", function () {
  const dir = __dirname;
  const html = fs.readFileSync(path.join(dir, "index.html"), "utf8");
  const css = fs.readFileSync(path.join(dir, "style.css"), "utf8");
  const app = fs.readFileSync(path.join(dir, "app.js"), "utf8");
  assert.match(html, /id="operate"/);
  assert.match(html, /id="ws-engineer"/);
  assert.match(html, /raspberrypi4\.tail09b237\.ts\.net:8081/);
  assert.match(html, /แบต 21V/);
  assert.doesNotMatch(html, /USB-C/);
  assert.match(html, /id="seg-person"/);
  assert.match(html, /คนเสมือน/);
  assert.match(html, /webcam_view :8082/);
  assert.match(html, /ทั้งแปดตัว|ครบ 8|แปดโปรแกรม/);
  assert.doesNotMatch(html, /ทั้งสามตัว/);
  assert.match(html, /preact-htm\.js/);
  assert.match(html, /robot\.js/);
  assert.doesNotMatch(css, /min-width:\s*900px/);
  assert.match(css, /min-width:\s*800px/);
  const columns = [];
  const re = /\.op-stage\s*\{[^}]*grid-template-columns:\s*([^;]+);/g;
  let match;
  while ((match = re.exec(css))) columns.push(match[1].trim());
  assert.ok(columns.length >= 1);
  assert.match(columns[columns.length - 1], /1\.35fr/);
  assert.match(app, /\/api\/arm\/stop/);
  assert.match(app, /visibilityState/);
  const stopFn = app.slice(app.indexOf("async function stop"), app.indexOf("async function stop") + 700);
  assert.match(stopFn, /\/api\/stop/);
  assert.match(stopFn, /\/api\/arm\/stop/);
  assert.doesNotMatch(stopFn, /stop-all/);
  assert.match(app, /virtual/);
});

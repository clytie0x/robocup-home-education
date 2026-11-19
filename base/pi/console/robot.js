/* Operate workspace model for the HMC console.
   Pure functions (health, banner, timeline, wording) plus a small Preact render
   of the islands. No new libraries: preact and htm are the vendored pair. */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.HmcRobot = api;
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";

  /* Same order as hmc.flow.STEPS. step_index from /api/state.mission is 1-based into this list; 0 is idle. */
  const STEPS = ["ตรวจ", "รอเริ่ม", "จดจำคน", "ตามคน", "ยืนยันรถ", "ถึงรถ", "ชี้ถุง", "หยิบถุง", "เดินกลับ", "วางถุง", "เสร็จ"];
  const STATE_TH = {
    IDLE: "ยังไม่เริ่ม", PRECHECK: "กำลังตรวจก่อนเริ่ม", READY: "รอคำสั่งตาม",
    ENROLLING: "กำลังจดจำคนนำ", FOLLOWING: "กำลังตามคน", LOST_SEARCH: "ตามคนไม่เจอ กำลังหา",
    CONFIRM_CAR: "กำลังถามว่าถึงรถหรือยัง", AT_CAR: "ถึงรถแล้ว รอชี้ถุง",
    POINTING: "กำลังหาถุง", PICKING: "กำลังหยิบถุง", HANDOVER: "กำลังรับถุง",
    RETURNING: "กำลังกลับจุดเริ่ม", DROPPING: "กำลังวางถุง",
    DONE: "เสร็จแล้ว", FAILED: "งานไม่สำเร็จ", ABORTED: "หยุดงานแล้ว",
  };
  const PHASE_STEP = { idle: 1, follow: 3, bag: 6, return: 8, done: 10 };
  const PHASE_TH = { idle: "รอ Follow", follow: "กำลังตามคน", bag: "กำลังหาถุง", return: "กำลังกลับ", done: "เสร็จแล้ว" };
  const ARM_SOURCES = /usb-snap|usb-stream|:8082|snap\.jpg/i;
  let events = [];

  function num(value) {
    if (typeof value === "boolean" || value == null || value === "") return null;
    const n = Number(value);
    return Number.isFinite(n) ? n : null;
  }
  function pad2(n) { return String(n).padStart(2, "0"); }
  function formatClock(seconds) {
    if (seconds == null || !isFinite(seconds)) return null;
    seconds = Math.max(0, Math.round(seconds));
    const h = Math.floor(seconds / 3600), m = Math.floor((seconds % 3600) / 60), s = seconds % 60;
    return h ? h + ":" + pad2(m) + ":" + pad2(s) : pad2(m) + ":" + pad2(s);
  }
  function formatTime(t) {
    const ms = t > 1e12 ? t : t * 1000;
    const d = new Date(ms);
    return pad2(d.getHours()) + ":" + pad2(d.getMinutes()) + ":" + pad2(d.getSeconds());
  }
  function normalizeState(value) {
    return String(value || "").trim().toUpperCase().replace(/[\s-]+/g, "_");
  }
  function card(state, id) {
    return state && (state.cards || []).find(function (c) { return c.id === id; }) || null;
  }
  function light(id, level, label, detail, fix) {
    return { id: id, level: level, label: label, detail: detail || label, fix: fix || "", required: true };
  }

  function normalizeReason(raw) {
    if (!raw || typeof raw !== "object") return null;
    return {
      code: String(raw.code || ""),
      th: String(raw.th || ""),
      en: String(raw.en || ""),
      fix: String(raw.fix || ""),
      detail: String(raw.detail || ""),
      level: String(raw.level || ""),
    };
  }

  /* Only state.mission. Absent mission falls back to the run phase. Guessed bags are ignored. */
  function readMission(state) {
    const src = state && state.mission;
    if (!src || typeof src !== "object" || Array.isArray(src)) {
      return { present: false, active: false, state: null, outcome: null, step_index: null, step_total: STEPS.length,
        elapsed: null, remaining: null, limit: null, reason: null, carrying: false, say: "", side: null, log: null };
    }
    return {
      present: true,
      active: !!src.active,
      state: src.state ? String(src.state) : null,
      outcome: src.outcome ? String(src.outcome) : null,
      step_index: num(src.step_index),
      step_total: num(src.step_total) || STEPS.length,
      elapsed: num(src.elapsed_s),
      remaining: num(src.remaining_s),
      limit: num(src.limit_s),
      reason: normalizeReason(src.reason),
      carrying: !!src.carrying,
      car_pose: src.car_pose || null,
      say: src.say ? String(src.say) : "",
      side: src.side ? String(src.side) : null,
      log: src.log || null,
    };
  }

  function programRunning(agent, name) {
    if (!agent || !agent.programs || !Object.prototype.hasOwnProperty.call(agent.programs, name)) return null;
    return !!agent.programs[name].running;
  }

  function jetsonCount(state) {
    const agent = state && state.jetson && state.jetson.agent;
    if (!agent) return { agent: false, running: 0, total: 8, stopped: [] };
    const names = agent.order && agent.order.length ? agent.order.slice() : Object.keys(agent.programs || {});
    const total = names.length || 8;
    const stopped = names.filter(function (n) { return !(agent.programs[n] && agent.programs[n].running); });
    return { agent: true, running: total - stopped.length, total: total, stopped: stopped };
  }

  function healthLights(state, extras) {
    extras = extras || {};
    state = state || {};
    const jet = state.jetson || {};
    const agent = jet.agent;
    const count = jetsonCount(state);
    const stoppedText = count.stopped.length ? count.stopped.join(" ") : "ไม่ทราบชื่อ";
    const jetson = !count.agent
      ? light("jetson", "bad", "Jetson 0/8", "โปรแกรมบน Jetson ไม่ตอบ (0 จาก 8)", "กดเปิดโปรแกรมหุ่น แล้วรอจนไฟครบ")
      : light("jetson", count.running === count.total && count.total > 0 ? "ok" : count.running === 0 ? "bad" : "warn",
        "Jetson " + count.running + "/" + count.total,
        count.running === count.total ? "โปรแกรมบน Jetson ทำงานครบ" : "โปรแกรมบน Jetson ทำงาน " + count.running + " จาก " + count.total + " (" + stoppedText + " ปิดอยู่)",
        "กดเปิดโปรแกรมหุ่น แล้วรอจนครบ");

    const wheelsCard = card(state, "wheels");
    const wheels = wheelsCard
      ? light("wheels", wheelsCard.level === "idle" ? "warn" : wheelsCard.level,
        "ล้อ " + (wheelsCard.level === "ok" ? "ผ่าน" : wheelsCard.value || ""),
        wheelsCard.text || wheelsCard.value || "ล้อ", wheelsCard.text || "ทดสอบล้อใหม่ก่อนวิ่ง")
      : light("wheels", "warn", "ล้อ ยังไม่ทดสอบ", "ยังไม่เคยทดสอบล้อ", "กดทดสอบล้อ (หุ่นขยับราว 4 ซม.) ก่อนวิ่ง");

    function device(id, devKey, cardId, label, cable) {
      const dev = state.devices && state.devices[devKey];
      const c = card(state, cardId);
      if (dev && dev.state === "held") return light(id, "ok", label + " งานใช้", "งานที่ทำอยู่เปิด" + label + "อยู่", "");
      if (dev && dev.state === "live") return light(id, "ok", label + " สด", (c && c.text) || "ทำงานปกติ", "");
      if (dev && dev.state === "stale") return light(id, "warn", label + " ค้าง", (c && c.text) || "ไม่มีข้อมูลใหม่", "ตรวจ" + cable);
      if (c && c.level === "bad") return light(id, "bad", label + " ไม่พบ", c.text || c.value || "", c.text || "ตรวจ" + cable);
      if (dev && dev.state === "off") return light(id, "bad", label + " ไม่พบ", dev.error || "หาไม่เจอ", "ตรวจ" + cable);
      if (c) return light(id, c.level === "ok" ? "ok" : c.level === "bad" ? "bad" : "warn", label + " " + (c.value || ""), c.text || "", c.text || "");
      return light(id, "idle", label, "ยังไม่มีข้อมูล" + label, "");
    }
    const lidar = device("lidar", "lidar", "lidar", "LiDAR", "สาย USB ของ LiDAR");
    const xtion = device("xtion", "camera", "camera", "Xtion", "สาย USB ของ Xtion");

    const webcamProg = programRunning(agent, "webcam");
    let webcam;
    if (jet.webcam === false || webcamProg === false) {
      webcam = light("webcam", "bad", "กล้องแขน ปิด", "กล้องแขนปิดอยู่ เพราะโปรแกรม webcam บน Jetson ไม่ทำงาน", "กดเปิดโปรแกรมหุ่น ภาพนี้มาจาก webcam_view พอร์ต 8082 ไม่ใช่ภาพอื่น");
    } else if (jet.webcam === true || webcamProg === true) {
      webcam = light("webcam", "ok", "กล้องแขน สด", "กล้องแขนส่งภาพจาก webcam_view พอร์ต 8082 ทาง /usb-stream", "");
    } else {
      webcam = light("webcam", "idle", "กล้องแขน …", "กำลังตรวจกล้องแขน (webcam_view พอร์ต 8082)", "");
    }

    const armProg = programRunning(agent, "arm");
    const armBody = extras.arm;
    const armState = armBody && !armBody.error ? armBody.state : null;
    const armError = armBody && armBody.error ? String(armBody.error) : "";
    let arm;
    if (armState && armState.fault) {
      arm = light("arm", "bad", "แขน fault", "แขนแจ้ง fault " + armState.fault, "กด Stop ที่ Engineer แล้วสั่ง Rest เมื่อพื้นที่โล่ง");
    } else if (/not answering|connection refused|5607/i.test(armError) || armProg === false) {
      arm = light("arm", "bad", "แขน ปิด", "งานแขนไม่ทำงาน จึงไม่มีโปรแกรมฟังที่พอร์ต 5607", "กดเปิดโปรแกรมหุ่น ไม่ใช่แขนพัง");
    } else if (armError) {
      arm = light("arm", "warn", "แขน มีข้อความ", armError, "ดูรายละเอียดที่ Engineer");
    } else if (armState) {
      arm = light("arm", "ok", "แขน " + (armState.state || "พร้อม"), "แขนอยู่สถานะ " + (armState.state || "-") + (armState.holding ? " ถือถุงอยู่" : ""), "");
    } else if (armProg === true) {
      arm = light("arm", "ok", "แขน ทำงาน", "โปรแกรมแขนทำงานอยู่", "");
    } else {
      arm = light("arm", "idle", "แขน …", "ยังไม่ทราบสถานะแขน", "");
    }

    const det = programRunning(agent, "detector");
    const reasonCode = (readMission(state).reason || {}).code || "";
    let reid;
    if (/REID|TENSOR|DETECTOR/i.test(reasonCode) && det !== true) {
      reid = light("reid", "bad", "re-id ปิด", "ตัวจำคน (detector) ใช้ไม่ได้", "กดเปิดโปรแกรมหุ่น แล้วดู log ของ detector ใน Engineer");
    } else if (!count.agent || det === false) {
      reid = light("reid", "bad", "re-id ปิด", "โปรแกรม detector ที่จำคนไม่ทำงาน", "กดเปิดโปรแกรมหุ่น");
    } else if (det === true) {
      reid = light("reid", "ok", "re-id ทำงาน", "detector ทำงานอยู่", "");
    } else {
      reid = light("reid", "idle", "re-id …", "ยังไม่ทราบสถานะตัวจำคน", "");
    }

    const bank = extras.bank;
    const pct = bank ? num(bank.percent) : null;
    let battery;
    if (pct == null) {
      battery = light("battery", "idle", "แบต —", "ยังไม่มีค่าประมาณแบตของ power bank", "กรอกค่าจากจอแบตที่ Engineer ถ้าต้องการ");
      battery.required = false;
    } else if (pct < 15) {
      battery = light("battery", "bad", "แบต ~" + Math.round(pct) + "%", "power bank ต่ำกว่า 15% Pi อาจดับ", "ชาร์จ power bank (จ่ายไฟ Pi) ก่อนงาน");
    } else if (pct < 30 || bank.warning) {
      battery = light("battery", "warn", "แบต ~" + Math.round(pct) + "%", "power bank ต่ำกว่า 30%", "ชาร์จ power bank ก่อนงานยาว");
    } else {
      battery = light("battery", "ok", "แบต ~" + Math.round(pct) + "%", "power bank ประมาณ " + Math.round(pct) + "%", "");
    }
    return [jetson, wheels, lidar, xtion, webcam, arm, reid, battery];
  }

  function readyBlocked(lights) {
    return lights.filter(function (l) { return l.required && l.level !== "ok"; });
  }

  function bannerFrom(mission, lights, state) {
    const reason = mission.reason;
    if (reason && (reason.th || reason.en || reason.fix) && reason.level !== "info") {
      const title = reason.th || reason.en;
      const fix = reason.fix || "";
      const blob = (reason.code || "") + " " + title + " " + fix;
      const recovery = /AGENT|JETSON|PROGRAM|WEBCAM|DETECTOR|ARM|REID/i.test(blob) ? "programs" : null;
      const level = reason.level === "warn" ? "warn" : "bad";
      return { key: "reason:" + (reason.code || title), level: level, title: title, detail: reason.detail || reason.en || "", fix: fix, recovery: recovery };
    }
    const bad = lights.find(function (l) { return l.level === "bad"; });
    const warn = lights.find(function (l) { return l.level === "warn"; });
    const pick = bad || warn;
    if (pick) {
      const recovery = pick.id === "jetson" || pick.id === "webcam" || pick.id === "arm" || pick.id === "reid" ? "programs" : null;
      return {
        key: "light:" + pick.id + ":" + pick.level + ":" + pick.label,
        level: pick.level === "bad" ? "bad" : "warn",
        title: pick.detail || pick.label,
        detail: "",
        fix: pick.fix || "",
        recovery: recovery,
      };
    }
    const extra = (state.cards || []).find(function (c) { return c.level === "bad"; });
    if (!extra) return null;
    return { key: "card:" + extra.id, level: "bad", title: (extra.title || extra.id) + (extra.value ? " " + extra.value : ""), detail: "", fix: extra.text || "", recovery: null };
  }

  function phaseStep(run, taskOn) {
    const rec = run && run.record;
    if (!taskOn) return 0;
    if (!rec) return 1;
    if (rec.phase === "idle") return rec.locked ? 2 : 1;
    if (PHASE_STEP[rec.phase] != null) return PHASE_STEP[rec.phase];
    return 1;
  }

  function displayedStepNumber(mission) {
    if (!mission.present || mission.step_index == null || mission.step_index <= 0) return null;
    return { n: mission.step_index, total: mission.step_total || STEPS.length };
  }

  /* step_index is already 1..11. Subtract one for the array. 0 stays off the spine (idle). */
  function currentStep(mission, run, on) {
    if (mission.present && mission.step_index != null) {
      if (mission.step_index <= 0) return -1;
      return Math.max(0, Math.min(STEPS.length - 1, mission.step_index - 1));
    }
    return phaseStep(run, on);
  }

  function taskOn(state, run, mission) {
    if (mission && mission.present && mission.active) return true;
    const job = state && state.job && state.job.job;
    if (state && state.job && state.job.running && job && job.kind === "task") return true;
    const code = normalizeState(mission && mission.state);
    return !!code && code !== "IDLE" && code !== "READY" && code !== "PRECHECK";
  }

  function statusOf(lights, mission, running, phase) {
    const blocked = readyBlocked(lights);
    const code = normalizeState(mission.state);
    const failed = code === "FAILED" || code === "ABORTED" || mission.outcome === "FAILED" || mission.outcome === "ABORTED";
    if (failed) {
      const text = (mission.reason && (mission.reason.th || mission.reason.en)) || STATE_TH[code] || "งานไม่สำเร็จ";
      return { text: text, level: "bad" };
    }
    if (running) {
      const text = (code && STATE_TH[code]) || PHASE_TH[phase] || "กำลังทำงาน";
      return { text: text, level: blocked.some(function (l) { return l.level === "bad"; }) ? "bad" : "ok" };
    }
    if (mission.present && code === "IDLE") {
      if (blocked.some(function (l) { return l.level === "bad"; })) return { text: "ยังไม่พร้อม", level: "bad" };
      if (blocked.length) return { text: "ยังไม่พร้อม", level: "warn" };
      return { text: "ยังไม่เริ่ม", level: "ok" };
    }
    if (blocked.some(function (l) { return l.level === "bad"; })) return { text: "ยังไม่พร้อม", level: "bad" };
    if (blocked.length) return { text: "ยังไม่พร้อม", level: "warn" };
    return { text: "พร้อมทำงาน", level: "ok" };
  }

  function clockOf(mission, state) {
    const job = state && state.job && state.job.running ? state.job.job : null;
    const elapsed = mission.elapsed != null ? mission.elapsed : job && num(job.elapsed);
    const minutes = job && job.info ? num(job.info.minutes) : null;
    let remaining = mission.remaining;
    if (remaining == null && elapsed != null && minutes != null) remaining = minutes * 60 - elapsed;
    if (elapsed == null && remaining == null) return "—";
    const main = formatClock(elapsed) || "—";
    const extra = remaining != null ? " · เหลือ " + formatClock(Math.max(0, remaining)) : "";
    const step = displayedStepNumber(mission);
    const stepText = step ? " · ขั้น " + step.n + "/" + step.total : "";
    return main + extra + stepText;
  }

  function jetsonSnapURL(state) {
    const jet = (state && state.jetson) || {};
    const list = [jet.snap_url, jet.snapshot_url, jet.image, state && state.jetson_snap];
    for (let i = 0; i < list.length; i++) {
      const url = list[i];
      if (typeof url !== "string" || !url) continue;
      if (ARM_SOURCES.test(url)) continue;
      return url;
    }
    return null;
  }

  function viewModel(state, extras) {
    extras = extras || {};
    state = state || {};
    const mission = readMission(state);
    const lights = healthLights(state, extras);
    const run = extras.run || null;
    const rec = run && run.record;
    const on = taskOn(state, run, mission);
    const phase = rec && rec.phase;
    const status = statusOf(lights, mission, on, phase);
    let banner = bannerFrom(mission, lights, state);
    if (banner && extras.ack && extras.ack === banner.key) banner = null;
    const index = currentStep(mission, run, on);
    const failed = /FAILED|ABORTED/.test(normalizeState(mission.state)) || mission.outcome === "FAILED" || mission.outcome === "ABORTED";
    const steps = STEPS.map(function (label, i) {
      return { label: label, state: index < 0 ? "todo" : i < index ? "done" : i === index ? (failed ? "bad" : "now") : "todo" };
    });
    const code = normalizeState(mission.state);
    const label = code || (on && phase ? String(phase).toUpperCase() : "IDLE");
    const sideText = mission.side === "left" ? "ด้านซ้าย" : mission.side === "right" ? "ด้านขวา" : "";
    const text = [(code && STATE_TH[code]) || PHASE_TH[phase] || label, mission.carrying ? "ถือถุงอยู่" : "", sideText].filter(Boolean).join(" · ");
    const version = state.version && (state.version.describe || state.version.commit);
    const bank = extras.bank;
    const pct = bank ? num(bank.percent) : null;
    const webcamUp = !!(state.jetson && state.jetson.webcam === true);
    return {
      status: status,
      mission: { code: label, text: text, say: mission.say, carrying: mission.carrying, side: mission.side, active: mission.active },
      clock: clockOf(mission, state),
      battery: pct == null ? "—" : "~" + Math.round(pct) + "%",
      version: version ? String(version).split("-")[0] : "—",
      lights: lights,
      steps: steps,
      stepIndex: index,
      banner: banner,
      cameras: {
        xtion: { label: "Xtion (กล้องล่าง)", source: "สด /stream" },
        arm: {
          label: "กล้องแขน",
          source: "webcam_view :8082 · /usb-stream",
          up: webcamUp,
          reason: webcamUp ? "" : "ปิดอยู่ เพราะโปรแกรม webcam บน Jetson ไม่ทำงาน กดเปิดโปรแกรมหุ่น ช่องนี้คือกล้องแขน ไม่ใช่ภาพจากพอร์ตอื่น",
        },
        jetson: {
          label: "ภาพนิ่ง Jetson",
          source: jetsonSnapURL(state) || "ไม่มีภาพกล้องแยก",
          url: jetsonSnapURL(state),
          reason: jetsonSnapURL(state) ? "" : "พอร์ต 8082 และ /usb-snap คือกล้องแขน (ช่องกล้องแขน) ไม่มีภาพนิ่งคนละตัวให้ช่องนี้",
        },
        lidar: { label: "LiDAR มุมบน", source: "วงละ 1 m · /scan" },
      },
    };
  }

  function explainMessage(raw) {
    const text = String(raw || "").trim();
    if (!text) return null;
    if (/status page does not answer|live view does not answer|camera view does not answer/i.test(text)) {
      return { noise: true, level: "info", th: "", fix: "", key: "noise-status" };
    }
    const rules = [
      [/bag_arm\.py is not answering|127\.0\.0\.1:5607|\/api\/arm\/stop/i, "warn", "หยุดแขนไม่สำเร็จ", "งานแขนยังไม่ถูกเปิด จึงไม่มีโปรแกรมฟังที่พอร์ต 5607 ไม่ใช่แขนพัง", "arm-down"],
      [/jetson does not answer|no answer from agent/i, "bad", "Jetson ไม่ตอบ", "ตรวจว่า Jetson เปิดอยู่และสาย LAN เสียบ แล้วกดเปิดโปรแกรมหุ่นอีกครั้ง", "jetson-down"],
      [/MISSED|I missed the object/i, "warn", "แขนคว้าไม่ถึงถุง", "จัดถุงให้อยู่ในระยะแขน แล้วสั่งหยิบใหม่", "pick-missed"],
      [/NOT_HOLDING|not holding anything/i, "warn", "แขนไม่ได้ถือถุงอยู่", "จะวางได้เมื่อหยิบสำเร็จแล้วเท่านั้น", "not-holding"],
      [/HOLDING|already holding/i, "warn", "แขนถือของอยู่แล้ว", "วางของที่มีอยู่ก่อน แล้วค่อยหยิบใหม่", "holding"],
      [/UNREACHABLE/i, "warn", "ถุงอยู่ไกลเกินระยะแขน", "ขยับหุ่นหรือถุงให้เข้าใกล้มือ", "unreachable"],
      [/NO_BAG|no bag in view/i, "warn", "ไม่เห็นถุงในภาพกล้องแขน", "เปิดไฟและตรวจว่ากล้องแขนทำงาน แล้วชี้ถุงอีกครั้ง", "no-bag"],
      [/no reached bag/i, "warn", "ยังไม่มีตำแหน่งถุงที่วัดได้", "ให้หุ่นหาถุงจนถึงระยะก่อน แล้วค่อยกดหยิบ", "no-reach"],
      [/moving the base/i, "warn", "หุ่นกำลังเคลื่อนที่", "รอให้ฐานหยุด แล้วค่อยสั่งแขน", "base-moving"],
      [/console is held/i, "warn", "คอนโซลถูกจองอยู่", "รอจนปล่อยการจอง (deploy หรือซ่อม) STOP ยังใช้ได้", "held"],
      [/connection refused/i, "warn", "โปรแกรมที่สั่งยังไม่เปิด", "เป็นช่วงเปิดหรือปิดปกติ ถ้าจะทำงานต่อให้กดเปิดโปรแกรมหุ่น", "refused"],
    ];
    for (let i = 0; i < rules.length; i++) {
      if (rules[i][0].test(text)) {
        return { noise: false, level: rules[i][1], th: rules[i][2], fix: rules[i][3], key: rules[i][4] };
      }
    }
    if (/[ก-๙]/.test(text)) return { noise: false, level: "warn", th: text, fix: "", key: "th:" + text.slice(0, 80) };
    return { noise: false, level: "warn", th: "ระบบแจ้งว่า " + text, fix: "ถ้าต้องการข้อความดิบ เปิด Engineer แล้วดู Log", key: "en:" + text.slice(0, 80) };
  }

  function loadEvents() {
    try {
      const raw = root && root.sessionStorage && root.sessionStorage.getItem("hmc-events");
      if (raw) events = JSON.parse(raw) || [];
    } catch (e) { events = events || []; }
  }
  function saveEvents() {
    try {
      if (root && root.sessionStorage) root.sessionStorage.setItem("hmc-events", JSON.stringify(events.slice(0, 80)));
    } catch (e) { /* private mode */ }
  }
  function pushEvent(ev) {
    if (!ev || !ev.th) return events;
    const key = ev.key || ev.th;
    if (events.some(function (item) { return item.key === key; })) return events;
    events.unshift({ t: ev.t || Date.now() / 1000, level: ev.level || "info", th: ev.th, fix: ev.fix || "", key: key });
    events = events.slice(0, 80);
    saveEvents();
    return events;
  }
  function getEvents() { return events.slice(); }
  function clearEvents() { events = []; saveEvents(); }

  function renderOperate(model, actions) {
    const doc = typeof document !== "undefined" ? document : null;
    if (!doc || !model) return;
    actions = actions || {};
    const win = typeof window !== "undefined" ? window : null;
    if (win && win.preact && win.html) renderPreact(doc, model, actions, win);
    else renderPlain(doc, model, actions);
    paintHeader(doc, model);
  }

  function paintHeader(doc, model) {
    function put(id, text, bad) {
      const el = doc.getElementById(id);
      if (!el) return;
      if (el.textContent !== text) el.textContent = text;
      el.classList.toggle("bad", !!bad);
    }
    put("op-status", model.status.text, model.status.level === "bad");
    put("op-mission", model.mission.code);
    put("op-clock", model.clock);
    put("op-battery", model.battery, model.battery !== "—" && parseInt(model.battery.replace(/[^0-9]/g, ""), 10) < 30);
    put("op-code", model.version);
  }

  function renderPreact(doc, model, actions, win) {
    const html = win.html;
    const render = win.preact.render;
    const banner = doc.getElementById("op-banner");
    if (banner) {
      render(model.banner ? html`<div class="banner ${model.banner.level}" role="status">
        <i class="dot ${model.banner.level}"></i>
        <div class="txt">
          <strong>${model.banner.title}</strong>
          ${model.banner.fix ? html`<p>${model.banner.fix}</p>` : null}
        </div>
        ${model.banner.recovery === "programs" ? html`<button type="button" class="btn pri" onClick=${actions.startPrograms}>เปิดโปรแกรมหุ่น</button>` : null}
        <button type="button" class="btn" onClick=${function () { actions.ack && actions.ack(model.banner.key); }}>รับทราบ</button>
      </div>` : null, banner);
    }
    const lights = doc.getElementById("op-lights");
    if (lights) {
      const selected = model.lights.find(function (l) { return l.id === actions.selected; });
      render(html`<div class="lights" role="list">
        ${model.lights.map(function (l) {
          return html`<button type="button" role="listitem" class=${actions.selected === l.id ? "on" : ""} title=${l.detail} onClick=${function () { actions.select && actions.select(l.id); }}>
            <i class=${"dot " + l.level}></i><span>${l.label}</span>
          </button>`;
        })}
      </div>
      ${selected ? html`<p class="light-detail"><b>${selected.detail}</b> ${selected.fix || ""}</p>` : null}`, lights);
    }
    const steps = doc.getElementById("op-steps");
    if (steps) {
      render(html`<div class="steps-line" aria-label="ลำดับงาน Help Me Carry">
        <span class="k">ลำดับงาน</span>
        ${model.steps.map(function (s, i) {
          return html`<span class=${"step " + s.state}>${(i + 1) + " " + s.label}</span>`;
        })}
      </div>`, steps);
      const line = steps.querySelector(".steps-line");
      const now = line && line.querySelector(".step.now, .step.bad");
      if (now) line.scrollLeft = Math.max(0, now.offsetLeft - 24);
    }
    const list = doc.getElementById("op-events");
    if (list) {
      const items = actions.events || [];
      render(html`<div class="ph">เหตุการณ์<span>สาเหตุและวิธีแก้ ไม่หายเอง</span></div>
        ${items.length ? items.map(function (ev) {
          return html`<div class="ev">
            <div><time>${formatTime(ev.t)}</time><b>${ev.th}</b></div>
            ${ev.fix ? html`<span class="fix">${ev.fix}</span>` : null}
          </div>`;
        }) : html`<p class="note">ยังไม่มีเหตุการณ์ ข้อความเตือนจะค้างอยู่ที่นี่จนกว่าจะเริ่มงานรอบใหม่</p>`}`, list);
    }
    paintCameras(doc, model);
  }

  function renderPlain(doc, model, actions) {
    const banner = doc.getElementById("op-banner");
    if (banner) {
      banner.innerHTML = model.banner ? '<div class="banner ' + model.banner.level + '" role="status"><div class="txt"><strong></strong><p></p></div></div>' : "";
      if (model.banner) {
        banner.querySelector("strong").textContent = model.banner.title;
        banner.querySelector("p").textContent = model.banner.fix || "";
      }
    }
    const lights = doc.getElementById("op-lights");
    if (lights) {
      lights.innerHTML = '<div class="lights">' + model.lights.map(function (l) {
        return '<span><i class="dot ' + l.level + '"></i>' + escapeText(l.label) + "</span>";
      }).join("") + "</div>";
    }
    const steps = doc.getElementById("op-steps");
    if (steps) {
      steps.innerHTML = '<div class="steps-line">' + model.steps.map(function (s, i) {
        return '<span class="step ' + s.state + '">' + (i + 1) + " " + escapeText(s.label) + "</span>";
      }).join("") + "</div>";
    }
    paintCameras(doc, model);
  }

  function paintCameras(doc, model) {
    const arm = doc.getElementById("arm-reason");
    if (arm) {
      arm.hidden = !model.cameras.arm.reason;
      arm.textContent = model.cameras.arm.reason || "";
    }
    const jet = doc.getElementById("jetson-snap-reason");
    const src = doc.getElementById("jetson-snap-src");
    if (src) src.textContent = model.cameras.jetson.source;
    if (jet) {
      jet.hidden = !model.cameras.jetson.reason;
      jet.textContent = model.cameras.jetson.reason || "";
    }
  }

  function escapeText(s) {
    return String(s).replace(/[&<>]/g, function (c) { return { "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]; });
  }

  function bagPage(win) {
    const html = win.html;
    const Component = win.preact.Component;
    return class BagFields extends Component {
      constructor(props) {
        super(props);
        this.state = { pickX: "", pickY: "", dropX: "", dropY: "", pickDirty: false, dropDirty: false, bagAt: null, dropFilled: false };
      }
      componentDidMount() { this.take(this.props); }
      componentDidUpdate() { this.take(this.props); }
      take(props) {
        const bag = props.state && props.state.bag;
        const drop = props.state && props.state.drop;
        const at = bag && bag.at;
        const next = {};
        if (at && at !== this.state.bagAt && !this.state.pickDirty) {
          next.pickX = String(bag.x);
          next.pickY = String(bag.y);
          next.bagAt = at;
          if (!this.state.dropDirty) { next.dropX = String(bag.x); next.dropY = String(bag.y); }
        } else if ((!bag || this.state.pickDirty) && drop && drop.x != null && !this.state.dropDirty && !this.state.dropFilled && this.state.dropX === "") {
          next.dropX = String(drop.x);
          next.dropY = String(drop.y == null ? 0 : drop.y);
          next.dropFilled = true;
        }
        if (Object.keys(next).length) this.setState(next);
      }
      field(key, dirty) { return (ev) => this.setState({ [key]: ev.target.value, [dirty]: true }); }
      useDrop() {
        const drop = this.props.state && this.props.state.drop;
        if (!drop || drop.x == null) return;
        this.setState({ dropX: String(drop.x), dropY: String(drop.y == null ? 0 : drop.y), dropDirty: true });
      }
      send(kind) {
        const x = num(kind === "pick" ? this.state.pickX : this.state.dropX);
        const y = num(kind === "pick" ? this.state.pickY : this.state.dropY);
        if (x == null || y == null || !this.props.onArm) return;
        const bag = this.props.state && this.props.state.bag;
        const body = { cmd: kind === "pick" ? "pick_bag" : "drop_bag", x: x, y: y };
        if (kind === "pick" && bag && bag.x === x && bag.y === y) {
          if (typeof bag.top === "number") body.top = bag.top;
          if (typeof bag.yaw === "number") body.yaw = bag.yaw;
        }
        this.props.onArm(body);
      }
      render(props, st) {
        const state = props.state || {};
        const drop = state.drop || {};
        const bag = state.bag;
        const outside = function (x) {
          const n = num(x);
          return n != null && drop.x_min != null && (n < drop.x_min || n > drop.x_max);
        };
        const bagLine = bag ? bag.source + "  " + bag.x + ", " + bag.y : "ยังไม่มีตำแหน่งที่วัดได้";
        return html`<div class="robot-app">
          <div class="ph"><span>ถุง</span><span class="num">${bagLine}</span></div>
          <p class="note">หยิบและวางส่งตัวเลขในช่องนี้ ตำแหน่งที่วัดได้ไม่ถูกแทนด้วย 0.36</p>
          ${outside(st.pickX) || outside(st.dropX) ? html`<p class="note bad">ค่า x นี้อยู่นอกระยะที่ตรวจแล้ว ${drop.x_min}–${drop.x_max} ม. แต่จะส่งตามที่พิมพ์</p>` : null}
          <div class="robot-fields">
            <label>หยิบ x<input value=${st.pickX} onInput=${this.field("pickX", "pickDirty")} inputMode="decimal"></label>
            <label>หยิบ y<input value=${st.pickY} onInput=${this.field("pickY", "pickDirty")} inputMode="decimal"></label>
            <label>วาง x<input value=${st.dropX} onInput=${this.field("dropX", "dropDirty")} inputMode="decimal"></label>
            <label>วาง y<input value=${st.dropY} onInput=${this.field("dropY", "dropDirty")} inputMode="decimal"></label>
          </div>
          <div class="btnrow">
            <button type="button" class="btn pri" onClick=${() => this.send("pick")}>หยิบ</button>
            <button type="button" class="btn" onClick=${() => this.send("drop")}>วาง</button>
            <button type="button" class="btn" onClick=${() => this.useDrop()}>ใช้จุดวางที่ถึง</button>
          </div>
          <p class="note">${drop.why || drop.error || ""}</p>
        </div>`;
      }
    };
  }

  let bagView = null;
  function updateBag(props) {
    const win = typeof window !== "undefined" ? window : null;
    const doc = typeof document !== "undefined" ? document : null;
    if (!win || !doc || !win.preact || !win.html) return;
    const host = doc.getElementById("robot-root");
    if (!host) return;
    if (!bagView) bagView = bagPage(win);
    win.preact.render(win.html`<${bagView} state=${props.state} onArm=${props.onArm} />`, host);
  }

  loadEvents();
  return {
    STEPS: STEPS,
    readMission: readMission,
    update: updateBag,
    healthLights: healthLights,
    viewModel: viewModel,
    explainMessage: explainMessage,
    jetsonSnapURL: jetsonSnapURL,
    pushEvent: pushEvent,
    getEvents: getEvents,
    clearEvents: clearEvents,
    renderOperate: renderOperate,
    formatClock: formatClock,
    formatTime: formatTime,
  };
});

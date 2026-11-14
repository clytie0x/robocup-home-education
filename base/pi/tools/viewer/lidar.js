// Draws the LiDAR scan from /scan on every <canvas class="scan">, seen from above
// (forward is up, the robot's left is to the left, a ring every metre), fills the
// number tiles and keeps the camera and LiDAR badges honest.
const RANGE = 4.0; // m from the centre to the edge
const OUTLINE = [[0.2, 0.16], [0.16, 0.2], [-0.16, 0.2], [-0.2, 0.16],
                 [-0.2, -0.16], [-0.16, -0.2], [0.16, -0.2], [0.2, -0.16]];
const COLOUR = {
  back: "#fbfcfe", ring: "#e2e7ef", axis: "#eef1f6", label: "#8a93a5",
  robot: "#2563eb", robotFill: "rgba(37, 99, 235, 0.10)", point: "#d92d20",
};

function fitCanvas(canvas) {
  const size = Math.round(canvas.clientWidth * (window.devicePixelRatio || 1));
  if (size > 0 && canvas.width !== size) { canvas.width = size; canvas.height = size; }
}

function draw(canvas, d) {
  fitCanvas(canvas);
  const g = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height, s = w / (2 * RANGE);
  const k = w / 480; // scale marks with the canvas
  const px = (x, y) => [w / 2 - y * s, h / 2 - x * s];

  g.fillStyle = COLOUR.back;
  g.fillRect(0, 0, w, h);

  g.strokeStyle = COLOUR.axis;
  g.lineWidth = k;
  g.beginPath(); g.moveTo(w / 2, 0); g.lineTo(w / 2, h); g.moveTo(0, h / 2); g.lineTo(w, h / 2); g.stroke();

  g.strokeStyle = COLOUR.ring;
  g.fillStyle = COLOUR.label;
  g.font = `${Math.round(11 * k)}px "Segoe UI", system-ui, sans-serif`;
  for (let m = 1; m <= RANGE; m++) {
    g.beginPath(); g.arc(w / 2, h / 2, m * s, 0, 2 * Math.PI); g.stroke();
    g.fillText(`${m} m`, w / 2 + 4 * k, h / 2 - m * s - 4 * k);
  }

  const older = (d.older || []).slice().reverse();
  older.forEach((pts, i) => {
    g.fillStyle = `rgba(217, 45, 32, ${(0.12 + 0.35 * (i + 1) / (older.length + 1)).toFixed(2)})`;
    for (const p of pts) { const q = px(p[0], p[1]); g.fillRect(q[0] - k, q[1] - k, 2 * k, 2 * k); }
  });
  g.fillStyle = COLOUR.point;
  for (const p of d.points) {
    const q = px(p[0], p[1]);
    g.beginPath(); g.arc(q[0], q[1], 1.8 * k, 0, 2 * Math.PI); g.fill();
  }

  g.strokeStyle = COLOUR.robot;
  g.fillStyle = COLOUR.robotFill;
  g.lineWidth = 2 * k;
  g.beginPath();
  OUTLINE.forEach((p, i) => { const q = px(p[0], p[1]); i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]); });
  g.closePath(); g.fill(); g.stroke();
  const nose = px(0.35, 0);
  g.beginPath(); g.moveTo(w / 2, h / 2); g.lineTo(nose[0], nose[1]); g.stroke();
}

function setBadge(cls, state, text) {
  for (const el of document.querySelectorAll(cls)) {
    el.className = `badge ${cls.slice(1)} ${state}`;
    el.textContent = text;
  }
}

function setStat(id, html, level) {
  const el = document.getElementById(id);
  if (!el) return;
  el.querySelector(".value").innerHTML = html;
  el.className = `stat ${level || ""}`;
}

function show(d) {
  for (const canvas of document.querySelectorAll("canvas.scan")) draw(canvas, d);
  const near = d.nearest;
  setStat("stat-near", near == null ? "-" : `${near.toFixed(2)}<small>m</small>`,
          near == null ? "" : near < 0.5 ? "bad" : near < 1.0 ? "warn" : "ok");
  setStat("stat-points", `${d.points.length}`);
  setStat("stat-age", d.age == null ? "-" : `${d.age.toFixed(1)}<small>s</small>`,
          d.age == null ? "" : d.age < 1.0 ? "ok" : "warn");
  const fresh = d.age != null && d.age < 1.0;
  if (d.age == null) setBadge(".lidar-badge", "off", "No LiDAR");
  else setBadge(".lidar-badge", fresh ? "live" : "stale", fresh ? "Live" : "No new scans");
  for (const el of document.querySelectorAll(".scan-info")) {
    el.textContent = `${d.points.length} points, nearest ${near == null ? "-" : near.toFixed(2) + " m"}`;
  }
}

async function tick() {
  if (document.querySelector("canvas.scan, #stat-near")) {
    try {
      show(await (await fetch("/scan", { cache: "no-store" })).json());
    } catch (e) {
      setBadge(".lidar-badge", "off", "No LiDAR");
      for (const el of document.querySelectorAll(".scan-info")) el.textContent = "no LiDAR data";
    }
  }
  setTimeout(tick, 100);
}

// The MJPEG stream has no per-frame event, so each camera's badge says Live once a
// frame has arrived and Offline if the stream fails (then it retries every 5 s).
for (const img of document.querySelectorAll("img.camera")) {
  const badge = img.closest(".card").querySelector(".camera-badge");
  const mark = (state, text) => { badge.className = `badge camera-badge ${state}`; badge.textContent = text; };
  const src = img.getAttribute("src");
  img.addEventListener("error", () => {
    mark("off", "Offline");
    setTimeout(() => { img.src = `${src}?t=${Date.now()}`; check(); }, 5000);
  });
  const check = () => {
    if (img.naturalWidth > 0) mark("live", "Live");
    else setTimeout(check, 500);
  };
  check();
}

tick();

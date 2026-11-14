// Draws the LiDAR scan from /scan on every <canvas class="scan">, seen from above:
// forward is up, the robot's left is to the left, a ring every metre.
const RANGE = 4.0; // m from the centre to the edge
const OUTLINE = [[0.2, 0.16], [0.16, 0.2], [-0.16, 0.2], [-0.2, 0.16],
                 [-0.2, -0.16], [-0.16, -0.2], [0.16, -0.2], [0.2, -0.16]];

function draw(canvas, info, d) {
  const g = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height, s = w / (2 * RANGE);
  const px = (x, y) => [w / 2 - y * s, h / 2 - x * s];

  g.fillStyle = "#000";
  g.fillRect(0, 0, w, h);
  g.strokeStyle = "#262626";
  for (let m = 1; m <= RANGE; m++) {
    g.beginPath(); g.arc(w / 2, h / 2, m * s, 0, 2 * Math.PI); g.stroke();
  }

  g.strokeStyle = "#4af";
  g.beginPath();
  OUTLINE.forEach((p, i) => { const q = px(p[0], p[1]); i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]); });
  g.closePath(); g.stroke();
  const nose = px(0.35, 0);
  g.beginPath(); g.moveTo(w / 2, h / 2); g.lineTo(nose[0], nose[1]); g.stroke();

  const older = (d.older || []).slice().reverse();
  older.forEach((pts, i) => {
    g.fillStyle = `rgba(255, 68, 68, ${(0.15 + 0.5 * (i + 1) / (older.length + 1)).toFixed(2)})`;
    for (const p of pts) { const q = px(p[0], p[1]); g.fillRect(q[0] - 1, q[1] - 1, 2, 2); }
  });
  g.fillStyle = "#f44";
  for (const p of d.points) { const q = px(p[0], p[1]); g.fillRect(q[0] - 1, q[1] - 1, 3, 3); }

  if (info) {
    const near = d.nearest == null ? "-" : d.nearest.toFixed(2) + " m";
    const age = d.age == null ? "" : `, ${d.age.toFixed(1)} s old`;
    info.textContent = `${d.points.length} points, nearest ${near}${age}`;
  }
}

async function tick() {
  try {
    const d = await (await fetch("/scan", { cache: "no-store" })).json();
    for (const canvas of document.querySelectorAll("canvas.scan")) {
      draw(canvas, document.getElementById(canvas.dataset.info), d);
    }
  } catch (e) {
    for (const el of document.querySelectorAll(".scan-info")) el.textContent = "no LiDAR data";
  }
  setTimeout(tick, 100);
}

tick();

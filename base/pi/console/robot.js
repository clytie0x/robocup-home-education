/* Robot page. Preact + htm come from /preact-htm.js (one local file, no CDN).
   The camera, webcam, snapshot and LiDAR stay in the HTML outside this tree, so a 1s redraw
   does not restart those streams. */
"use strict";
(function () {
  const { h, render, Component } = window.preact;
  const html = window.htm.bind(h);
  const EN = {
    pi: "Pi", lidar: "LiDAR", camera: "Xtion", wheels: "Wheels", link: "Link",
    services: "Services", webcam: "Webcam",
  };

  function num(v) {
    const n = Number(v);
    return Number.isFinite(n) ? n : null;
  }

  function when(t) {
    if (!t) return "";
    const d = new Date(t * 1000);
    const p = (n) => String(n).padStart(2, "0");
    return p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds());
  }

  function merge(server, local) {
    const seen = new Set();
    const out = [];
    for (const e of (server || []).concat(local || [])) {
      if (!e || !e.text) continue;
      const key = String(e.time) + "|" + e.text;
      if (seen.has(key)) continue;
      seen.add(key);
      out.push(e);
    }
    out.sort((a, b) => (b.time || 0) - (a.time || 0));
    return out.slice(0, 80);
  }

  function statusCards(state) {
    const out = [];
    const bank = state.bank;
    if (bank && bank.telemetry !== false) {
      const pct = bank.percent == null ? "-" : Math.round(bank.percent) + "%";
      const motor = bank.motor && bank.motor.wh_used != null ? bank.motor.wh_used.toFixed(1) + " Wh wheels" : "power bank estimate";
      out.push({ id: "bank", title: "Battery", level: bank.warning ? "bad" : "ok", value: pct, text: motor });
    }
    for (const c of state.cards || []) {
      out.push({ id: c.id, title: EN[c.id] || c.title, level: c.level, value: c.value || "-", text: c.text || "" });
    }
    const arm = state.jetson && state.jetson.arm;
    const s = arm && arm.state;
    out.push({
      id: "arm", title: "Arm",
      level: s && s.fault ? "bad" : arm && arm.error ? "warn" : "ok",
      value: s ? (s.voltage_min != null ? Number(s.voltage_min).toFixed(1) + " V" : s.state) : "-",
      text: (s && s.fault ? "fault " + s.fault : "") || (arm && arm.error) || (s && s.last_message) || "no arm status yet",
    });
    const agent = state.jetson && state.jetson.agent;
    const programs = agent && agent.programs;
    if (programs) {
      for (const name of agent.order || Object.keys(programs)) {
        const p = programs[name];
        out.push({
          id: "p-" + name, title: name,
          level: p && p.running ? "ok" : p && p.code ? "bad" : "idle",
          value: p && p.running ? "running" : "stopped",
          text: p && p.pid ? "pid " + p.pid : "",
        });
      }
    } else if (state.jetson) {
      out.push({ id: "jetson", title: "Jetson", level: "warn", value: "no agent", text: state.jetson.agent_error || "" });
    }
    return out;
  }

  class RobotPage extends Component {
    constructor(props) {
      super(props);
      this.state = {
        pickX: "", pickY: "", dropX: "", dropY: "",
        pickDirty: false, dropDirty: false, bagAt: null, dropFilled: false,
      };
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
        if (!this.state.dropDirty) {
          next.dropX = String(bag.x);
          next.dropY = String(bag.y);
        }
      } else if ((!bag || this.state.pickDirty) && drop && drop.x != null && !this.state.dropDirty && !this.state.dropFilled && this.state.dropX === "") {
        next.dropX = String(drop.x);
        next.dropY = String(drop.y == null ? 0 : drop.y);
        next.dropFilled = true;
      }
      if (Object.keys(next).length) this.setState(next);
    }

    field(key, dirty) {
      return (ev) => this.setState({ [key]: ev.target.value, [dirty]: true });
    }

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
      const body = { cmd: kind === "pick" ? "pick_bag" : "drop_bag", x, y };
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
      const outside = (x) => {
        const n = num(x);
        return n != null && drop.x_min != null && (n < drop.x_min || n > drop.x_max);
      };
      const events = merge(state.events, props.localEvents);
      const bagLine = bag ? bag.source + "  " + bag.x + ", " + bag.y : "no measurement yet";
      return html`<div class="robot-app">
        <button type="button" class="robot-stop" onClick=${() => props.onStop && props.onStop()}>STOP</button>
        <div class="cards">${statusCards(state).map((c) => html`<div class="card ${c.level}" key=${c.id}>
          <span>${c.title}</span><b>${c.value}</b><small>${c.text}</small>
        </div>`)}</div>
        <div class="panel">
          <div class="ph"><span>Bag</span><span class="num">${bagLine}</span></div>
          <p class="note">Pick and drop send the numbers in these fields. A measured position is never replaced with 0.36.</p>
          ${outside(st.pickX) || outside(st.dropX) ? html`<p class="note bad">This x is outside the checked reach ${drop.x_min}–${drop.x_max} m. It is still sent as typed.</p>` : null}
          <div class="robot-fields">
            <label>Pick x<input value=${st.pickX} onInput=${this.field("pickX", "pickDirty")} inputMode="decimal"></label>
            <label>Pick y<input value=${st.pickY} onInput=${this.field("pickY", "pickDirty")} inputMode="decimal"></label>
            <label>Drop x<input value=${st.dropX} onInput=${this.field("dropX", "dropDirty")} inputMode="decimal"></label>
            <label>Drop y<input value=${st.dropY} onInput=${this.field("dropY", "dropDirty")} inputMode="decimal"></label>
          </div>
          <div class="btnrow">
            <button type="button" class="btn pri" onClick=${() => this.send("pick")}>Pick</button>
            <button type="button" class="btn" onClick=${() => this.send("drop")}>Drop</button>
            <button type="button" class="btn" onClick=${() => this.useDrop()}>Use reachable drop</button>
          </div>
          <p class="note">${drop.why || drop.error || ""}</p>
        </div>
        <div class="panel">
          <div class="ph"><span>Events</span><span class="num">${String(events.length)}</span></div>
          <div class="events">${events.length
            ? events.map((e) => html`<div key=${String(e.time) + e.text}><span class="num">${when(e.time)}</span><span class=${e.level === "error" ? "bad" : ""}>${e.text}</span></div>`)
            : html`<p class="note">No events yet.</p>`}</div>
        </div>
      </div>`;
    }
  }

  let root = null;
  window.HmcRobot = {
    update(props) {
      root = root || document.getElementById("robot-root");
      if (!root || !window.preact || !window.htm) return;
      render(html`<${RobotPage} state=${props.state} localEvents=${props.localEvents} onStop=${props.onStop} onArm=${props.onArm} />`, root);
    },
  };
})();

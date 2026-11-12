#!/usr/bin/env python3
"""Robot Voice control panel: a small web page with buttons to start / stop the voice system.

Run on the Jetson:   python3 ~/robot_voice/scripts/control_panel.py
Open in a browser:   http://<jetson-ip>:8090      (PC or phone on the same network)

Only Python's standard library is used. The page can only run the fixed commands below
(servers.sh, ros2 launch voice_assistant ..., the arm team's so100_moveit_config arm.launch.py,
fake_arm_server, publish a sentence); the options
are checked against a whitelist. Anyone on the same network can press the buttons.
"""
import json
import os
import re
import signal
import socket
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
SERVERS = HERE / 'servers.sh'
LOG_DIR = Path(os.environ.get('ROBOT_VOICE_LOG_DIR', '/tmp'))
VOICE_LOG = LOG_DIR / 'robot_voice_launch.log'
FAKE_LOG = LOG_DIR / 'robot_voice_fake_arm.log'
ARM_LOG = LOG_DIR / 'robot_voice_arm.log'
ARM_HW = {'real', 'sim'}                 # so100_moveit_config arm.launch.py hardware:=
ARM_MATCH = 'so100_moveit_config arm.launch.py'
PORT = int(os.environ.get('PANEL_PORT', '8090'))
ROS_SETUP = ('for f in /opt/ros/jazzy/setup.bash "$HOME/so100_ws/install/setup.bash" "$HOME/ros2_ws/install/setup.bash"; do '
             '[ -f "$f" ] && source "$f"; done; export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"; '
             'export PYTHONUNBUFFERED=1; ')
ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')

BRAINS = {'rules', 'llm'}
TARGETS = {                      # where the commands go
    'arm_actions': 'use_bridge:=true arm:=actions',    # real robot, SO-100 API /arm/pick, /arm/place
    'arm_topics': 'use_bridge:=true arm:=topics',      # real robot, old /robot/carry_bag topics
    'sim': 'use_bridge:=false',                        # simulator on the VM runs the bridge
}


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=0.3):
            return True
    except OSError:
        return False


def llama_state():
    try:
        with urllib.request.urlopen('http://127.0.0.1:8081/health', timeout=0.5) as r:
            return 'ready' if r.status == 200 else 'loading'
    except urllib.error.HTTPError:
        return 'loading'
    except Exception:  # noqa: BLE001
        return 'off'


def ram():
    try:
        info = dict(l.split(':', 1) for l in open('/proc/meminfo'))
        total = int(info['MemTotal'].split()[0]) / 1048576
        avail = int(info['MemAvailable'].split()[0]) / 1048576
        return {'used': round(total - avail, 1), 'total': round(total, 1)}
    except Exception:  # noqa: BLE001
        return {'used': 0, 'total': 0}


def tail(path, lines=200, max_bytes=80000):
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            text = f.read().decode('utf-8', 'replace')
    except OSError:
        return []
    return [ANSI.sub('', l) for l in text.splitlines()[-lines:]]


class Panel:
    def __init__(self):
        self.lock = threading.Lock()
        self.voice = None
        self.voice_cfg = None
        self.fake = None
        self.arm = None
        self.arm_hw = None
        self.job = ''            # what the panel is doing right now ('' = idle)
        self.message = ''        # result of the last action

    # ---------------- processes ----------------
    def _spawn(self, cmd, log):
        f = open(log, 'wb', buffering=0)
        f.write(f'===== {time.strftime("%F %T")}  $ {cmd}\n'.encode())
        return subprocess.Popen(['bash', '-c', ROS_SETUP + 'exec ' + cmd], stdout=f, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True)

    @staticmethod
    def _stop_proc(proc):
        if proc is None or proc.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(proc.pid, sig)
                proc.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return

    @staticmethod
    def _sh(args, timeout=100):
        try:
            r = subprocess.run(args, capture_output=True, text=True, timeout=timeout, start_new_session=True)
            return (r.stdout + r.stderr).strip()
        except subprocess.TimeoutExpired:
            return 'timeout'
        except OSError as e:
            return f'error: {e}'

    def voice_running(self):
        return self.voice is not None and self.voice.poll() is None

    def _stop_voice(self):
        self._stop_proc(self.voice)
        self.voice = None
        # a launch started by hand in a terminal (not by this panel)
        subprocess.run(['pkill', '-INT', '-f', 'voice_assistant.launch.py'], capture_output=True)

    def _run_job(self, title, fn):
        with self.lock:
            if self.job:
                return False
            self.job = title

        def work():
            try:
                self.message = fn() or ''
            except Exception as e:  # noqa: BLE001
                self.message = f'ผิดพลาด: {e}'
            finally:
                self.job = ''
        threading.Thread(target=work, daemon=True).start()
        return True

    # ---------------- actions ----------------
    def start(self, brain, target, mic):
        if brain not in BRAINS or target not in TARGETS:
            return False

        def fn():
            self._stop_voice()
            if brain == 'llm':
                out = self._sh([str(SERVERS), 'start'])
            else:
                subprocess.run(['pkill', '-f', 'llama-server'], capture_output=True)   # free ~1.4 GB RAM
                out = self._sh([str(SERVERS), 'start-stt'])
            if 'FAILED' in out or out.startswith(('error', 'timeout')):
                return f'เปิด server ไม่สำเร็จ: {out[-300:]}'
            cmd = (f'ros2 launch voice_assistant voice_assistant.launch.py brain:={brain} '
                   f'{TARGETS[target]} use_mic:={"true" if mic else "false"}')
            self.voice = self._spawn(cmd, VOICE_LOG)
            self.voice_cfg = {'brain': brain, 'target': target, 'mic': bool(mic)}
            time.sleep(2.5)
            if not self.voice_running():
                return 'ระบบเสียงเปิดแล้วปิดทันที ดู log ด้านล่าง'
            return 'เปิดระบบเสียงแล้ว'
        return self._run_job('กำลังเปิดระบบ' + (' (โหลด LLM อาจใช้เวลาถึง 1 นาที)' if brain == 'llm' else ''), fn)

    def stop_voice(self):
        def fn():
            self._stop_voice()
            return 'หยุดระบบเสียงแล้ว (server ยังเปิดอยู่)'
        return self._run_job('กำลังหยุดระบบเสียง', fn)

    def stop_all(self):
        def fn():
            self._stop_voice()
            self._stop_proc(self.fake)
            self.fake = None
            self._stop_arm()
            self._sh([str(SERVERS), 'stop'], timeout=20)
            return 'ปิดทุกอย่างแล้ว (รวมแขน)'
        return self._run_job('กำลังปิดทุกอย่าง', fn)

    def fake_arm(self, on):
        def fn():
            self._stop_proc(self.fake)
            self.fake = None
            if on:
                if self.arm_running() or self.arm_external():
                    return 'ปิดแขนก่อน ห้ามเปิด fake_arm_server พร้อมแขน'
                self.fake = self._spawn('ros2 run voice_assistant fake_arm_server', FAKE_LOG)
                return 'เปิดแขนจำลองแล้ว (ห้ามเปิดพร้อมแขนจริง)'
            return 'ปิดแขนจำลองแล้ว'
        return self._run_job('แขนจำลอง', fn)

    def arm_running(self):
        return self.arm is not None and self.arm.poll() is None

    @staticmethod
    def arm_external():
        return subprocess.run(['pgrep', '-f', ARM_MATCH], capture_output=True).returncode == 0

    def _start_arm(self, hw):
        self._stop_arm()
        self._stop_proc(self.fake)              # never two arms answering /arm/pick
        self.fake = None
        self.arm = self._spawn(f'ros2 launch so100_moveit_config arm.launch.py hardware:={hw} rviz:=false', ARM_LOG)
        self.arm_hw = hw
        deadline = time.time() + 45             # wait until /arm/pick exists (MoveIt takes a while)
        while time.time() < deadline:
            time.sleep(2.0)
            if not self.arm_running():
                return False
            if self._arm_ready():
                return True
        return self.arm_running()

    def _arm_ready(self):
        out = self._sh(['bash', '-c', ROS_SETUP + 'timeout 4 ros2 action list'], timeout=8)
        return '/arm/pick' in out

    def _stop_arm(self):
        self._stop_proc(self.arm)
        self.arm = None
        subprocess.run(['pkill', '-INT', '-f', ARM_MATCH], capture_output=True)   # started by hand / Start-SO100

    def arm_on(self, hw):
        if hw not in ARM_HW:
            return False

        def fn():
            if not self._start_arm(hw):
                return 'เปิดแขนไม่สำเร็จ ดู log แขน'
            return 'เปิดแขน' + ('จริงแล้ว' if hw == 'real' else 'จำลอง (hardware:=sim) แล้ว')
        return self._run_job('กำลังเปิดแขน (MoveIt ใช้เวลาสักครู่)', fn)

    def arm_off(self):
        def fn():
            self._stop_arm()
            return 'ปิดแขนแล้ว'
        return self._run_job('กำลังปิดแขน', fn)

    def start_all(self, mic):
        """Competition button: real arm + voice (no LLM) + bridge with the arm API."""
        def fn():
            sim_now = self.arm_running() and self.arm_hw == 'sim'
            if sim_now or not (self.arm_running() or self.arm_external()):
                if not self._start_arm('real'):         # competition = real arm, never the simulated one
                    return 'เปิดแขนไม่สำเร็จ ดู log แขน (ระบบเสียงยังไม่ได้เปิด)'
            self._stop_voice()
            subprocess.run(['pkill', '-f', 'llama-server'], capture_output=True)
            out = self._sh([str(SERVERS), 'start-stt'])
            if 'FAILED' in out:
                return f'เปิด whisper ไม่สำเร็จ: {out[-300:]}'
            cmd = ('ros2 launch voice_assistant voice_assistant.launch.py brain:=rules '
                   f'{TARGETS["arm_actions"]} use_mic:={"true" if mic else "false"}')
            self.voice = self._spawn(cmd, VOICE_LOG)
            self.voice_cfg = {'brain': 'rules', 'target': 'arm_actions', 'mic': bool(mic)}
            time.sleep(2.5)
            return 'เปิดทั้งหมดแล้ว (แขนจริง + เสียงไม่ใช้ LLM)' if self.voice_running() else \
                'แขนเปิดแล้ว แต่ระบบเสียงปิดทันที ดู log'
        return self._run_job('กำลังเปิดทั้งหมด', fn)

    def say(self, text):
        """Same as speaking into the mic: publish one sentence on /speech/heard."""
        text = ' '.join(str(text).split())[:200]
        if not text:
            return False
        payload = '{data: ' + json.dumps(text) + '}'
        script = ROS_SETUP + 'exec ros2 topic pub --once /speech/heard std_msgs/msg/String "$1"'

        def work():
            subprocess.run(['bash', '-c', script, '_', payload], capture_output=True, timeout=15)
        threading.Thread(target=work, daemon=True).start()
        self.message = f'ส่งข้อความ: "{text}"'
        return True

    # ---------------- status ----------------
    def status(self):
        running = self.voice_running()
        external = False
        if not running:
            external = subprocess.run(['pgrep', '-f', 'voice_assistant.launch.py'], capture_output=True).returncode == 0
        flow, heard = '', ''
        if running:
            for line in reversed(tail(VOICE_LOG, 400)):
                if not flow:
                    m = re.search(r'== state: (\w+) ==', line)
                    if m:
                        flow = m.group(1)
                if not heard:
                    m = re.search(r'heard \([^)]*\): "(.*)"', line)
                    if m:
                        heard = m.group(1)
                if flow and heard:
                    break
        return {
            'whisper': port_open(8080), 'llama': llama_state(),
            'voice': running, 'external': external, 'cfg': self.voice_cfg if running else None,
            'fake_arm': self.fake is not None and self.fake.poll() is None,
            'arm': 'panel' if self.arm_running() else ('external' if self.arm_external() else 'off'),
            'arm_hw': self.arm_hw if self.arm_running() else None,
            'flow': flow, 'heard': heard, 'ram': ram(), 'job': self.job, 'message': self.message,
        }

    def shutdown(self):
        self._stop_proc(self.voice)
        self._stop_proc(self.fake)
        self._stop_proc(self.arm)


PANEL = Panel()

PAGE = r"""<!doctype html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Voice</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1c2330;--mute:#667085;--line:#d9dde5;--go:#1f7a4d;--go2:#165c3a;
--llm:#2b5fb8;--stop:#b3261e;--warn:#9a6700;--ok:#1f7a4d;--off:#98a2b3;--log:#10151d;--logink:#d6dde8}
@media (prefers-color-scheme:dark){:root{--bg:#12161d;--card:#1b212b;--ink:#e6eaf0;--mute:#9aa4b2;--line:#2c3542;
--go:#2f9e68;--go2:#27845a;--llm:#4f86e0;--stop:#e05b52;--warn:#d9a13a;--ok:#3fbf7f;--off:#667085;--log:#0b0f15}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.5 "Noto Sans Thai","Leelawadee UI",Tahoma,system-ui,sans-serif}
main{max-width:980px;margin:0 auto;padding:16px}
h1{font-size:22px;margin:4px 0 12px}
h2{font-size:15px;margin:0 0 10px;color:var(--mute);font-weight:600}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:14px}
.chips{display:flex;flex-wrap:wrap;gap:8px}
.chip{display:flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:999px;padding:5px 12px;font-size:14px}
.dot{width:10px;height:10px;border-radius:50%;background:var(--off)}
.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}
.state{font-size:26px;font-weight:700;margin:10px 0 2px}
.sub{color:var(--mute);font-size:14px;min-height:21px;overflow-wrap:anywhere}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media (max-width:640px){.grid{grid-template-columns:1fr}}
button{font:inherit;border:0;border-radius:10px;padding:12px 16px;cursor:pointer;color:#fff;background:var(--mute)}
button:disabled{opacity:.5;cursor:not-allowed}
button.big{width:100%;text-align:left;padding:16px 18px}
button.big b{display:block;font-size:19px}button.big span{font-size:14px;opacity:.92}
.go{background:var(--go)}.llm{background:var(--llm)}.stop{background:var(--stop)}
.ghost{background:transparent;color:var(--ink);border:1px solid var(--line)}
.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
label{font-size:15px}select,input[type=text]{font:inherit;padding:10px;border-radius:8px;border:1px solid var(--line);
background:var(--card);color:var(--ink)}
input[type=text]{flex:1;min-width:180px}
#msg{margin-top:10px;font-size:15px;min-height:24px}
#msg.busy{color:var(--warn)}
pre{background:var(--log);color:var(--logink);border-radius:10px;padding:12px;margin:0;height:340px;overflow:auto;
font:12.5px/1.45 ui-monospace,Consolas,monospace;white-space:pre-wrap;overflow-wrap:anywhere}
.hint{color:var(--mute);font-size:13px;margin-top:8px}
</style></head><body><main>
<h1>Robot Voice · แผงควบคุม</h1>

<div class="card">
  <div class="chips">
    <div class="chip"><span class="dot" id="d-voice"></span><span id="t-voice">ระบบเสียง</span></div>
    <div class="chip"><span class="dot" id="d-whisper"></span>ฟังเสียง (whisper)</div>
    <div class="chip"><span class="dot" id="d-llama"></span><span id="t-llama">LLM</span></div>
    <div class="chip"><span class="dot" id="d-arm"></span><span id="t-arm">แขน</span></div>
    <div class="chip" id="ram">RAM</div>
  </div>
  <div class="state" id="flow">ยังไม่ได้เปิด</div>
  <div class="sub" id="heard"></div>
  <div id="msg"></div>
</div>

<div class="card">
  <h2>วันแข่ง</h2>
  <button class="big go" id="b-all"><b>เปิดทั้งหมด</b><span>แขนจริง + ระบบเสียงไม่ใช้ LLM + สั่งแขนผ่าน /arm/pick</span></button>
</div>

<div class="card">
  <h2>แขน (so100_moveit_config, ไม่เปิด RViz)</h2>
  <div class="row">
    <select id="armhw">
      <option value="real">แขนจริง (hardware:=real)</option>
      <option value="sim">แขนจำลองของทีมแขน (hardware:=sim)</option>
    </select>
    <button class="go" id="b-armon">เปิดแขน</button>
    <button class="stop" id="b-armoff">ปิดแขน</button>
  </div>
  <div class="hint">ถ้าแขนถือของอยู่ ให้สั่งวางก่อนปิดแขน · แขนจำลองไม่ขยับมอเตอร์ แต่ใช้โค้ดหยิบวางจริงของทีมแขน</div>
</div>

<div class="card">
  <h2>ระบบเสียง</h2>
  <div class="grid">
    <button class="big go" id="b-rules"><b>ไม่ใช้ LLM</b><span>ตอบทันที ประหยัดแรม เข้าใจคำสั่งตามรูปประโยคที่กำหนด</span></button>
    <button class="big llm" id="b-llm"><b>ใช้ LLM</b><span>เข้าใจประโยคหลากหลาย ช้ากว่าราว 1 วินาที ใช้แรมเพิ่ม ~1.4 GB</span></button>
  </div>
  <div class="row" style="margin-top:14px">
    <label for="target">ส่งคำสั่งไปที่</label>
    <select id="target">
      <option value="arm_actions">หุ่นจริง + แขน API ใหม่ (/arm/pick)</option>
      <option value="arm_topics">หุ่นจริง + แขน topic แบบเดิม</option>
      <option value="sim">ซิมบน VM (ไม่เปิด bridge บน Jetson)</option>
    </select>
    <label><input type="checkbox" id="mic" checked> ใช้ไมค์</label>
  </div>
  <div class="row" style="margin-top:14px">
    <button class="stop" id="b-stopvoice">หยุดระบบเสียง</button>
    <button class="ghost" id="b-stopall">ปิดทุกอย่าง (รวมแขนและ server)</button>
  </div>
</div>

<div class="card">
  <h2>สั่งด้วยการพิมพ์ (เหมือนพูดใส่ไมค์)</h2>
  <div class="row">
    <input type="text" id="say" placeholder="เช่น Help me carry this bag" maxlength="200">
    <button class="go" id="b-say">ส่ง</button>
    <button class="stop" id="b-stoprobot">STOP หุ่น</button>
  </div>
  <div class="row" style="margin-top:10px" id="quick"></div>
</div>

<div class="card">
  <div class="row" style="justify-content:space-between;margin-bottom:10px">
    <h2 style="margin:0">Log</h2>
    <select id="logwhich"><option value="voice">ระบบเสียง</option><option value="arm">แขน</option></select>
  </div>
  <pre id="log">(ยังไม่มี log)</pre>
</div>
</main>
<script>
const $ = id => document.getElementById(id);
const NAMES = {listening:'กำลังฟัง (listening)', picking_bag:'กำลังหยิบของ', announce_ready:'พร้อมเดินตาม',
  following:'กำลังเดินตาม', dropping_bag:'กำลังวางของ', error:'มีข้อผิดพลาด'};
const QUICK = ['Help me carry this bag','Drop the bag','Follow me','Move forward one meter','Turn left ninety degrees','Do a beatbox'];
let busy = false;
function save(k,v){try{localStorage.setItem(k,v)}catch(e){}}
function load(k){try{return localStorage.getItem(k)}catch(e){return null}}
async function post(path, body){
  try{ await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})}); }
  catch(e){ $('msg').textContent = 'ติดต่อแผงควบคุมไม่ได้'; }
  refresh();
}
function start(brain){ post('/api/start',{brain, target:$('target').value, mic:$('mic').checked}); }
function say(t){ if(t.trim()) post('/api/say',{text:t}); }
function dot(id, cls){ $(id).className = 'dot' + (cls ? ' '+cls : ''); }
async function refresh(){
  let s;
  try{ s = await (await fetch('/api/status')).json(); }
  catch(e){ $('msg').textContent = 'ติดต่อแผงควบคุมไม่ได้ (control_panel.py ยังรันอยู่ไหม)'; return; }
  busy = !!s.job;
  dot('d-voice', s.voice ? 'ok' : (s.external ? 'warn' : ''));
  const c = s.cfg;
  $('t-voice').textContent = s.voice ? 'ระบบเสียง: ' + (c.brain==='llm' ? 'ใช้ LLM' : 'ไม่ใช้ LLM')
     : (s.external ? 'ระบบเสียง: เปิดจาก terminal' : 'ระบบเสียง: ปิด');
  dot('d-whisper', s.whisper ? 'ok' : '');
  dot('d-llama', s.llama==='ready' ? 'ok' : (s.llama==='loading' ? 'warn' : ''));
  $('t-llama').textContent = 'LLM' + (s.llama==='loading' ? ' (กำลังโหลด)' : '');
  dot('d-arm', s.arm==='off' ? (s.fake_arm ? 'warn' : '') : 'ok');
  $('t-arm').textContent = s.arm==='panel' ? 'แขน: ' + (s.arm_hw==='sim' ? 'จำลอง' : 'จริง')
     : s.arm==='external' ? 'แขน: เปิดจากที่อื่น' : (s.fake_arm ? 'แขน: fake_arm_server' : 'แขน: ปิด');
  $('ram').textContent = 'RAM ' + s.ram.used + ' / ' + s.ram.total + ' GB';
  $('flow').textContent = s.voice ? (NAMES[s.flow] || s.flow || 'กำลังเริ่ม...') : 'ยังไม่ได้เปิด';
  $('heard').textContent = s.voice && s.heard ? 'ได้ยินล่าสุด: "' + s.heard + '"' : '';
  $('msg').textContent = s.job ? s.job + ' ...' : s.message;
  $('msg').className = s.job ? 'busy' : '';
  for (const id of ['b-all','b-armon','b-armoff','b-rules','b-llm','b-stopvoice','b-stopall']) $(id).disabled = busy;
}
async function refreshLog(){
  try{
    const r = await (await fetch('/api/log?which=' + $('logwhich').value)).json();
    const el = $('log'), atEnd = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
    el.textContent = r.lines.length ? r.lines.join('\n') : '(ยังไม่มี log)';
    if (atEnd) el.scrollTop = el.scrollHeight;
  }catch(e){}
}
$('b-rules').onclick = () => start('rules');
$('b-llm').onclick = () => start('llm');
$('b-stopvoice').onclick = () => post('/api/stop_voice');
$('b-stopall').onclick = () => post('/api/stop_all');
$('b-all').onclick = () => post('/api/start_all',{mic:$('mic').checked});
$('b-armon').onclick = () => post('/api/arm',{on:true, hardware:$('armhw').value});
$('b-armoff').onclick = () => post('/api/arm',{on:false});
$('logwhich').onchange = () => { save('logwhich', $('logwhich').value); refreshLog(); };
$('b-say').onclick = () => { say($('say').value); $('say').value=''; };
$('say').onkeydown = e => { if(e.key==='Enter') $('b-say').onclick(); };
$('b-stoprobot').onclick = () => say('stop');
for (const q of QUICK){ const b=document.createElement('button'); b.className='ghost'; b.textContent=q; b.onclick=()=>say(q); $('quick').appendChild(b); }
$('target').value = load('target') || 'arm_actions';
$('mic').checked = load('mic') !== '0';
$('armhw').value = load('armhw') || 'real';
$('armhw').onchange = () => save('armhw', $('armhw').value);
$('logwhich').value = load('logwhich') || 'voice';
$('target').onchange = () => save('target', $('target').value);
$('mic').onchange = () => save('mic', $('mic').checked ? '1' : '0');
refresh(); refreshLog(); setInterval(refresh, 1500); setInterval(refreshLog, 2000);
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, body, ctype='application/json', code=200):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/':
            self._send(PAGE.encode(), 'text/html; charset=utf-8')
        elif self.path == '/api/status':
            self._send(PANEL.status())
        elif self.path.startswith('/api/log'):
            self._send({'lines': tail(ARM_LOG if 'which=arm' in self.path else VOICE_LOG)})
        else:
            self._send({'error': 'not found'}, code=404)

    def do_POST(self):
        try:
            n = min(int(self.headers.get('Content-Length') or 0), 4096)
            body = json.loads(self.rfile.read(n) or b'{}')
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            return self._send({'error': 'bad request'}, code=400)
        if self.path == '/api/start':
            ok = PANEL.start(body.get('brain'), body.get('target'), bool(body.get('mic', True)))
        elif self.path == '/api/stop_voice':
            ok = PANEL.stop_voice()
        elif self.path == '/api/stop_all':
            ok = PANEL.stop_all()
        elif self.path == '/api/fake_arm':
            ok = PANEL.fake_arm(bool(body.get('on')))
        elif self.path == '/api/arm':
            ok = PANEL.arm_on(body.get('hardware')) if body.get('on') else PANEL.arm_off()
        elif self.path == '/api/start_all':
            ok = PANEL.start_all(bool(body.get('mic', True)))
        elif self.path == '/api/say':
            ok = PANEL.say(body.get('text', ''))
        else:
            return self._send({'error': 'not found'}, code=404)
        self._send({'ok': bool(ok)}, code=200 if ok else 409)


def my_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
    except OSError:
        return '<jetson-ip>'


def main():
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f'Robot Voice control panel:  http://{my_ip()}:{PORT}   (Ctrl+C to quit)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print('stopping the voice system started from the panel...', flush=True)
        PANEL.shutdown()


if __name__ == '__main__':
    main()

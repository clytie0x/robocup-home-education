# ทดสอบ voice_assistant ด้วย simulator (turtlesim)

ภาพรวม
```
Jetson (192.168.31.137)                                              VM ros2-for-robot (192.168.31.241, มีหน้าจอ)
ไมค์ → stt_node → flow_node ⇄ rule_node/llm_node      turtlesim + command_bridge_node + turtlesim_bag_sim
                      │  └→ tts_node → ลำโพง              ▲              │
                      └──── /robot/command (DDS) ─────────┘              │
                      ◄──── /robot/event (grab_success, drop_done, see_person) ┘
```
- Jetson: ฟังเสียง, แปลคำสั่ง, state machine (flow_node), พูดตอบ (`use_bridge:=false`)
- VM: รับ `/robot/command` แล้วขยับเต่า + จำลองถุง + ส่ง event จำลองกลับ (`sim_turtlesim.launch.py`)
- ทางเลือก: ถ้า Jetson ต่อจอ รันทั้งสองฝั่งบน Jetson ได้เลย (ใส่ `DISPLAY=:0` หน้าคำสั่ง sim)

### สมอง 2 แบบ
| | `brain:=rules` (แนะนำ) | `brain:=llm` |
|---|---|---|
| ความเร็วแปลคำสั่ง | < 1 ms | 0.7–1.5 s |
| แรม | ไม่ต้องเปิด llama (ประหยัด ~1.4 GB) | ต้องเปิด llama-server |
| เข้าใจ | เฉพาะรูปประโยคที่กำหนด | ประโยคหลากหลาย คุยเล่นได้ |
| server | `servers.sh start-stt` | `servers.sh start` |

### Flow (flow_node) — ไม่มี wake word
แผนภาพเต็ม + ตาราง event สำหรับเพื่อน: ดูเอกสาร `robot-task-flow.md`
```
listening ("How can I help you?", ถามซ้ำทุก 10 วิ)
  ├─ "help me carry this bag" → picking bag → grab_success → announce ready ("I am ready to follow you")
  │                                                                → see person → following
  ├─ "follow me" → announce ready → following
  ├─ "drop the bag" [มีถุง] → drop bag → drop_done → listening ("Here is your bag")
  ├─ "drop the bag" [ไม่มีถุง] → error ("I don't have a bag") → listening
  └─ move / turn / beatbox / คุย → ทำ + ตอบ แล้วอยู่ที่ listening
"stop" ใช้ได้ทุก state · ทุก state ที่รอหุ่นมี timeout → error → listening
```

---

## 1. ตั้งค่าครั้งเดียว

**VM network → Bridged** (ROS 2 ข้ามเครื่องผ่าน NAT ไม่ได้)
1. ปิด VM → ทำ Snapshot
2. VirtualBox → Settings → Network → Adapter 1 → Attached to: **Bridged Adapter** → เลือกการ์ด Wi-Fi/LAN ที่ Windows ใช้
3. เปิด VM → `ip a` → ต้องได้ IP `192.168.31.x`
4. SSH เข้า VM: `ssh ros2@192.168.31.241` (port 2222 ใช้ไม่ได้แล้ว)

**ทั้ง Jetson และ VM**
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**ทดสอบว่าเห็นกัน**
- Jetson: `ros2 run demo_nodes_cpp talker`
- VM: `ros2 run demo_nodes_py listener` → ต้องขึ้น `I heard: Hello World`
- ไม่เห็นกัน: เช็ก Windows firewall / router บล็อก multicast / ROS_DOMAIN_ID ไม่ตรง

**Jetson: โมเดล whisper ที่แม่นกว่า (ครั้งเดียว)**
```bash
~/robot_voice/scripts/servers.sh get-small
```
หลังดาวน์โหลด `servers.sh` ใช้ small.en อัตโนมัติ (แม่นกับสำเนียงไทยกว่า base.en มาก)

## 2. ติดตั้ง/อัปเดตโค้ด (ทุกครั้งที่ได้ zip ใหม่)

🪟 Windows PowerShell (บรรทัดเดียว — **ห้ามรันใน SSH**):
```
scp D:\robot_voice.zip robotpsu@192.168.31.137:~/robot_voice_new.zip
scp D:\robot_voice.zip ros2@192.168.31.241:~/robot_voice_new.zip
```
ทั้งสองเครื่อง:
```bash
cd ~ && unzip -o robot_voice_new.zip && chmod +x ~/robot_voice/scripts/*.sh
mkdir -p ~/ros2_ws/src && cp -r ~/robot_voice/voice_assistant ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 pkg executables voice_assistant     # ต้องมี flow_node, rule_node, turtlesim_bag_sim
```
การอัปเดตไม่ทับค่าไมค์/ลำโพง เพราะเก็บแยกใน `~/.robot_voice_local.yaml` (ข้อ 3)

## 3. ไมค์/ลำโพงบน Jetson

ปกติ**ไม่ต้องตั้ง** — ใช้ค่าเริ่มต้นของเครื่อง ถ้าไมค์เปิดไม่ได้ log จะแสดงรายการไมค์ที่ใช้ได้ (เลข index) เอง

ถ้าต้องเลือกเอง ใส่ในไฟล์นอก package (โหลดหลัง params.yaml):
```bash
cat > ~/.robot_voice_local.yaml <<'EOF'
stt_node:
  ros__parameters:
    mic_device: "2"
tts_node:
  ros__parameters:
    speaker_device: "3"
EOF
```
ไม่ต้อง build ใหม่ แค่ launch ใหม่ · กลับไปใช้ค่าเริ่มต้น: `rm ~/.robot_voice_local.yaml`

เช็กไมค์+ลำโพง (อัด 3 วิแล้วเล่นกลับ):
```bash
arecord -f S16_LE -r 16000 -d 3 /tmp/t.wav && aplay /tmp/t.wav
```

## 4. รัน

**VM**:
```bash
DISPLAY=:0 ros2 launch voice_assistant sim_turtlesim.launch.py
```
**Jetson แบบไม่ใช้ LLM (แนะนำ)**:
```bash
~/robot_voice/scripts/servers.sh stop
~/robot_voice/scripts/servers.sh start-stt
ros2 launch voice_assistant voice_assistant.launch.py use_bridge:=false brain:=rules
```
**Jetson แบบ LLM**:
```bash
~/robot_voice/scripts/servers.sh start
ros2 launch voice_assistant voice_assistant.launch.py use_bridge:=false
```
รอ `microphone open: ...`, `flow ready`, `rule brain ready` (หรือ `LLM ready`), `TTS ready` แล้วหุ่นจะถาม "How can I help you?"

**server:** `servers.sh status` (ดูสถานะ + แรม) · `servers.sh stop` · `servers.sh restart`

ไม่มีไมค์/อยากพิมพ์แทนพูด: เพิ่ม `use_mic:=false` แล้วส่งประโยคจากหน้าต่างอื่น
```bash
hear()  { ros2 topic pub --once /speech/heard std_msgs/msg/String "{data: '$1'}"; }   # เหมือนพูดใส่ไมค์
event() { ros2 topic pub --once /robot/event  std_msgs/msg/String "{data: '$1'}"; }   # แกล้งเป็นหุ่นส่ง event
hear "Help me carry this bag"
event lost_person
```
ดูสถานะ: `ros2 topic echo /flow/state`

## 5. สคริปต์ทดสอบ (พูดตามลำดับ)

**ก. Flow (ซิมส่ง grab_success / drop_done / see_person ให้อัตโนมัติ)**
| # | พูด / ทำ | ควรได้ (`/flow/state`) |
|---|---|---|
| F1 | (เปิดระบบ แล้วเงียบ) | "How can I help you?" และถามซ้ำทุก 10 วิ |
| F2 | Drop the bag (ยังไม่มีถุง) | "I don't have a bag." → listening |
| F3 | Help me carry this bag | picking_bag → (1 วิ) announce_ready "I am ready to follow you" → following |
| F4 | Stop | หยุด → listening |
| F5 | Help me carry this bag → (following) Drop the bag | following → dropping_bag → "Here is your bag" → listening |
| F6 | Help me carry this bag → (ระหว่าง picking) Drop the bag | "Okay, cancelled." → listening |
| F7 | Follow me แล้วพิมพ์ `event lost_person` | "I lost you..." → listening |
| F8 | ปิดซิม person (`simulate_person:=false`) → Follow me | หลัง 5 วิ "I can't see you..." → listening |

**ข. คำสั่ง + ซิม**
| # | พูด | ผลบน turtlesim | log ที่ควรเห็น |
|---|---|---|---|
| 1 | Move forward one meter | เต่าเดินตรง แล้วหยุด | `action=move_forward` |
| 2 | Turn left ninety degrees | หมุนซ้าย 90° | `action=turn ... angle 90` |
| 3 | Help me carry this bag | เส้นเปลี่ยนเป็นเขียวหนา, หุ่นพูด "I am ready to follow you" | `carry_bag`, `event -> grab_success`, `state: following` |
| 4 | Stop | หยุด กลับ listening | `-> robot: stop` |
| 5 | Drop the bag | ถุง (เต่าตัวใหม่) + กล่องเหลือง ด้านหลังหุ่น, "Here is your bag" | `drop_bag`, `event -> drop_done` |
| 6 | Move forward one meter | เดินออก ถุงอยู่ที่เดิม | `move_forward` |
| 7 | Move forward two meters → Stop | หยุดทันที | `action=stop` |
| 8 | Do a beatbox | หุ่นพูด "Okay, check this out!" แล้วเล่นคลิปแมว | `playing ...cat_beatbox.wav` |
| 9 | Do not drop the bag | ไม่มีอะไรเกิด | ไม่ใช่ drop_bag |
| 10 | Do a beatbox → พูด "Stop!" ดัง ๆ ระหว่างเล่น | เสียงหยุดทันที | `BARGE-IN: "Stop!"`, tts `interrupted by the user` |

ดูใน log Jetson ทุกประโยค: `heard (0.xx s, level xN): "..."` = whisper ได้ยินอะไร + ดังกว่าเสียงพื้นหลังกี่เท่า
จด: ข้อที่ whisper ได้ยินผิด

**เคล็ดลับการพูด**
- ห่างไมค์ 20–50 ซม. พูดชัด ช้ากว่าปกตินิดหนึ่ง แล้วเงียบ ~1 วินาที (ระบบตัดประโยคเอง)
- รอหุ่นพูดตอบ/เล่นเสียงจบก่อนพูดประโยคถัดไป — ยกเว้น **"Stop"** พูดแทรกได้ (ต้องดังกว่าเสียงหุ่น 2 เท่า)
- ห้องเสียงพัดลม/แอร์ดัง → `vad_aggressiveness: 3`
- ระหว่าง following หุ่นรับแค่ "stop" กับ "drop the bag"

## 6. Topic ที่ดูได้ (หน้าต่างอื่นบนเครื่องไหนก็ได้)
```bash
ros2 topic echo /speech/heard                       # whisper ได้ยินอะไร (ทุกประโยค)
ros2 topic echo /speech/text                        # ประโยคที่ส่งให้สมอง
ros2 topic echo /flow/state                         # listening / picking_bag / following / ...
ros2 topic echo /robot/event                        # event จากหุ่น (ซิม)
ros2 topic echo /robot/command                      # คำสั่งที่ส่งไปหุ่น
ros2 topic echo /robot/state                        # สถานะหุ่น
ros2 topic echo /robot/carry_bag std_msgs/msg/Empty
ros2 topic echo /robot/drop_bag std_msgs/msg/Empty
ros2 topic echo /turtle1/pose                       # ตำแหน่งเต่า
```
รีเซ็ตเต่า: `ros2 service call /reset std_srvs/srv/Empty`

## 7. ปัญหาที่เจอบ่อย
| อาการ | แก้ |
|---|---|
| `Could not resolve hostname d` | รัน scp ใน SSH → ต้องรันใน Windows PowerShell |
| `ros2: command not found` | `source ~/.bashrc` (ดูข้อ 1) |
| `sim_turtlesim.launch.py was not found` | ยังเป็นโค้ดเก่า → ทำข้อ 2 ใหม่ |
| `Waiting for at least 1 matching subscription` | ฝั่ง launch ยังไม่ได้เปิด / ROS_DOMAIN_ID ไม่ตรง |
| `Connection refused` port 8080 | whisper ไม่รัน → `servers.sh start-stt` |
| `Connection refused` port 8081 | llama ไม่รัน → `servers.sh start` หรือใช้ `brain:=rules` |
| `invalid argument: --no-mmap` | ใช้ `servers.sh` แทน `start_servers.sh` |
| `cannot open microphone` + รายการไมค์ | ใส่เลขจากรายการใน `~/.robot_voice_local.yaml` หรือ `rm` ไฟล์นั้นเพื่อใช้ค่าเริ่มต้น |
| `PortAudioError: Invalid number of channels` | `mic_device` ชี้ไปที่ลำโพง/HDMI → แก้หรือลบ `~/.robot_voice_local.yaml` |
| `(dropped, too quiet ...)` ตอนคุณพูด | พูดใกล้ไมค์ขึ้น หรือลด `min_snr` เป็น 1.5 |
| เสียงคนอื่นในห้องยังถูกจับ | เพิ่ม `min_snr` เป็น 3 |
| `(dropped, single-word noise)` | whisper เดาคำเดี่ยวจากเสียงรบกวน — ปกติ ไม่ต้องแก้ |
| เต่าไม่ขยับแต่ Jetson log ปกติ | สองเครื่องไม่เห็นกัน → ทดสอบ talker/listener |
| turtlesim ไม่ขึ้นตอนรันผ่าน SSH | ใส่ `DISPLAY=:0` และต้องล็อกอินหน้าจอ VM ไว้ |
| ค้างที่ picking_bag / dropping_bag แล้ว error "could not ..." | ไม่มีใครส่ง `grab_success` / `drop_done` → เปิดซิมบน VM หรือส่ง event เอง |
| "I can't see you" ทุกครั้ง | ไม่มี `see_person` → เปิดซิม หรือตั้ง `require_person: false` |
| หุ่นถาม "How can I help you?" บ่อยเกินไป | เพิ่ม `reask_s` ใน `~/.robot_voice_local.yaml` ใต้ `flow_node:` |
| ไม่มีเสียงพูด/ติ๊ง | ลำโพงผิดตัว → ตั้ง `speaker_device` ใน `~/.robot_voice_local.yaml`, ทดสอบ `speaker-test -D plughw:<card>,0 -c 2 -t wav -l 1` |
| พูด "stop" ตอนหุ่นพูดแล้วไม่หยุด | ดู log `(while robot talks, ignored)` = ได้ยินแต่ไม่ใช่ stop · ไม่มีอะไรขึ้นเลย = เบาไป → พูดดังขึ้น/ใกล้ไมค์ หรือ `barge_ratio: 1.5` |
| หุ่นหยุดพูดเองโดยไม่มีใครพูด | `barge_ratio: 2.5` หรือ `barge_in: false` |
| หุ่นได้ยินเสียงตัวเองแล้ววน | หันลำโพงออกจากไมค์ / ลดเสียง / เพิ่ม `echo_guard_s` เป็น 1.5 |
| whisper ได้ยินผิด (เช่น "Kelly get back") | `servers.sh get-small` (ข้อ 1), พูดช้าลง/ใกล้ไมค์ขึ้น |

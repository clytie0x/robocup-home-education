# Robot Voice: STT → LLM → TTS บน Jetson Orin Nano + ROS 2 Jazzy

ระบบนี้เป็นแบบ offline 100% และเน้นภาษาอังกฤษ ใช้แรมรวมประมาณ 2–2.5 GB ซึ่งต่ำกว่าเพดาน 4 GB ที่ตั้งไว้

```
 Mic ─► [stt_node] ──/speech/text──► [llm_node] ──/tts/say──► [tts_node] ─► Speaker
         VAD + whisper.cpp            llama.cpp     │           Piper
           ▲                          (JSON schema) │             │
           └──────────── /tts/speaking (mute mic) ──┼─────────────┘
                                                    └─/robot/command (JSON)─► [command_bridge_node] ─► /cmd_vel
                                                    ◄─/robot/state───────────┘
```

## ติดตั้งจาก GitHub (Jetson)
```bash
cd ~ && git clone https://github.com/clytie0x/robocup-home-education.git
ln -sfn ~/robocup-home-education/voice ~/robot_voice          # สคริปต์/พาเนลอ้าง ~/robot_voice
ln -sfn ~/robocup-home-education/voice/voice_assistant ~/ros2_ws/src/voice_assistant
cd ~/ros2_ws && colcon build --symlink-install --packages-select voice_assistant && source install/setup.bash
~/robot_voice/scripts/install_jetson.sh      # ครั้งแรกเท่านั้น: build whisper.cpp/llama.cpp + โหลดโมเดล
```
- ถ้ามี `~/robot_voice` เดิมอยู่แล้ว ให้ย้ายออกก่อน (`mv ~/robot_voice ~/robot_voice.old`) และคัดลอก `sounds/cat_beatbox.wav` กลับมาเอง (ไม่อยู่ใน repo)
- โมเดลไม่อยู่ใน repo (อยู่ที่ `/opt/robot_voice`)
- `voice_rviz_sim/` ใช้บน VM เท่านั้น: `ln -sfn ~/robocup-home-education/voice/voice_rviz_sim ~/ros2_ws/src/`
- `arm:=actions` ต้องมี `so100_interfaces` ของทีมแขนใน workspace เดียวกัน
- อัปเดต: `cd ~/robocup-home-education && git pull` (Python ไม่ต้อง build ใหม่เพราะ `--symlink-install` ยกเว้นเพิ่มไฟล์ใหม่)

## งบแรม (Jetson ใช้ unified memory หมายความว่าหน่วยความจำของ GPU ก็คือแรมก้อนเดียวกัน)

| ส่วน | โมเดล | แรมโดยประมาณ |
|---|---|---|
| STT | whisper.cpp `base.en` q5_1 (CUDA) | ~0.3 GB |
| LLM | Qwen2.5-1.5B-Instruct Q4_K_M, ctx 2048 (CUDA) | ~1.4 GB |
| TTS | Piper `en_US-lessac-medium` (CPU) | ~0.1 GB |
| ROS 2 nodes ×4 | Python | ~0.3 GB |
| **รวม** | | **~2.1–2.5 GB** |

ถ้าอยากได้ LLM ที่ฉลาดขึ้น ยังมีที่เหลือพอสำหรับโมเดล 3B Q4_K_M เช่น Llama-3.2-3B-Instruct หรือ Qwen2.5-3B-Instruct ซึ่งใช้ราว 2.3 GB ทำให้ยอดรวมขึ้นไปอยู่ราว 3.2 GB ให้ตั้ง `LLM_MODEL=/path/model.gguf ./scripts/start_servers.sh`
ถ้าอยากให้ STT แม่นขึ้นก็เปลี่ยนเป็น `small.en-q5_1` ได้ ซึ่งใช้ราว 0.6 GB แต่จะช้าลง

ตรวจแรมจริงด้วยคำสั่ง `tegrastats` แล้วดูค่า `RAM xxxx/yyyyMB` ขณะระบบทำงานทุกส่วน
ถ้าต้องการแรมเพิ่ม ให้ปิด desktop GUI ซึ่งคืนแรมได้ประมาณ 0.8 GB: `sudo systemctl set-default multi-user.target`

## สิ่งที่ต้องมี
- **JetPack 7.2 (Ubuntu 24.04, CUDA 13.2)** ROS 2 Jazzy ต้องใช้ Ubuntu 24.04 จึงติดตั้งผ่าน apt ได้ตรงๆ
  ถ้าเครื่องยังเป็น JetPack 6.x (Ubuntu 22.04) ให้แฟลชเป็น 7.2 หรือรัน Jazzy ใน Docker แทน
- ROS 2 Jazzy (`ros-jazzy-ros-base`)
- ไมค์ USB + ลำโพง หรือ USB sound card

## ติดตั้ง
```bash
# 1) build whisper.cpp / llama.cpp ด้วย CUDA และโหลดโมเดล (ใช้เวลาราว 20-40 นาที)
./scripts/install_jetson.sh

# 2) start STT + LLM servers
./scripts/start_servers.sh

# 3) build ROS 2 package
mkdir -p ~/ros2_ws/src && cp -r voice_assistant ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source install/setup.bash

# 4) run
ros2 launch voice_assistant voice_assistant.launch.py
#    ไม่มีไมค์ / ทดสอบด้วยการพิมพ์:
ros2 launch voice_assistant voice_assistant.launch.py use_mic:=false
./scripts/test_text_only.sh
```

หาชื่อหรือ index ของไมค์และลำโพงด้วย `python3 -m sounddevice` แล้วใส่ค่าใน `config/params.yaml`
ไมค์ USB หลายรุ่นไม่รองรับ 16 kHz ถ้าเป็นแบบนั้นให้ตั้ง `mic_rate: 48000` เพราะโหนดจะ resample เป็น 16 kHz เอง

## โหมดสมอง: LLM หรือ กฎ (ไม่ใช้ LLM)
```bash
# LLM (เข้าใจประโยคหลากหลาย ~1 วินาที)
~/robot_voice/scripts/servers.sh start
ros2 launch voice_assistant voice_assistant.launch.py

# กฎ/keyword (ตอบทันที ประหยัดแรม ~1.4 GB แต่เข้าใจเฉพาะรูปประโยคที่กำหนด)
~/robot_voice/scripts/servers.sh start-stt
ros2 launch voice_assistant voice_assistant.launch.py brain:=rules

~/robot_voice/scripts/servers.sh stop     # ปิด server
~/robot_voice/scripts/servers.sh status   # ดูสถานะ + RAM
```

## ค่าเฉพาะเครื่อง (ไม่ถูกทับตอนอัปเดต)
ใส่เลขไมค์/ลำโพงใน `~/.robot_voice_local.yaml` (โหลดหลัง params.yaml ค่าในนี้ชนะ):
```yaml
stt_node:
  ros__parameters:
    mic_device: "2"
tts_node:
  ros__parameters:
    speaker_device: "3"
```

## ปรับความแม่นและเสียงรบกวน
- **โมเดล whisper ที่แม่นกว่า:** `~/robot_voice/scripts/servers.sh get-small` (ครั้งเดียว) แล้ว `servers.sh stop && servers.sh start-stt` — ใช้ small.en อัตโนมัติ
- **Volume gate:** log แต่ละประโยคมี `level xN` = ดังกว่าเสียงพื้นหลังกี่เท่า ประโยคที่ < `min_snr` (2.0) จะถูกทิ้ง `(dropped, too quiet)` ถ้าหุ่นไม่ได้ยินคุณ ลด `min_snr` เป็น 1.5 ถ้ายังจับเสียงคนอื่นเพิ่มเป็น 3
- **Chime:** ปลุกครั้งแรกพูดแนะนำตัว ครั้งต่อไปมีแค่เสียง "ติ๊ง" (`wake_chime`) และเสียงตอนกลับไปหลับ (`sleep_chime`)
- **ไมค์เปิดไม่ได้:** log จะแสดงรายการไมค์ที่ใช้ได้ (เลข index) และลองใหม่ทุก 5 วินาที

## พูดแทรกด้วย "stop" (barge-in)
ระหว่างหุ่นพูด/เล่นเสียง ถ้าพูด **"Stop"** ดัง ๆ หุ่นจะหยุดพูดทันทีและหยุดเคลื่อนที่ (log: `BARGE-IN: "Stop" -> interrupting the robot`)
- ต้องดังกว่าเสียงหุ่นที่ไมค์ได้ยิน `barge_ratio` เท่า (2.0) ถ้าพูดแล้วไม่หยุด → ลดเป็น 1.5 / ถ้าหยุดเองโดยไม่ได้พูด → เพิ่มเป็น 2.5
- รับเฉพาะ stop / halt / freeze · 0.3 วินาทีแรกที่หุ่นเริ่มพูดยังจับไม่ได้ (กำลังวัดเสียงหุ่น)
- ปิด: `barge_in: false`

## แผงควบคุม (หน้าเว็บ)
```bash
python3 ~/robot_voice/scripts/control_panel.py     # แล้วเปิด http://<IP Jetson>:8090 จากคอมหรือมือถือ
```
ปุ่มเปิดระบบแบบไม่ใช้ LLM / ใช้ LLM, หยุด, แขนจำลอง, พิมพ์คำสั่งแทนพูด, ดูสถานะและ log

## Topics

| Topic | Type | ทิศทาง | ความหมาย |
|---|---|---|---|
| `/speech/heard` | `std_msgs/String` | STT → dialog | ทุกประโยคที่ whisper ได้ยิน |
| `/speech/text` | `std_msgs/String` | dialog → LLM | ประโยคที่ส่งให้ LLM (หลัง wake word) |
| `/dialog/state` | `std_msgs/String` | dialog | `sleeping` / `awake` |
| `/tts/say` | `std_msgs/String` | LLM → TTS | ประโยคที่จะพูด (โหนดอื่นส่งมาให้หุ่นพูดได้ด้วย) |
| `/tts/play` | `std_msgs/String` | brain → TTS | path ของไฟล์ .wav ที่จะเล่น (เช่น beatbox) |
| `/tts/interrupt` | `std_msgs/Empty` | STT → TTS | ตัดเสียงหุ่นทันที (ผู้ใช้พูด stop แทรก) |
| `/tts/speaking` | `std_msgs/Bool` | TTS → STT | ปิดไมค์ขณะหุ่นพูด กันไม่ให้หุ่นได้ยินเสียงตัวเอง |
| `/robot/intent` | `std_msgs/String` (JSON) | rules/LLM → flow | สิ่งที่ผู้ใช้ต้องการ `{action, params, reply}` |
| `/robot/event` | `std_msgs/String` | หุ่น (โหนดเพื่อน) → flow | `grab_success`, `grab_failed`, `drop_done`, `drop_failed`, `see_person`, `lost_person`, `bag_lost` |
| `/flow/state` | `std_msgs/String` | flow | state ปัจจุบัน |
| `/robot/command` | `std_msgs/String` (JSON) | LLM → หุ่นยนต์ | **จุดต่อกับระบบ ROS 2 ของคุณ** |
| `/robot/state` | `std_msgs/String` | หุ่นยนต์ → LLM | สถานะหุ่นที่ใส่เข้าไปใน prompt |
| `/robot/drop_bag` | `std_msgs/Empty` | bridge → gripper | ส่งเมื่อสั่ง "drop the bag" (หุ่นหยุดก่อนแล้วค่อยส่ง) |
| `/robot/carry_bag` | `std_msgs/Empty` | bridge → gripper | ส่งเมื่อสั่ง "carry this bag" (หุ่นหยุดก่อนแล้วค่อยส่ง) |
| `/llm/response` | `std_msgs/String` (JSON) | debug | ผลลัพธ์ดิบจาก LLM |

ตัวอย่าง `/robot/command`:
```json
{"action": "move_forward", "params": {"distance": 1.0, "speed": 0.2}, "utterance": "move forward one meter", "stamp": 1730000000.0}
```

LLM ถูกบังคับด้วย JSON schema ผ่าน grammar ของ llama.cpp จึงตอบเป็น JSON ที่ parse ได้เสมอ
ค่า `action` จะเป็นค่าจาก `allowed_actions` เท่านั้น จะแก้รายการนี้ใน `params.yaml` ก็ได้

## ต่อเข้ากับหุ่นยนต์ของคุณ
`command_bridge_node.py` เป็นตัวอย่างที่แปลงคำสั่ง move/turn/stop เป็น `/cmd_vel` แบบตั้งเวลา (open-loop) โดยจำกัดความเร็วและระยะไว้แล้ว
ส่วน `go_to`, `follow_me` และ `look_at` เว้นเป็น TODO ไว้ให้ต่อกับ Nav2 (`NavigateToPose`) หรือโหนดอื่นของคุณ
ถ้าเพิ่ม action ใหม่ ให้เพิ่มทั้งใน `allowed_actions` และใน `_dispatch()`

## เวลาตอบสนองโดยประมาณบน Orin Nano 8GB
- whisper base.en: ~0.3–0.6 s ต่อประโยค
- LLM 1.5B: ~25–35 token/s ทำให้ได้คำตอบสั้นในราว 1–2 s
- Piper: เร็วกว่า real-time บน CPU

## ปัญหาที่พบบ่อย
- **หุ่นได้ยินเสียงตัวเอง:** เพิ่มค่า unmute delay ใน `stt_node.py` (ค่าปัจจุบัน 0.4 s) หรือใช้ไมค์ที่มี echo cancellation
- **whisper ถอดเสียงมั่วตอนเงียบ:** เพิ่ม `vad_aggressiveness` เป็น 3 หรือเพิ่ม `min_speech_ms`
- **อยากให้ตอบเฉพาะตอนถูกเรียก:** ตั้ง `wake_word: "robot"`
- **แรมเกิน:** ลด `-c` ใน `start_servers.sh`, ใช้ `tiny.en` หรือใช้ LLM ขนาด 1B

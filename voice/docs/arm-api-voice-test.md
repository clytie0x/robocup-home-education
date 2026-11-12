# ทดสอบระบบเสียงกับ SO-100 Arm API (`/arm/pick`, `/arm/place`)

ระบบเสียงสลับได้ 2 แบบด้วย launch arg `arm:=`
| `arm:=topics` (ค่าเริ่มต้น เหมือนเดิม) | `arm:=actions` (ใหม่) |
|---|---|
| `/robot/carry_bag`, `/robot/drop_bag` → `/robot/event` | `arm_action_bridge` → `/arm/pick`, `/arm/place` (action) |
| ตำแหน่งของจากกล่องฟ้าใน RViz | จุดทดสอบตายตัวใน params หรือ `PoseStamped` บน `/object/pose` (กล้อง) |
| ล้มเหลว → "Sorry, I could not pick up the bag." | ล้มเหลว → หุ่นพูดข้อความของแขน เช่น "Sorry. I can't reach it." |

flow ไม่เปลี่ยน: bridge แปลงผล action เป็น event เดิม (`grab_success`, `grab_failed|CODE|message`, `drop_done`, `drop_failed|CODE|message`)
· `HOLDING` ตอนหยิบ = ถือว่าถืออยู่แล้ว → ไปต่อ · `STOPPED` (ผู้ใช้พูด stop) = ไม่พูด error ซ้ำ
· "stop" เรียก `/arm/stop` เสมอ

## 0. ต้องมีก่อน (ครั้งเดียว)
1. ขอโฟลเดอร์ **`so100_interfaces`** จากทีมแขน (`~/so100_ws/src/so100_interfaces` มีแค่ไฟล์ .action/.msg)
   **ต้องเป็นเวอร์ชันเดียวกับที่แขนใช้** ไม่งั้นต่อกันไม่ได้ (type ไม่ตรง)
2. Jetson กับเครื่องแขนใช้ ROS 2 รุ่นเดียวกัน (Jazzy) และ `ROS_DOMAIN_ID=42`

```bash
# Jetson
cp -r ~/so100_interfaces ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install --packages-select so100_interfaces voice_assistant
source install/setup.bash
ros2 interface show so100_interfaces/action/Pick     # ต้องแสดง object_pose, object_size, ...
```

## 1. ทดสอบกับแขนจำลอง (ยังไม่ต่อแขนจริง)
**ห้ามเปิดพร้อมแขนจริง** (ตอบ 2 ตัว)
```bash
# หน้าต่าง 1: แขนจำลอง
ros2 run voice_assistant fake_arm_server
# หน้าต่าง 2: ระบบเสียง
~/robot_voice/scripts/servers.sh start-stt
ros2 launch voice_assistant voice_assistant.launch.py brain:=rules arm:=actions
```
ต้องเห็น `arm action bridge ready -> /arm/pick, /arm/place` และ `FAKE arm ready`

| # | พูด | ควรได้ |
|---|---|---|
| A1 | Help me carry this bag | "Okay, picking up the bag." → แขนจำลองขึ้น step 1/6…6/6 (4 วิ) → "I am ready to follow you" |
| A2 | Drop the bag | "Okay, dropping the bag." → step (3 วิ) → "Here is your bag." |
| A3 | Drop the bag (ไม่ได้ถือ) | "I don't have a bag." (flow รู้เอง ไม่ส่งไปแขน) |
| A4 | Help me carry this bag → ระหว่างหยิบพูด "Stop" | แขนจำลองขึ้น `/arm/stop called` → หยุด ไม่พูด error |

ทดสอบความล้มเหลว (รีสตาร์ทแขนจำลองพร้อม parameter):
```bash
ros2 run voice_assistant fake_arm_server --ros-args -p pick_result:=UNREACHABLE   # → "Sorry. I can't reach it."
ros2 run voice_assistant fake_arm_server --ros-args -p pick_result:=MISSED        # → "Sorry. I missed it..."
ros2 run voice_assistant fake_arm_server --ros-args -p place_result:=COLLISION    # → "Sorry. Something is in my way."
ros2 run voice_assistant fake_arm_server --ros-args -p pick_seconds:=40.0          # → timeout 30 วิ → "Sorry, I could not pick up the bag."
```
ไม่ต้องรีสตาร์ทระบบเสียง

## 2. ทดสอบกับแขนจริง
1. เครื่องแขน: `Start-SO100.cmd` (โหมด MoveIt) → `ros2 topic echo /arm/state` ต้องขึ้น `idle`
2. **ปิดแขนจำลองและ event จำลองทั้งหมด** (`fake_arm_server`, `turtlesim_bag_sim`, `voice_rviz_sim ... simulate_events:=false`)
3. วางของ (≤ 6.5 ซม. กว้าง) ที่**จุดทดสอบ**: หน้าแกนฐานแขน **20 ซม.** ตรงกลาง (ค่าเริ่มต้น)
4. Jetson: `ros2 launch voice_assistant voice_assistant.launch.py brain:=rules arm:=actions`
5. พูด "Help me carry this bag" → แขนหยิบ (~12–18 วิ) → "I am ready to follow you" → "Drop the bag" → แขนวางที่ หน้า 20 ซม. / ขวา 10 ซม. (~8–12 วิ)

**ย้ายจุดหยิบ/วาง** — ใน `~/.robot_voice_local.yaml` (ไม่ต้อง build):
```yaml
arm_action_bridge:
  ros__parameters:
    pick_forward: 0.25        # m หน้าแกนฐาน
    pick_left: 0.05           # m ทางซ้ายของแขน (ลบ = ขวา)
    object_size: [0.04, 0.03, 0.06]   # ลึก กว้าง สูง (m)
    place_forward: 0.20
    place_left: -0.10
```
**ส่งตำแหน่งเองแทนจุดตายตัว** (เหมือนที่กล้องจะส่ง ใช้ได้ภายใน 5 วิ):
```bash
ros2 topic pub --once /object/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base}, pose: {position: {x: 0.05, y: -0.2452}}}"
```
(frame `base`: หน้า = −y, ซ้าย = +x, แกนฐานอยู่ที่ y = −0.0452 → หน้า 20 ซม. ซ้าย 5 ซม. = x 0.05, y −0.2452)

## ปัญหา
| อาการ | แก้ |
|---|---|
| log `so100_interfaces is not installed` / หุ่นพูด "My arm software is not set up yet." | ทำข้อ 0 |
| หุ่นพูด "My arm is not connected." | ไม่เห็น `/arm/pick` → แขนไม่ได้รัน / ROS_DOMAIN_ID / คนละ network · `ros2 action list` |
| แขนหยิบ 2 ครั้ง / ขึ้น BUSY | เปิด `arm:=topics` ทับ หรือมี bridge 2 ตัว (Jetson + VM) → ใช้ `arm:=actions` ที่เดียว |
| ไปต่อเองก่อนแขนหยิบเสร็จ | มี event จำลองรันอยู่ (sim) → ปิด |
| `UNREACHABLE` ทุกครั้ง | จุดหยิบไกล/ใกล้เกิน → ปรับ `pick_forward` (เริ่มที่ 0.20) |
| `TOO_WIDE` | `object_size` กว้างเกิน 6.5 ซม. หรือใส่ผิดลำดับ (ลึก, กว้าง, สูง) |

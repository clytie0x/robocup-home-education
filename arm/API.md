# SO-100 Arm API (`/arm/*`)

API สำหรับให้ส่วนอื่นของหุ่น (ระบบเสียง, กล้อง, ตัวสั่งงานหลัก) สั่งแขนผ่าน ROS 2
นิยามอยู่ในแพ็กเกจ `so100_interfaces` ตัวเซิร์ฟเวอร์อยู่ในโหนด `so100_pick_place`
ซึ่งเปิดเองเมื่อรัน `Start-SO100.cmd` (โหมด MoveIt)

- ใช้ `ROS_DOMAIN_ID=42` และ `source ~/so100_ws/install/setup.bash` ก่อน
- หน่วยเป็น **เมตร** และ **เรเดียน**
- พิกัดเทียบกับ frame `base` (ฐานแขน) หรือ frame อื่นที่มี TF ไปถึง `base` เช่น frame ของกล้อง

## สรุป
| ชื่อ | ชนิด | ทำอะไร |
|---|---|---|
| `/arm/pick` | action `so100_interfaces/action/Pick` | หยิบของ แล้วยกมาถือไว้ที่ท่า ready |
| `/arm/place` | action `so100_interfaces/action/Place` | วางของที่ถืออยู่ แล้วกลับท่า ready |
| `/arm/pick_place` | action `so100_interfaces/action/PickPlace` | หยิบแล้ววางในคำสั่งเดียว |
| `/arm/home` | action `so100_interfaces/action/Home` | กลับท่า ready |
| `/arm/rest` | action `so100_interfaces/action/Rest` | หุบก้ามแล้วพับแขนลงท่าพัก (ถ้าถือของอยู่จะได้ `HOLDING`) |
| `/arm/stop` | service `std_srvs/srv/Trigger` | หยุดทันที แขนค้างท่าตรงนั้น งานที่ทำอยู่จบด้วย `STOPPED` |
| `/arm/torque` | service `std_srvs/srv/SetBool` | `true` = ล็อกแรงที่ท่าปัจจุบัน, `false` = ปลดแรง (**ประคองแขนก่อน แขนจะร่วง**) |
| `/arm/state` | topic `so100_interfaces/msg/ArmState` (2 Hz) | สถานะแขน |

ทำได้ทีละงาน ถ้าสั่งงานใหม่ระหว่างที่งานเดิมยังไม่เสร็จ จะได้ `BUSY` ทันที แขนจะไม่ขยับ
(ยกเว้นตอนแขนกำลังไปท่าพัก ดูหัวข้อถัดไป)

## กลับท่าพักเอง (auto rest)
- แขนจะหุบก้ามแล้วพับลงท่าพักเอง ถ้าไม่ได้ถือของ และไม่มีงานใหม่มานาน `rest_after_s` วินาที (ค่าเริ่มต้น 120)
  - เปลี่ยนเวลาได้: `ros2 param set /so100_pick_place rest_after_s 60.0` ถ้าใส่ `0.0` คือปิด
  - งานจากทุกช่องทางนับหมด (API, topic เสียงเดิม, ปุ่มใน panel) เริ่มนับใหม่ทุกครั้งที่งานจบ
- จะ**ไม่**ไปท่าพักเองในกรณีเหล่านี้:
  - ถือของอยู่
  - ปลดแรงอยู่ หรือมี fault
  - งานล่าสุดโดนสั่งหยุด (`STOPPED`)
  - งานล่าสุดพังระหว่างที่ของอาจยังค้างอยู่ในก้าม
  - สองกรณีหลังจะกลับมานับเวลาใหม่เมื่อมีคำสั่งใหม่ หรือมีคนปลดแรงแขน
- ถ้าสั่งงานใหม่ตอนแขนกำลังไปท่าพัก จะไม่ได้ `BUSY` แขนจะเลิกไปท่าพักแล้วทำงานใหม่ทันที
- ถ้าสั่งงานตอนอยู่ท่าพัก แขนจะขึ้นไปท่า ready ก่อนแล้วค่อยทำงาน งานจึงนานขึ้นประมาณ 3–5 วินาที
- ที่ท่าพักแขนยังล็อกแรงไว้ ส่วน `/arm/state` จะแสดงแบบนี้:
  - ระหว่างไปท่าพัก: `state=busy`, `task=rest`
  - ถึงท่าพักแล้ว: `state=resting`

## ข้อมูลที่ส่งเข้า
**ตำแหน่งของ / จุดวาง** (`geometry_msgs/PoseStamped`)
- `position.x, y` = จุดกึ่งกลางฐานของของบนโต๊ะ ค่า `z` ไม่ใช้ เพราะถือว่าของตั้งอยู่บนโต๊ะเสมอ
- `orientation` = มุมหมุนของของรอบแกนตั้ง (yaw)
  - ถ้าใส่ quaternion เป็น 0 ทั้งหมด หมายถึง "ไม่รู้มุม" แขนจะถือว่าด้าน "ลึก" ของของชี้มาทางแขน
- `header.frame_id` ใส่ `""` หรือ `"base"` หมายถึงพิกัดเทียบกับฐานแขน
  - frame อื่น (เช่น กล้อง) แขนจะแปลงผ่าน TF ให้ แต่ต้องมีคน publish TF ของกล้องเทียบกับ `base` ก่อน

**ขนาดของ** `object_size = [ลึก, กว้าง, สูง]` (เมตร)
- ลึก = ตามแกน x ของของเอง, กว้าง = ตามแกน y ของของเอง
- ก้ามหนีบได้กว้างสุด ~6.5 ซม. แขนเลือกหนีบด้านที่แคบกว่าให้เอง

**ท่าจับ** `grasp_mode` = `"auto"` (ค่าเริ่มต้น), `"top"` (จับจากด้านบน) หรือ `"side"` (จับจากด้านข้าง)

**พิกัดฐานแขน (`base`) เทียบกับการวัดด้วยตลับเมตร:**
แกนหมุนฐานอยู่ที่ `(x=0, y=-0.0452)` ทิศ "หน้า" ของแขนคือ **-y** ทิศ "ซ้าย" คือ **+x** เช่น
- ของอยู่หน้าแกนฐาน 20 ซม. เยื้องซ้าย 5 ซม. → `x = 0.05, y = -0.0452 - 0.20 = -0.2452`
- ใน Python ใช้ `user_xy(0.20, 0.05)` แปลงให้ได้

## ผลลัพธ์ (ทุก action)
- `success` (bool)
- `error_code` (string)
- `message`: ภาษาอังกฤษ ระบบเสียงเอาไปพูดได้เลย

| error_code | ความหมาย |
|---|---|
| `SUCCESS` | สำเร็จ |
| `UNREACHABLE` | เอื้อมไม่ถึงของหรือจุดวาง (ไกล/ใกล้เกิน หรือท่าจับนั้นทำไม่ได้) |
| `TOO_WIDE` | ของกว้างเกินก้าม |
| `MISSED` | หนีบพลาด (ก้ามหุบสุด) แขนถอยขึ้นแล้ว |
| `COLLISION` | แขนจะชนโต๊ะ หรือถูกขวาง/ติดกลางทาง |
| `BUSY` | กำลังทำงานอื่นอยู่ |
| `HOLDING` | ถือของอยู่แล้ว (สั่ง pick ซ้ำ) |
| `NOT_HOLDING` | สั่ง place แต่ไม่ได้ถือของ |
| `HOT` | มอเตอร์ร้อนเกิน รอให้เย็นก่อน |
| `STOPPED` | ถูกสั่งหยุด (`/arm/stop` หรือยกเลิก goal) |
| `FAILED` | ผิดพลาดอื่น ๆ (ดูข้อความใน log ของ `so100_pick_place`) |

**ความคืบหน้า (feedback)** ระหว่างทำงาน:
- `step`: `leave_rest`, `move_above`, `open`, `approach`, `grasp`, `lift`, `carry`, `move_above_target`, `lower`, `retreat`, `home`, `close`, `rest`
- `step_index` / `step_count`: เช่น 3 / 6

**เวลาโดยประมาณ:** pick ~12–18 วิ, place ~8–12 วิ, pick_place ~20–30 วิ
ไม่ต้องตั้งเวลารอเอง action จะตอบกลับเมื่อเสร็จ

## สถานะ `/arm/state`
`state` มีค่าได้ดังนี้:
- `idle`: ว่าง
- `resting`: อยู่ท่าพัก (ว่าง พร้อมรับงาน)
- `busy`: กำลังทำงาน
- `holding`: ถือของอยู่
- `fault`: หยุดเพราะมีปัญหา เช่น ร้อนเกิน
- `torque_off`: ปลดแรงอยู่
- `no_arm`: ไม่ได้ข้อมูลจากแขน

ฟิลด์อื่น: `holding`, `torque_on`, `fault`, `voltage_min`, `temp_max`, `task`, `step`, `last_error_code`, `last_message`

## ตัวอย่างจาก terminal
```
ros2 topic echo /arm/state
ros2 action send_goal --feedback /arm/pick so100_interfaces/action/Pick \
  "{object_pose: {header: {frame_id: base}, pose: {position: {x: 0.0, y: -0.245}}}, object_size: [0.03, 0.03, 0.05]}"
ros2 action send_goal /arm/place so100_interfaces/action/Place \
  "{target_pose: {header: {frame_id: base}, pose: {position: {x: -0.10, y: -0.245}}}}"
ros2 action send_goal /arm/home so100_interfaces/action/Home "{}"
ros2 action send_goal /arm/rest so100_interfaces/action/Rest "{}"
ros2 service call /arm/stop std_srvs/srv/Trigger
ros2 service call /arm/torque std_srvs/srv/SetBool "{data: true}"
```
คำสั่งย่อ (ใช้พิกัด "หน้า ซ้าย" แบบตลับเมตร หน่วยเมตร):
```
ros2 run so100_driver arm_cli pick 0.20 0.0 --size 0.03 0.03 0.05
ros2 run so100_driver arm_cli place 0.20 -0.10
ros2 run so100_driver arm_cli pick_place 0.20 0.0 0.20 -0.10 --size 0.03 0.03 0.05
ros2 run so100_driver arm_cli home | rest | stop | lock | release | state
```

## ตัวอย่างจาก Python (โหนดของเพื่อน)
```python
import threading, rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from so100_driver.arm_client import ArmClient, user_xy

rclpy.init()
node = Node('my_task')
arm = ArmClient(node)
ex = MultiThreadedExecutor(); ex.add_node(node)
threading.Thread(target=ex.spin, daemon=True).start()

ok, code, msg = arm.pick(user_xy(0.20, 0.0), size=(0.03, 0.03, 0.05))   # รอจนเสร็จ
if ok:
    ok, code, msg = arm.place(user_xy(0.20, -0.10))
print(ok, code, msg)          # เช่น True SUCCESS "I have put it down."
```
ถ้าตำแหน่งมาจากกล้อง ส่ง `PoseStamped` ใน frame ของกล้องเข้าไปได้เลย: `arm.pick(pose_from_camera, size=...)`

## ของเดิมที่ยังใช้ได้ (ระบบเสียงรุ่นแรก)
`/robot/carry_bag`, `/robot/drop_bag` (`std_msgs/Empty`) → `/robot/event` (`grab_success`/`grab_failed`, `drop_done`/`drop_failed`)
ตำแหน่งของมาจากกล่องฟ้า/ม่วงใน RViz แนะนำให้ย้ายมาใช้ `/arm/pick` และ `/arm/place` แทน

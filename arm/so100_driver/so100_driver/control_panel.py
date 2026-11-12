"""Small SO-100 control window (shown next to RViz): release / lock torque, stop, live status."""
import threading
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

FONT = ('Noto Sans Thai', 12)
BIG = ('Noto Sans Thai', 13, 'bold')


class Panel(Node):
    def __init__(self):
        super().__init__('so100_control_panel')
        self.status = 'รอข้อมูลจากแขน...'
        self.pick_status = ''
        self.create_subscription(String, 'so100/status', self.on_status, 10)
        self.create_subscription(String, 'so100/pick_place/status',
                                 lambda m: setattr(self, 'pick_status', m.data), 10)
        self.enable = self.create_client(SetBool, 'so100/enable')
        self.stop = self.create_client(Trigger, 'so100/stop')
        self.pick = self.create_client(Trigger, 'so100/pick_place')
        self.touch = self.create_client(Trigger, 'so100/touch')
        self.set_size = self.create_client(SetParameters, 'so100_pick_place/set_parameters')

    def on_status(self, msg):
        self.status = msg.data

    def call(self, client, req, label, out):
        if not client.wait_for_service(timeout_sec=2.0):
            out.set(f'{label}: ไม่พบไดรเวอร์แขน')
            return
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: out.set(f'{label}: ' + (f.result().message if f.result() else 'ล้มเหลว')))

    def send_size(self, size_m, out):
        if not self.set_size.wait_for_service(timeout_sec=2.0):
            out.set('ขนาด: ไม่พบโปรแกรมหยิบ-วาง (เปิดโหมด MoveIt หรือยัง?)')
            return
        val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=size_m)
        fut = self.set_size.call_async(SetParameters.Request(parameters=[Parameter(name='object_size', value=val)]))

        def done(f):
            r = f.result().results[0] if f.result() else None
            out.set('ขนาด: ' + ('ตั้งแล้ว' if r and r.successful else (r.reason if r else 'ล้มเหลว')))
        fut.add_done_callback(done)


def main():
    rclpy.init()
    node = Panel()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title('SO-100 ควบคุมแขน')
    root.geometry('+0+372')   # over RViz's empty lower-left area, 3D view stays clear
    root.attributes('-topmost', True)
    msg = tk.StringVar(value='')
    stat = tk.StringVar(value=node.status)

    def release():
        if messagebox.askyesno('ปลดแรงมอเตอร์',
                               'แขนจะอ่อนและร่วงลงทันที\n\nเอามือประคองแขนไว้แล้วใช่ไหม?', icon='warning'):
            node.call(node.enable, SetBool.Request(data=False), 'ปลดแรง', msg)

    def lock():
        node.call(node.enable, SetBool.Request(data=True), 'ล็อกแรง', msg)

    def stop():
        node.call(node.stop, Trigger.Request(), 'หยุด', msg)

    tk.Button(root, text='🔓 ปลดแรง — ขยับแขนด้วยมือ', font=BIG, bg='#f4c542', command=release,
              width=28).pack(padx=10, pady=(6, 2))
    tk.Button(root, text='🔒 ล็อกแรงที่ท่าปัจจุบัน', font=BIG, bg='#7ec87e', command=lock,
              width=28).pack(padx=10, pady=2)
    tk.Button(root, text='■ หยุดการเคลื่อนที่', font=BIG, bg='#e86a5f', fg='white', command=stop,
              width=28).pack(padx=10, pady=2)
    # ---- IK pick-and-place test
    box = tk.LabelFrame(root, text=' หยิบ-วาง: ใน RViz ลากกล่องฟ้า = ของ, กล่องม่วง = จุดวาง ',
                        font=('Noto Sans Thai', 10, 'bold'))
    box.pack(padx=10, pady=(8, 4), fill='x')
    row = tk.Frame(box)
    row.pack(pady=(4, 2))
    entries = []
    for label, default in (('ลึก', '3'), ('กว้าง', '3'), ('สูง', '5')):
        tk.Label(row, text=label, font=FONT).pack(side='left')
        e = tk.Entry(row, width=4, font=FONT, justify='center')
        e.insert(0, default)
        e.pack(side='left', padx=(2, 6))
        entries.append(e)
    tk.Label(row, text='ซม.', font=FONT).pack(side='left')

    def apply_size():
        try:
            v = [float(e.get()) / 100 for e in entries]
        except ValueError:
            msg.set('ขนาด: ใส่ตัวเลขเป็นเซนติเมตร')
            return
        node.send_size(v, msg)

    tk.Button(row, text='ใช้ขนาดนี้', font=FONT, command=apply_size).pack(side='left', padx=(6, 0))
    tk.Button(box, text='▶ หยิบ-วาง', font=BIG, bg='#5b9bd5', fg='white', width=26,
              command=lambda: node.call(node.pick, Trigger.Request(), 'หยิบ-วาง', msg)).pack(padx=6, pady=3)
    tk.Button(box, text='☝ แตะจุดกล่องฟ้า (เช็กความแม่น)', font=FONT, width=30,
              command=lambda: node.call(node.touch, Trigger.Request(), 'แตะจุด', msg)).pack(padx=6, pady=(0, 3))
    pick = tk.StringVar(value='')
    tk.Label(box, textvariable=pick, font=('Noto Sans Thai', 10), justify='left', wraplength=330,
             fg='#333').pack(padx=6, pady=(0, 6), anchor='w')

    tk.Label(root, textvariable=msg, font=FONT, fg='#245').pack(padx=10, pady=(6, 0))
    tk.Label(root, textvariable=stat, font=('Noto Sans Mono', 9), wraplength=330, justify='left').pack(
        padx=10, pady=(4, 10))

    def refresh():
        if not rclpy.ok():          # launch stopped us (SIGINT/SIGTERM) -> close the window too
            root.destroy()
            return
        stat.set(node.status.replace(' fault=', '\nfault=').replace(' load%', '\nload%'))
        pick.set(node.pick_status)
        root.after(500, refresh)

    refresh()
    root.mainloop()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Record /joint_states and summarize shaking per joint.

  ros2 run so100_driver record_motion <seconds>

Shake = rapid back-and-forth: we count direction reversals of each joint's velocity
with amplitude > 0.3 deg, and the RMS of the position minus its 0.2 s moving average.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    rclpy.init()
    node = Node('record_motion')
    rows = []
    node.create_subscription(JointState, 'joint_states',
                             lambda m: rows.append((time.monotonic(), dict(zip(m.name, m.position)))), 50)
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()
    if len(rows) < 20:
        sys.exit('too few samples')
    names = list(rows[0][1])
    print(f'{len(rows)} samples in {rows[-1][0] - rows[0][0]:.1f}s')
    print(f'{"joint":14s} {"travel":>8s} {"reversals":>9s} {"jitter_rms":>10s}')
    for n in names:
        x = [math.degrees(r[1][n]) for r in rows]
        w = 10
        smooth = [sum(x[max(0, i - w):i + w + 1]) / len(x[max(0, i - w):i + w + 1]) for i in range(len(x))]
        jitter = math.sqrt(sum((a - b) ** 2 for a, b in zip(x, smooth)) / len(x))
        rev, last_dir, anchor = 0, 0, x[0]
        for v in x[1:]:
            d = v - anchor
            if abs(d) > 0.3:
                direction = 1 if d > 0 else -1
                if last_dir and direction != last_dir:
                    rev += 1
                last_dir, anchor = direction, v
        print(f'{n:14s} {max(x) - min(x):7.1f}° {rev:9d} {jitter:9.2f}°')


if __name__ == '__main__':
    main()

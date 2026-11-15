"""End-to-end check of the /arm/* API against a running stack (use hardware:=sim!).

  ros2 run so100_driver api_selftest
Runs pick / place / pick_place / home / rest, the error cases, stop and the idle auto-rest, and prints PASS/FAIL per case.
"""
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .arm_client import ArmClient, user_xy

SIZE = (0.03, 0.03, 0.05)


def main():
    rclpy.init()
    node = Node('arm_api_selftest')
    from rcl_interfaces.srv import GetParameters
    params = node.create_client(GetParameters, '/so100_pick_place/get_parameters')

    def rest_after():
        if not params.wait_for_service(timeout_sec=2.0):
            return None
        fut = params.call_async(GetParameters.Request(names=['rest_after_s']))
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        return fut.result().values[0].double_value if fut.done() else None
    arm = ArmClient(node)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    results = []

    def check(name, got, want):
        ok = got[1] == want if isinstance(got, tuple) and len(got) == 3 else got == want
        results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}  {name:38s} -> {got}', flush=True)

    for _ in range(100):
        if arm.state is not None:
            break
        time.sleep(0.1)
    check('state is published', arm.state is not None and arm.state.state in ('idle', 'resting', 'torque_off'), True)
    check('place while not holding', arm.place(user_xy(0.20, -0.10)), 'NOT_HOLDING')
    check('pick too far', arm.pick(user_xy(0.60, 0.0), SIZE), 'UNREACHABLE')
    check('pick too wide', arm.pick(user_xy(0.20, 0.0), (0.09, 0.09, 0.05)), 'TOO_WIDE')
    check('pick', arm.pick(user_xy(0.20, 0.0), SIZE), 'SUCCESS')
    time.sleep(1.2)                  # /arm/state is 2 Hz
    check('state says holding', arm.state.holding and arm.state.state == 'holding', True)
    check('pick again while holding', arm.pick(user_xy(0.20, 0.0), SIZE), 'HOLDING')
    check('place', arm.place(user_xy(0.18, -0.10)), 'SUCCESS')
    check('pick_place (top grasp)', arm.pick_place(user_xy(0.18, -0.10), user_xy(0.22, 0.08), SIZE,
                                                   grasp_mode='top'), 'SUCCESS')
    # busy: start a pick without waiting, then ask for home
    fut = arm.pick(user_xy(0.22, 0.08), SIZE, wait=False)
    time.sleep(1.5)
    check('home while busy', arm.home(), 'BUSY')
    time.sleep(1.0)
    check('stop service', arm.stop()[0], True)
    handle = fut.result()
    res = handle.get_result_async()
    while not res.done():
        time.sleep(0.05)
    check('stopped pick reports STOPPED', res.result().result.error_code, 'STOPPED')
    check('home', arm.home(), 'SUCCESS')
    check('torque lock', arm.torque(True)[0], True)
    # rest pose: /arm/rest, then a pick straight from the rest pose, then the idle timer (rest_after_s)
    check('rest', arm.rest(), 'SUCCESS')
    time.sleep(1.2)
    check('state says resting', arm.state.state, 'resting')
    check('pick from the rest pose', arm.pick(user_xy(0.20, 0.0), SIZE), 'SUCCESS')
    check('rest while holding', arm.rest(), 'HOLDING')
    check('place', arm.place(user_xy(0.20, 0.0)), 'SUCCESS')
    after = rest_after()
    if after and after <= 20:
        t0 = time.monotonic()
        seen_task = False
        while time.monotonic() - t0 < after + 30:
            time.sleep(0.2)
            seen_task |= arm.state.task == 'rest'
            if arm.state.state == 'resting':
                break
        check(f'auto rest after {after:.0f} s (task=rest seen)', seen_task and arm.state.state == 'resting', True)
        # a new goal while folding must not get BUSY
        check('home', arm.home(), 'SUCCESS')
        t0 = time.monotonic()
        while arm.state.task != 'rest' and time.monotonic() - t0 < after + 10:
            time.sleep(0.1)
        time.sleep(0.5)
        check('goal during the fold replaces it', arm.home(), 'SUCCESS')
    else:
        print('(skip auto-rest checks: start the stack with a short rest_after_s, e.g. '
              'ros2 param set /so100_pick_place rest_after_s 5.0)', flush=True)
    print(f'\n{sum(results)}/{len(results)} passed', flush=True)
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

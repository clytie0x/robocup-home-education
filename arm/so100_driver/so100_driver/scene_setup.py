"""Adds the table to MoveIt's planning scene once move_group is up (then exits)."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from std_msgs.msg import ColorRGBA
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node('so100_scene_setup')
    table_z = node.declare_parameter('table_z', -0.003).value
    cli = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    while not cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('waiting for move_group ...')
    box = CollisionObject()
    box.header.frame_id = 'base'
    box.id = 'table'
    box.operation = CollisionObject.ADD
    box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.2, 1.2, 0.02]))
    pose = Pose()
    pose.position.z = table_z - 0.01   # box top surface at table_z
    pose.orientation.w = 1.0
    box.primitive_poses.append(pose)
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects.append(box)
    scene.object_colors.append(ObjectColor(id='table', color=ColorRGBA(r=0.55, g=0.45, b=0.35, a=1.0)))
    future = cli.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    ok = future.result() is not None and future.result().success
    node.get_logger().info(f'table at z={table_z:.3f} m added: {ok}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

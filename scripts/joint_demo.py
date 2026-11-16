#!/usr/bin/env python3
"""Small base swivel only; not a pick-and-place demonstration."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rclpy.init()
    node = Node('gp7_joint_demo')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError('arm_controller action server is not available')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES
    for seconds, angle in [(1, 0.0), (4, 0.15), (7, -0.15), (10, 0.0)]:
        point = JointTrajectoryPoint()
        point.positions = [angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.velocities = [0.0] * 6
        point.time_from_start.sec = seconds
        goal.trajectory.points.append(point)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError('Trajectory rejected')
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    node.get_logger().info(str(result.result().result))
    node.destroy_node()
    rclpy.shutdown()


JOINT_NAMES = ['L0_L1_Joint', 'L1_L2_Joint', 'L2_L3_Joint', 'L3_L4_Joint', 'L4_L5_Joint', 'L5_L6_Joint']
if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Execute a planned pick/place using an idealized Gazebo fixed-joint grasp.

The detachable joint models adhesion, not contact/friction force closure. No object
pose is ever commanded. Success requires measured lift and released final pose.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from gp7_kinematics import Kinematics


class Executor(Node):
    def __init__(self, plan):
        super().__init__('robo2_pick_place')
        self.plan = plan
        self.pose = None
        self.pose_time = 0.0
        self.state = None
        self.state_seq = 0
        self.joints = {}
        self.joint_time = 0.0
        self.carry_poses = []
        self.carrying = False
        self.kinematics = Kinematics(Path(get_package_share_directory('robo2')) / 'urdf/robo2.urdf')
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/model/box_b/pose', self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/gripper/state', self.on_state, 10)
        self.attach = self.create_publisher(Empty, '/gripper/attach', 10)
        self.detach = self.create_publisher(Empty, '/gripper/detach', 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    '/gripper_controller/follow_joint_trajectory')
        self.events = []

    def on_pose(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.plan.get('object_name', 'box_b'):
                t, q = tf.transform.translation, tf.transform.rotation
                self.pose = {'position': [t.x, t.y, t.z],
                             'orientation': [q.x, q.y, q.z, q.w]}
                self.pose_time = time.monotonic()
                if self.carrying:
                    self.carry_poses.append(dict(self.pose, wall_time=self.pose_time))

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        self.joint_time = time.monotonic()

    def on_state(self, msg):
        self.state = msg.data.strip().lower()
        self.state_seq += 1

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('Timed out: ' + description)

    def future(self, future, timeout, description):
        self.wait(future.done, timeout, description)
        return future.result()

    def fresh_pose(self):
        start = time.monotonic()
        self.wait(lambda: self.pose_time >= start, 10, 'fresh object pose')
        return dict(self.pose)

    def trajectory(self, client, names, points):
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Trajectory action server unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        previous = -1.0
        for sample in points:
            seconds = float(sample['time_from_start'])
            positions = sample['positions']
            if seconds <= previous or len(positions) != len(names) or not all(
                    math.isfinite(x) for x in positions):
                raise ValueError('Invalid trajectory point')
            previous = seconds
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = sample.get('velocities', [0.0] * len(names))
            if len(point.velocities) != len(names) or not all(math.isfinite(x) for x in point.velocities):
                raise ValueError('Invalid trajectory velocities')
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = round((seconds % 1) * 1e9)
            goal.trajectory.points.append(point)
        if not points:
            raise ValueError('Empty trajectory')
        handle = self.future(client.send_goal_async(goal), 15, 'goal acceptance')
        if not handle.accepted:
            raise RuntimeError('Trajectory rejected')
        pending = handle.get_result_async()
        try:
            result = self.future(pending, previous * 4 + 30, 'trajectory completion')
        except Exception:
            handle.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError('Trajectory failed: ' + str(result.result))

    def fingers(self, value):
        self.trajectory(self.gripper, ['L6_L7_Joint', 'L6_L7.2'],
                        [{'positions': [value, value], 'time_from_start': 1.0}])

    def attachment(self, attached):
        publisher = self.attach if attached else self.detach
        expected = 'attached' if attached else 'detached'
        self.wait(lambda: publisher.get_subscription_count() > 0, 10,
                  'attachment bridge subscriber')
        seq = self.state_seq
        # ROS bridge discovery can precede its Gazebo transport subscription.
        # Repeat the idempotent command until the simulator acknowledges it.
        deadline = time.monotonic() + 10.0
        while not (self.state_seq > seq and self.state == expected):
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out: Gazebo grasp state ' + expected)
            publisher.publish(Empty())
            end = time.monotonic() + 0.2
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        self.carrying = attached
        self.events.append({'attachment': expected, 'pose': self.fresh_pose()})

    def run(self):
        phases = self.plan['phases']
        if [p['name'] for p in phases] != ['pregrasp', 'grasp', 'lift', 'transfer', 'place', 'retreat']:
            raise ValueError('Expected pregrasp/grasp/lift/transfer/place/retreat phases')
        # The Harmonic plugin starts attached; detach before either controller moves.
        self.attachment(False)
        self.fingers(0.0)
        initial = self.fresh_pose()
        lifted = None
        for phase in phases:
            self.get_logger().info('Executing ' + phase['name'])
            self.trajectory(self.arm, self.plan['joint_names'], phase['points'])
            self.events.append({'phase': phase['name'], 'pose': self.fresh_pose()})
            if phase['name'] == 'grasp':
                approached = self.fresh_pose()
                displacement = math.dist(approached['position'], initial['position'])
                qa, qb = approached['orientation'], initial['orientation']
                dot = abs(sum(a*b for a,b in zip(qa,qb)))
                norm = math.sqrt(sum(a*a for a in qa)*sum(b*b for b in qb))
                rotation = 2*math.acos(min(1.0, dot/norm))
                if displacement > 0.003 or rotation > 0.02:
                    raise RuntimeError('Cube disturbed during approach: '
                                       f'{displacement:.6f}m, {rotation:.6f}rad')
                start = time.monotonic()
                self.wait(lambda: self.joint_time >= start and all(n in self.joints for n in self.plan['joint_names']), 10, 'fresh grasp joint state')
                tcp = self.kinematics.fk([self.joints[n] for n in self.plan['joint_names']])[:3, 3]
                separation = math.dist(tcp, self.fresh_pose()['position'])
                self.events.append({'grasp_tcp_object_distance_m': separation})
                if separation > 0.025:
                    raise RuntimeError('TCP too far from object to grasp')
                self.fingers(0.023)
                self.attachment(True)
            elif phase['name'] == 'lift':
                lifted = self.fresh_pose()
                if lifted['position'][2] - initial['position'][2] < 0.05:
                    raise RuntimeError('Object did not lift by at least 5 cm')
            elif phase['name'] == 'place':
                self.attachment(False)
                self.fingers(0.0)
        # Allow gravity/contact to settle before reading a new simulator pose.
        end = time.monotonic() + 2.0
        self.wait(lambda: time.monotonic() >= end, 4, 'settling')
        final = self.fresh_pose()
        target = self.plan['expected_object_pose']
        distance = math.dist(final['position'], target['position'])
        qa, qb = final['orientation'], target['orientation']
        norm = math.sqrt(sum(x*x for x in qa) * sum(x*x for x in qb))
        if norm < 1e-12:
            raise RuntimeError('Invalid orientation quaternion')
        angle = 2 * math.acos(min(1.0, abs(sum(a*b for a,b in zip(qa,qb))) / norm))
        carry_angles = []
        for pose in self.carry_poses:
            q = pose['orientation']
            norm_q = math.sqrt(sum(x*x for x in q) * sum(x*x for x in qb))
            carry_angles.append(2 * math.acos(min(1.0, abs(sum(a*b for a,b in zip(q,qb))) / norm_q)))
        max_carry_angle = max(carry_angles, default=math.inf)
        success = (distance <= self.plan.get('position_tolerance_m', 0.025)
                   and max_carry_angle <= self.plan.get('orientation_tolerance_rad', 0.15)
                   and angle <= self.plan.get('orientation_tolerance_rad', 0.15)
                   and self.state == 'detached')
        return {'success': success, 'initial_pose': initial, 'lifted_pose': lifted,
                'final_pose': final, 'expected_pose': target,
                'position_error_m': distance, 'orientation_error_rad': angle,
                'max_carry_orientation_error_rad': max_carry_angle,
                'grasp_model': 'Gazebo detachable fixed joint; idealized adhesion, not friction'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=Path(get_package_share_directory('robo2')) / 'config/pick_place_plan.json')
    parser.add_argument('--results', type=Path, default=Path('pick_place_results.json'))
    args = parser.parse_args()
    rclpy.init()
    node = None
    result = {'success': False}
    try:
        node = Executor(json.loads(args.plan.read_text()))
        result = node.run()
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if node:
            result['events'] = node.events
            result['carry_pose_samples'] = node.carry_poses
            node.destroy_node()
        rclpy.shutdown()
        args.results.parent.mkdir(parents=True, exist_ok=True)
        args.results.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

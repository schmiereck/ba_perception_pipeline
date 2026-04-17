#!/usr/bin/env python3
"""Minimal MoveIt goal test — bypasses the perception pipeline.

Sends a hard-coded goal to /move_action in two possible modes:

    --mode pose    (default) send PositionConstraint + OrientationConstraint
                   (matches what goal_generator_node currently does)
    --mode joint   resolve IK first via /compute_ik, then send JointConstraints
                   (matches what RViz's "Plan & Execute" button does)

Usage:
    python3 test_moveit_goal.py --x 0.187 --y 0.043 --z 0.053 --mode pose
    python3 test_moveit_goal.py --x 0.187 --y 0.043 --z 0.053 --mode joint
    python3 test_moveit_goal.py --x 0.15 --y 0.0 --z 0.20 --mode pose   # reachable sanity check

Adds --execute to actually run the motion (default: plan only).
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


ARM_GROUP = 'arm'
TIP_LINK = 'tcp_link'
BASE_FRAME = 'base_link'


def make_pose_constraints(x, y, z, qx, qy, qz, qw,
                          pos_tol=0.02, ori_tol=(2.0, 2.0, 3.14)):
    c = Constraints()
    c.name = 'test_pose'

    pc = PositionConstraint()
    pc.header.frame_id = BASE_FRAME
    pc.link_name = TIP_LINK
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.SPHERE
    prim.dimensions = [pos_tol]
    bv = BoundingVolume()
    bv.primitives.append(prim)
    center = Pose()
    center.position.x = x
    center.position.y = y
    center.position.z = z
    center.orientation.w = 1.0
    bv.primitive_poses.append(center)
    pc.constraint_region = bv
    pc.weight = 1.0
    c.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header.frame_id = BASE_FRAME
    oc.link_name = TIP_LINK
    oc.orientation.x = qx
    oc.orientation.y = qy
    oc.orientation.z = qz
    oc.orientation.w = qw
    oc.absolute_x_axis_tolerance = ori_tol[0]
    oc.absolute_y_axis_tolerance = ori_tol[1]
    oc.absolute_z_axis_tolerance = ori_tol[2]
    oc.weight = 1.0
    c.orientation_constraints.append(oc)
    return c


def make_joint_constraints(joint_state: JointState, tol=0.01):
    c = Constraints()
    c.name = 'test_joint'
    for name, pos in zip(joint_state.name, joint_state.position):
        if not name.startswith('joint_'):
            continue
        if name == 'joint_5' or name == 'joint_5_mimic':
            continue  # gripper not in arm group
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


class TestNode(Node):
    def __init__(self):
        super().__init__('moveit_test_goal')
        self._action = ActionClient(self, MoveGroup, 'move_action')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._latest_js = None
        self.create_subscription(
            JointState, '/joint_states',
            lambda m: setattr(self, '_latest_js', m), 10)

    def wait_for_joint_state(self, timeout=5.0):
        end = self.get_clock().now().seconds_nanoseconds()[0] + timeout
        while self._latest_js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().seconds_nanoseconds()[0] > end:
                return False
        return True

    def compute_ik(self, x, y, z, qx, qy, qz, qw):
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/compute_ik service not available')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP
        req.ik_request.ik_link_name = TIP_LINK
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = True

        req.ik_request.pose_stamped.header.frame_id = BASE_FRAME
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw

        if self._latest_js is not None:
            req.ik_request.robot_state.joint_state = self._latest_js

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error('IK service call failed / timeout')
            return None
        res = future.result()
        if res.error_code.val != 1:
            self.get_logger().error(f'IK failed: error_code={res.error_code.val}')
            return None

        self.get_logger().info(
            'IK ok: ' + ', '.join(
                f'{n}={p:.3f}' for n, p in zip(
                    res.solution.joint_state.name,
                    res.solution.joint_state.position)))
        return res.solution.joint_state

    def send_goal(self, constraints: Constraints, execute: bool):
        if not self._action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action not available')
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.goal_constraints.append(constraints)

        if self._latest_js is not None:
            goal.request.start_state.joint_state = self._latest_js
            goal.request.start_state.is_diff = False

        goal.planning_options.plan_only = not execute

        self.get_logger().info(f'Sending goal (plan_only={not execute})...')
        send_future = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted; waiting for result...')
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        ec = result.error_code.val
        if ec == 1:
            self.get_logger().info(f'SUCCESS (error_code={ec})')
        else:
            self.get_logger().error(f'FAILED (error_code={ec})')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--x', type=float, default=0.187)
    p.add_argument('--y', type=float, default=0.043)
    p.add_argument('--z', type=float, default=0.053)
    p.add_argument('--qx', type=float, default=0.0)
    p.add_argument('--qy', type=float, default=-0.7071068)
    p.add_argument('--qz', type=float, default=0.0)
    p.add_argument('--qw', type=float, default=0.7071068)
    p.add_argument('--mode', choices=['pose', 'joint'], default='pose')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()

    rclpy.init()
    node = TestNode()

    if not node.wait_for_joint_state():
        node.get_logger().warn('No /joint_states received; continuing without start_state.')

    if args.mode == 'pose':
        c = make_pose_constraints(args.x, args.y, args.z,
                                  args.qx, args.qy, args.qz, args.qw)
        node.send_goal(c, args.execute)
    else:
        js = node.compute_ik(args.x, args.y, args.z,
                             args.qx, args.qy, args.qz, args.qw)
        if js is None:
            node.get_logger().error('No IK solution — target unreachable or blocked.')
            node.destroy_node()
            rclpy.shutdown()
            return 1
        c = make_joint_constraints(js)
        node.send_goal(c, args.execute)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

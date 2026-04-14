#!/usr/bin/env python3
"""MoveIt Goal Generator for the Bracket-Arm (BA).

Listens for 3D target points from the perception pipeline and sends
move commands to MoveIt.

Subscribe: /ba_perception/target_pose (geometry_msgs/PoseStamped)
Action:    /ba_move_action (moveit_msgs/action/MoveGroup)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
import math
import numpy as np


def quaternion_from_euler(ai, aj, ak):
    """Convert Euler angles (roll, pitch, yaw) to Quaternion."""
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)

    q = Quaternion()
    q.x = si * cj * ck - ci * sj * sk
    q.y = ci * sj * ck + si * cj * sk
    q.z = ci * cj * sk - si * sj * ck
    q.w = ci * cj * ck + si * sj * sk
    return q


class BAGoalGenerator(Node):
    def __init__(self):
        super().__init__('ba_goal_generator')

        # -- parameters --------------------------------------------------
        self.declare_parameter('planning_group', 'bracket_arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('z_offset', 0.10)  # 10 cm safety distance
        self.declare_parameter('z_min', 0.0)      # Minimum safety height (table level)
        self.declare_parameter('auto_execute', False)

        self._group = self.get_parameter('planning_group').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self._z_min = self.get_parameter('z_min').get_parameter_value().double_value

        # -- Action Client for MoveIt -------------------------------------
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # -- Subscriber --------------------------------------------------
        self._sub = self.create_subscription(
            PoseStamped,
            '/ba_perception/target_pose',
            self._target_cb,
            10)

        self.get_logger().info(f'BA Goal Generator ready. Group: {self._group}')
        self.get_logger().info(f'Safety: z_min={self._z_min}m, z_offset={self._z_offset}m')

    def _target_cb(self, msg: PoseStamped):
        self.get_logger().info(f'Received target: X={msg.pose.position.x:.3f}, Y={msg.pose.position.y:.3f}, Z={msg.pose.position.z:.3f}')
        
        # 1. Create target pose with offset
        target_z = msg.pose.position.z + self._z_offset
        
        # 2. Safety Check: Don't go below z_min
        if target_z < self._z_min:
            self.get_logger().warn(f'Target Z ({target_z:.3f}m) is below safety limit ({self._z_min}m). Clipping to limit!')
            target_z = self._z_min

        goal_pose = msg
        goal_pose.pose.position.z = target_z
        
        # 3. Set Orientation: Gripper pointing DOWN
        # x=0, y=0.707, z=0, w=0.707 (Rotation by 90 deg around Y)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.7071
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.7071

        self.get_logger().info(f'Planning to Pre-Grasp Pose: Z={goal_pose.pose.position.z:.3f}')
        self.get_logger().info('To execute this plan, call MoveIt MoveGroup action...')
        # self._send_goal(goal_pose)

    def _send_goal(self, pose):
        # Implementation of the MoveGroup Action call would go here.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BAGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

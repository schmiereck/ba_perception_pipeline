#!/usr/bin/env python3
"""MoveIt Goal Generator for the Bracket-Arm (BA).

Listens for 3D target points from the perception pipeline and sends
move commands to MoveIt.

Subscribe: /ba_perception/target_pose (geometry_msgs/PoseStamped)
Action:    /move_action (moveit_msgs/action/MoveGroup)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint, BoundingVolume, RobotState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import math
import numpy as np


class BAGoalGenerator(Node):
    def __init__(self):
        super().__init__('ba_goal_generator')

        # -- state tracking ----------------------------------------------
        self._last_joint_state = None

        # -- parameters --------------------------------------------------
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('z_offset', 0.05)
        self.declare_parameter('z_min', 0.02)
        self.declare_parameter('auto_execute', False)

        self._group = self.get_parameter('planning_group').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self._z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self._auto_execute = self.get_parameter('auto_execute').get_parameter_value().bool_value

        # -- Action Client for MoveIt -------------------------------------
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # -- Subscribers --------------------------------------------------
        self._sub = self.create_subscription(
            PoseStamped,
            '/ba_perception/target_pose',
            self._target_cb,
            10)

        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_cb,
            10)

        self.get_logger().info(f'BA Goal Generator ready. Group: {self._group}')
        self.get_logger().info(f'Safety: z_min={self._z_min}m, z_offset={self._z_offset}m')
        self.get_logger().info(f'Auto-Execute: {self._auto_execute}')

    def _joint_cb(self, msg: JointState):
        """Store the latest joint positions from the robot."""
        self._last_joint_state = msg

    def _target_cb(self, msg: PoseStamped):
        """Handle incoming 3D target points."""
        self.get_logger().info(f'Received target: X={msg.pose.position.x:.3f}, Y={msg.pose.position.y:.3f}, Z={msg.pose.position.z:.3f}')
        
        # 1. Create target pose with offset (deep copy by creating a new message)
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y
        
        target_z = msg.pose.position.z + self._z_offset
        
        # 2. Safety Check: Don't go below z_min
        if target_z < self._z_min:
            self.get_logger().warn(f'Target Z ({target_z:.3f}m) is below safety limit ({self._z_min}m). Clipping!')
            target_z = self._z_min

        goal_pose.pose.position.z = target_z
        
        # 3. Set Orientation: Gripper horizontal (forward)
        # identity orientation (0,0,0,1)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Calculated Pre-Grasp Pose: Z={goal_pose.pose.position.z:.3f}')
        
        if self._auto_execute:
            self._send_goal(goal_pose)
        else:
            self.get_logger().info('Auto-Execute is OFF. Use --ros-args -p auto_execute:=true to enable.')

    def _send_goal(self, pose: PoseStamped):
        """Send the plan & execute request to MoveIt."""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveIt Action Server (move_action) not available!')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self._group
        goal_msg.request.num_planning_attempts = 50
        goal_msg.request.allowed_planning_time = 20.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # 1. Provide the Start State explicitly
        if self._last_joint_state:
            goal_msg.request.start_state.joint_state = self._last_joint_state
            goal_msg.request.start_state.is_diff = False
            self.get_logger().info('Using explicit Start State from /joint_states.')
        else:
            self.get_logger().warn('No joint states received yet. MoveIt will use its monitored state.')

        # 2. Build Constraints
        constraints = Constraints()
        constraints.name = "goal_pose"
        
        # Position Constraint (2cm Sphere)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = self._base_frame
        # Use zero timestamp to avoid clock sync issues in WSL2
        pos_con.header.stamp = rclpy.time.Time().to_msg()
        pos_con.link_name = 'tcp_link'
        
        volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose.pose)
        pos_con.constraint_region = volume
        pos_con.weight = 1.0
        constraints.position_constraints.append(pos_con)

        # Orientation Constraint (Loose)
        ori_con = OrientationConstraint()
        ori_con.header = pos_con.header
        ori_con.link_name = 'tcp_link'
        ori_con.orientation = pose.pose.orientation
        # Loosen tolerances significantly to help IK find any valid state
        ori_con.absolute_x_axis_tolerance = 2.0
        ori_con.absolute_y_axis_tolerance = 2.0
        ori_con.absolute_z_axis_tolerance = 3.14
        ori_con.weight = 1.0
        constraints.orientation_constraints.append(ori_con)

        goal_msg.request.goal_constraints.append(constraints)
        
        # We want MoveIt to plan AND execute
        goal_msg.planning_options.plan_only = False

        self.get_logger().info(f'Sending goal to MoveIt for "tcp_link" at [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}]')
        self._action_client.send_goal_async(goal_msg)

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

#!/usr/bin/env python3
"""MoveIt Goal Generator for the Bracket-Arm (BA).

Listens for 3D target points from the perception pipeline and sends
move commands to MoveIt.

The arm is 5-DOF, so arbitrary 6D pose-goals are infeasible. We call
/compute_ik first (pick_ik with rotation_scale=0.0 picks any feasible
orientation at the target position) and then send JointConstraints —
the same pattern RViz uses internally.

Subscribe: /ba_perception/target_pose (geometry_msgs/PoseStamped)
Service:   /compute_ik (moveit_msgs/srv/GetPositionIK)
Action:    /move_action (moveit_msgs/action/MoveGroup)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


class BAGoalGenerator(Node):
    def __init__(self):
        super().__init__('ba_goal_generator')

        self._last_joint_state = None

        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_link', 'tcp_link')
        self.declare_parameter('z_offset', 0.05)
        self.declare_parameter('z_min', 0.02)
        self.declare_parameter('max_reach', 0.25)  # safe horizontal reach from origin
        self.declare_parameter('auto_execute', False)
        self.declare_parameter('ik_timeout_sec', 1.0)
        self.declare_parameter('planning_time_sec', 10.0)
        self.declare_parameter('joint_tolerance', 0.01)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)

        self._group = self.get_parameter('planning_group').value
        self._base_frame = self.get_parameter('base_frame').value
        self._tip_link = self.get_parameter('tip_link').value
        self._z_offset = self.get_parameter('z_offset').value
        self._z_min = self.get_parameter('z_min').value
        self._max_reach = self.get_parameter('max_reach').value
        self._auto_execute = self.get_parameter('auto_execute').value
        self._ik_timeout = self.get_parameter('ik_timeout_sec').value
        self._plan_time = self.get_parameter('planning_time_sec').value
        self._joint_tol = self.get_parameter('joint_tolerance').value
        self._vel_scale = self.get_parameter('velocity_scaling').value
        self._acc_scale = self.get_parameter('acceleration_scaling').value

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

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
        self._last_joint_state = msg

    def _target_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received target: X={msg.pose.position.x:.3f}, '
            f'Y={msg.pose.position.y:.3f}, Z={msg.pose.position.z:.3f}')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._base_frame
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y

        target_z = msg.pose.position.z + self._z_offset
        if target_z < self._z_min:
            self.get_logger().warn(
                f'Target Z ({target_z:.3f}m) below safety limit ({self._z_min}m). Clipping.')
            target_z = self._z_min
        goal_pose.pose.position.z = target_z

        radial = (goal_pose.pose.position.x ** 2 + goal_pose.pose.position.y ** 2) ** 0.5
        if radial > self._max_reach:
            self.get_logger().error(
                f'Target out of reach: radial={radial:.3f}m > max_reach={self._max_reach}m. '
                'Check perception calibration (depth / hand-eye). Skipping goal.')
            return

        # Orientation is ignored by pick_ik (rotation_scale=0.0). Any value works
        # as a seed; identity keeps things readable.
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Pre-Grasp Pose: Z={goal_pose.pose.position.z:.3f}')

        self._request_ik(goal_pose)

    def _request_ik(self, pose: PoseStamped):
        """Fire async IK request; result is handled in _on_ik_done."""
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/compute_ik service not available.')
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group
        req.ik_request.ik_link_name = self._tip_link
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = int(self._ik_timeout)
        req.ik_request.timeout.nanosec = int(
            (self._ik_timeout - int(self._ik_timeout)) * 1e9)

        if self._last_joint_state is not None:
            req.ik_request.robot_state.joint_state = self._last_joint_state

        future = self._ik_client.call_async(req)
        future.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'IK service call failed: {exc}')
            return

        if result is None:
            self.get_logger().error('IK service returned None.')
            return

        if result.error_code.val != 1:
            self.get_logger().warn(
                f'IK failed: error_code={result.error_code.val} — target unreachable.')
            return

        js = result.solution.joint_state
        joint_log = ', '.join(f'{n}={p:.3f}' for n, p in zip(js.name, js.position))
        self.get_logger().info(f'IK solution: {joint_log}')

        if self._auto_execute:
            self._send_joint_goal(js)
        else:
            self.get_logger().info('Auto-Execute OFF. Set -p auto_execute:=true to enable.')

    def _send_joint_goal(self, joint_state: JointState):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveIt Action Server (move_action) not available.')
            return

        constraints = Constraints()
        constraints.name = 'ik_joint_goal'
        for name, pos in zip(joint_state.name, joint_state.position):
            if not name.startswith('joint_'):
                continue
            if name in ('joint_5', 'joint_5_mimic'):
                continue  # gripper not part of arm group
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = self._joint_tol
            jc.tolerance_below = self._joint_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self._group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = self._plan_time
        goal_msg.request.max_velocity_scaling_factor = self._vel_scale
        goal_msg.request.max_acceleration_scaling_factor = self._acc_scale
        goal_msg.request.goal_constraints.append(constraints)

        if self._last_joint_state is not None:
            goal_msg.request.start_state.joint_state = self._last_joint_state
            goal_msg.request.start_state.is_diff = False

        goal_msg.planning_options.plan_only = False

        self.get_logger().info('Sending joint goal to MoveIt...')
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

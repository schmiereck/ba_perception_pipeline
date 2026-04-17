#!/usr/bin/env python3
"""Ground-truth diagnostic for the perception pipeline.

For each known object position (in robot frame, measured with a ruler):
  1. send a detect_request with the given prompt
  2. listen on /ba_perception/target_pose (robot frame) and
     /ba_perception/target_pose_cam (camera frame)
  3. compute expected camera-frame position from ground-truth via
     inverse of hand_eye_transform
  4. print a comparison table — deltas in both frames

Where the errors land tells us which stage is broken:
  * cam-frame delta large -> depth estimation / calibration
  * cam-frame OK, robot-frame delta large -> hand-eye transform

Usage (single point):
    python3 diagnose_perception.py \
        --prompt "the red cup" --x 0.17 --y 0.0 --z 0.10

Interactive (multiple points, CSV output):
    python3 diagnose_perception.py --prompt "the red cup" --interactive \
        --csv /tmp/perception_diag.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'perception_pipeline.yaml'


def load_hand_eye(config_path: Path) -> np.ndarray:
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    params = cfg['/**']['ros__parameters']
    flat = params['hand_eye_transform']
    return np.array(flat, dtype=np.float64).reshape(4, 4)


def robot_to_cam(T_robot_cam: np.ndarray, p_robot: np.ndarray) -> np.ndarray:
    """Inverse of the 4x4 hand-eye transform applied to a 3D point."""
    R = T_robot_cam[:3, :3]
    t = T_robot_cam[:3, 3]
    return R.T @ (p_robot - t)


class DiagNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_diagnose')
        self._pose_robot: PoseStamped | None = None
        self._pose_cam: PoseStamped | None = None
        self._status: str | None = None

        self._req_pub = self.create_publisher(
            String, '/ba_perception/detect_request', 10)
        self.create_subscription(
            PoseStamped, '/ba_perception/target_pose',
            self._on_pose_robot, 10)
        self.create_subscription(
            PoseStamped, '/ba_perception/target_pose_cam',
            self._on_pose_cam, 10)
        self.create_subscription(
            String, '/ba_perception/status', self._on_status, 10)

    def _on_pose_robot(self, msg: PoseStamped) -> None:
        self._pose_robot = msg

    def _on_pose_cam(self, msg: PoseStamped) -> None:
        self._pose_cam = msg

    def _on_status(self, msg: String) -> None:
        self._status = msg.data

    def reset(self) -> None:
        self._pose_robot = None
        self._pose_cam = None
        self._status = None

    def trigger(self, prompt: str) -> None:
        msg = String()
        msg.data = prompt
        # Give subscribers on the pipeline side a moment after reset.
        time.sleep(0.2)
        self._req_pub.publish(msg)

    def wait_for_result(self, timeout: float = 30.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._status and self._status.startswith('ERROR'):
                return False
            if self._pose_robot is not None and self._pose_cam is not None:
                return True
        return False


def measure(
    node: DiagNode,
    T_robot_cam: np.ndarray,
    prompt: str,
    gt_robot: np.ndarray,
) -> dict | None:
    node.reset()
    node.trigger(prompt)
    if not node.wait_for_result():
        node.get_logger().error(
            f'No result received. Last status: {node._status}')
        return None

    meas_robot = np.array([
        node._pose_robot.pose.position.x,
        node._pose_robot.pose.position.y,
        node._pose_robot.pose.position.z,
    ])
    meas_cam = np.array([
        node._pose_cam.pose.position.x,
        node._pose_cam.pose.position.y,
        node._pose_cam.pose.position.z,
    ])
    exp_cam = robot_to_cam(T_robot_cam, gt_robot)

    return {
        'gt_robot': gt_robot,
        'meas_robot': meas_robot,
        'exp_cam': exp_cam,
        'meas_cam': meas_cam,
        'delta_robot': meas_robot - gt_robot,
        'delta_cam': meas_cam - exp_cam,
        'status': node._status,
    }


def print_row(r: dict) -> None:
    print()
    print(f'Status: {r["status"]}')
    print('                     X         Y         Z   (meters)')
    print(f'  GT  (robot):  {r["gt_robot"][0]:+8.4f}  {r["gt_robot"][1]:+8.4f}  {r["gt_robot"][2]:+8.4f}')
    print(f'  Meas(robot):  {r["meas_robot"][0]:+8.4f}  {r["meas_robot"][1]:+8.4f}  {r["meas_robot"][2]:+8.4f}')
    print(f'  Δ   (robot):  {r["delta_robot"][0]:+8.4f}  {r["delta_robot"][1]:+8.4f}  {r["delta_robot"][2]:+8.4f}'
          f'   |Δ|={np.linalg.norm(r["delta_robot"]):.4f}')
    print(f'  Exp (cam):    {r["exp_cam"][0]:+8.4f}  {r["exp_cam"][1]:+8.4f}  {r["exp_cam"][2]:+8.4f}')
    print(f'  Meas(cam):    {r["meas_cam"][0]:+8.4f}  {r["meas_cam"][1]:+8.4f}  {r["meas_cam"][2]:+8.4f}')
    print(f'  Δ   (cam):    {r["delta_cam"][0]:+8.4f}  {r["delta_cam"][1]:+8.4f}  {r["delta_cam"][2]:+8.4f}'
          f'   |Δ|={np.linalg.norm(r["delta_cam"]):.4f}')


def ask_point() -> np.ndarray | None:
    raw = input('Ground-truth robot XYZ in meters (e.g. "0.17 0.0 0.10"), empty to stop: ').strip()
    if not raw:
        return None
    parts = raw.replace(',', ' ').split()
    if len(parts) != 3:
        print('Need 3 numbers.')
        return ask_point()
    try:
        gt = np.array([float(p) for p in parts])
    except ValueError:
        print('Could not parse numbers.')
        return ask_point()
    input(f'Place object at ({gt[0]:.3f}, {gt[1]:.3f}, {gt[2]:.3f})  — press Enter to MEASURE: ')
    return gt


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--prompt', required=True, help='VLM prompt')
    p.add_argument('--x', type=float, help='Ground-truth X in robot frame [m]')
    p.add_argument('--y', type=float, help='Ground-truth Y in robot frame [m]')
    p.add_argument('--z', type=float, help='Ground-truth Z in robot frame [m]')
    p.add_argument('--interactive', action='store_true',
                   help='Loop: ask for more points until empty input')
    p.add_argument('--csv', type=str, default='',
                   help='Optional CSV output file')
    p.add_argument('--config', type=str, default=str(DEFAULT_CONFIG),
                   help='Path to perception_pipeline.yaml (for hand_eye_transform)')
    args = p.parse_args()

    if not args.interactive and (args.x is None or args.y is None or args.z is None):
        print('Provide --x --y --z or use --interactive.', file=sys.stderr)
        return 2

    T = load_hand_eye(Path(args.config))
    print(f'Loaded hand_eye_transform from {args.config}:')
    print(T)
    print()
    print('Robot frame convention (ROS / base_link):')
    print('  +X = forward   +Y = LEFT   +Z = up')
    print('  Enter positions in meters using this convention.')
    print()

    rclpy.init()
    node = DiagNode()

    results: list[dict] = []

    try:
        if args.interactive:
            while True:
                gt = ask_point()
                if gt is None:
                    break
                r = measure(node, T, args.prompt, gt)
                if r is not None:
                    print_row(r)
                    results.append(r)
        else:
            gt = np.array([args.x, args.y, args.z])
            r = measure(node, T, args.prompt, gt)
            if r is not None:
                print_row(r)
                results.append(r)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.csv and results:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'gt_rx', 'gt_ry', 'gt_rz',
                'meas_rx', 'meas_ry', 'meas_rz',
                'dr_x', 'dr_y', 'dr_z',
                'exp_cx', 'exp_cy', 'exp_cz',
                'meas_cx', 'meas_cy', 'meas_cz',
                'dc_x', 'dc_y', 'dc_z',
                'status',
            ])
            for r in results:
                w.writerow([
                    *r['gt_robot'], *r['meas_robot'], *r['delta_robot'],
                    *r['exp_cam'], *r['meas_cam'], *r['delta_cam'],
                    r['status'],
                ])
        print(f'\nCSV written: {args.csv}')

    return 0


if __name__ == '__main__':
    sys.exit(main())

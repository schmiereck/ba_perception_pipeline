#!/usr/bin/env python3
"""Interactive depth calibration script.

Captures checkerboard images at various distances, computes metric depth
via solvePnP, runs DA V2 inference for relative depth, and fits a
mapping function Z_metric = f(d_relative).

Works headless (no GUI) — captures are triggered by pressing ENTER in
the terminal.  Debug images are saved to disk for visual verification.

Usage
-----
::

    source ~/venvs/ba_depth_node/bin/activate
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml
    source ~/ros2_ws/install/setup.bash

    python3 scripts/calibrate_depth.py \\
        --board-size 9x7 \\
        --square-size 0.025 \\
        --output config/depth_calibration.yaml
"""

import argparse
import datetime
import os
import sys
import threading
import time

import cv2
import numpy as np
import yaml

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage

# Depth estimator (from ba_depth_node)
from ba_depth_node.depth_estimator import DepthEstimator


# -----------------------------------------------------------------------
# Fitting
# -----------------------------------------------------------------------

def _fit_inverse(d: np.ndarray, Z: np.ndarray):
    """Fit Z = a / d + b."""
    # Avoid division by zero
    mask = d > 1e-4
    d, Z = d[mask], Z[mask]
    # Linear regression on (1/d, Z): Z = a * (1/d) + b
    A = np.column_stack([1.0 / d, np.ones(len(d))])
    result, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    a, b = result
    Z_pred = a / d + b
    rmse = float(np.sqrt(np.mean((Z - Z_pred) ** 2)))
    return 'inverse', float(a), float(b), rmse


def _fit_inverse_no_offset(d: np.ndarray, Z: np.ndarray):
    """Fit Z = a / d (no offset)."""
    mask = d > 1e-4
    d, Z = d[mask], Z[mask]
    A = (1.0 / d).reshape(-1, 1)
    result, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    a = result[0]
    Z_pred = a / d
    rmse = float(np.sqrt(np.mean((Z - Z_pred) ** 2)))
    return 'inverse_no_offset', float(a), 0.0, rmse


def _fit_linear(d: np.ndarray, Z: np.ndarray):
    """Fit Z = a * d + b."""
    A = np.column_stack([d, np.ones(len(d))])
    result, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    a, b = result
    Z_pred = a * d + b
    rmse = float(np.sqrt(np.mean((Z - Z_pred) ** 2)))
    return 'linear', float(a), float(b), rmse


def fit_best_model(d_all: np.ndarray, Z_all: np.ndarray):
    """Fit all candidate models and return the best one."""
    models = [
        _fit_inverse(d_all, Z_all),
        _fit_inverse_no_offset(d_all, Z_all),
        _fit_linear(d_all, Z_all),
    ]
    # Sort by RMSE
    models.sort(key=lambda m: m[3])
    best = models[0]
    return {
        'model_type': best[0],
        'a': best[1],
        'b': best[2],
        'rmse_m': best[3],
    }, models


# -----------------------------------------------------------------------
# Calibration Node
# -----------------------------------------------------------------------

class DepthCalibrationNode(Node):
    """ROS2 node for interactive depth calibration."""

    def __init__(
        self,
        board_cols: int,
        board_rows: int,
        square_size: float,
        output_path: str,
        debug_dir: str,
        depth_model_id: str,
        depth_device: str,
    ) -> None:
        super().__init__('depth_calibration')

        self._board_size = (board_cols, board_rows)
        self._square_size = square_size
        self._output_path = output_path
        self._debug_dir = debug_dir

        # 3D object points for the checkerboard (Z=0 plane)
        self._obj_points = np.zeros(
            (board_cols * board_rows, 3), dtype=np.float32)
        for i in range(board_rows):
            for j in range(board_cols):
                self._obj_points[i * board_cols + j] = [
                    j * square_size, i * square_size, 0.0]

        # Collected samples: list of (d_relative[], Z_metric[]) per capture
        self._samples_d: list[np.ndarray] = []
        self._samples_Z: list[np.ndarray] = []
        self._capture_count = 0

        # Latest frame + camera intrinsics
        self._latest_bgr: np.ndarray | None = None
        self._latest_stamp = None
        self._frame_lock = threading.Lock()
        self._K: np.ndarray | None = None
        self._dist: np.ndarray | None = None

        # Rectification
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None

        # Depth estimator
        self._depth = DepthEstimator(
            model_id=depth_model_id, device=depth_device,
            log_fn=self.get_logger().info)

        # Subscribers
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage,
            '/ba_overview_camera/image_raw/compressed',
            self._image_cb, image_qos)

        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CameraInfo,
            '/ba_overview_camera/camera_info',
            self._camera_info_cb, info_qos)

        self.get_logger().info(
            f'Depth calibration ready. Board: {board_cols}x{board_rows}, '
            f'square: {square_size*1000:.0f}mm')
        self.get_logger().info('Waiting for camera frames + camera_info ...')

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._K is not None:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist = np.array(msg.d, dtype=np.float64)
        w, h = msg.width, msg.height

        # Build rectification maps
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            self._K, self._dist, None, self._K, (w, h), cv2.CV_32FC1)

        self.get_logger().info(
            f'Camera intrinsics received: {w}x{h}, '
            f'fx={self._K[0,0]:.1f}, fy={self._K[1,1]:.1f}')

    def _image_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        # Rectify
        if self._map_x is not None:
            bgr = cv2.remap(
                bgr, self._map_x, self._map_y, cv2.INTER_LINEAR)

        with self._frame_lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp

    def get_preview(self) -> np.ndarray | None:
        """Return current frame with checkerboard detection overlay."""
        with self._frame_lock:
            bgr = self._latest_bgr
        if bgr is None:
            return None

        preview = bgr.copy()
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 | cv2.CALIB_CB_NORMALIZE_IMAGE
                 | cv2.CALIB_CB_FAST_CHECK)
        found, corners = cv2.findChessboardCorners(
            gray, self._board_size, flags)

        if found:
            cv2.drawChessboardCorners(
                preview, self._board_size, corners, found)
            label = f'DETECTED ({self._board_size[0]}x{self._board_size[1]}) - press SPACE'
            color = (0, 255, 0)
        else:
            label = 'No checkerboard detected'
            color = (0, 0, 255)

        cv2.putText(preview, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(preview, f'Captures: {self._capture_count}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(preview, 'SPACE=capture  Q=finish', (10, preview.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return preview

    def is_ready(self) -> bool:
        return self._latest_bgr is not None and self._K is not None

    def capture(self) -> bool:
        """Capture one calibration sample. Returns True on success."""
        with self._frame_lock:
            bgr = self._latest_bgr
        if bgr is None:
            self.get_logger().error('No frame available')
            return False
        if self._K is None:
            self.get_logger().error('No camera intrinsics available')
            return False

        # Frame is already rectified in _image_cb
        bgr = bgr.copy()  # avoid modifying the buffered frame

        # Detect checkerboard
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 | cv2.CALIB_CB_NORMALIZE_IMAGE
                 | cv2.CALIB_CB_FAST_CHECK)
        found, corners = cv2.findChessboardCorners(
            gray, self._board_size, flags)

        if not found:
            self.get_logger().warn('Checkerboard not detected — try again')
            # Save failed frame for debugging
            if self._debug_dir:
                cv2.imwrite(
                    os.path.join(self._debug_dir, 'calib_failed.png'), bgr)
            return False

        # Refine corners to subpixel
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   criteria)

        # solvePnP → rotation + translation of board in camera frame
        # Use rectified K (same as original K after undistortRectifyMap
        # with newCameraMatrix=K), dist=0 since image is rectified.
        success, rvec, tvec = cv2.solvePnP(
            self._obj_points, corners, self._K, None)
        if not success:
            self.get_logger().warn('solvePnP failed')
            return False

        # Transform all object points to camera frame
        R, _ = cv2.Rodrigues(rvec)
        points_cam = (R @ self._obj_points.T).T + tvec.T  # (N, 3)
        Z_metric = points_cam[:, 2].astype(np.float64)  # depth in meters

        # Run depth estimation
        t0 = time.perf_counter()
        depth_f32 = self._depth.estimate(bgr)
        dt = (time.perf_counter() - t0) * 1000.0

        # Extract relative depth at each corner
        corners_int = corners.reshape(-1, 2).astype(int)
        h, w = depth_f32.shape
        d_relative = np.array([
            depth_f32[
                np.clip(int(pt[1]), 0, h - 1),
                np.clip(int(pt[0]), 0, w - 1)]
            for pt in corners_int
        ], dtype=np.float64)

        # Filter out invalid samples (d ≈ 0 or Z ≤ 0)
        valid = (d_relative > 1e-4) & (Z_metric > 0.01)
        if valid.sum() < 10:
            self.get_logger().warn(
                f'Too few valid samples ({valid.sum()}) — try again')
            return False

        self._samples_d.append(d_relative[valid])
        self._samples_Z.append(Z_metric[valid])
        self._capture_count += 1

        Z_mean = float(Z_metric[valid].mean())
        Z_min = float(Z_metric[valid].min())
        Z_max = float(Z_metric[valid].max())
        d_mean = float(d_relative[valid].mean())

        self.get_logger().info(
            f'Capture #{self._capture_count}: '
            f'{valid.sum()} points, '
            f'Z={Z_min:.3f}–{Z_max:.3f}m (mean {Z_mean:.3f}m), '
            f'd_rel mean={d_mean:.3f}, '
            f'depth inference {dt:.0f}ms')

        # Save debug images
        if self._debug_dir:
            # Annotated image with detected corners
            annotated = bgr.copy()
            cv2.drawChessboardCorners(
                annotated, self._board_size, corners, found)
            cv2.putText(
                annotated,
                f'#{self._capture_count} Z={Z_mean:.3f}m d={d_mean:.3f}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite(
                os.path.join(
                    self._debug_dir,
                    f'calib_capture_{self._capture_count:02d}.png'),
                annotated)

            # Depth image
            depth_uint8 = (depth_f32 * 255.0).astype(np.uint8)
            cv2.imwrite(
                os.path.join(
                    self._debug_dir,
                    f'calib_depth_{self._capture_count:02d}.png'),
                depth_uint8)

        return True

    def compute_calibration(self) -> dict | None:
        """Fit the calibration model and save results."""
        if not self._samples_d:
            self.get_logger().error('No samples collected')
            return None

        d_all = np.concatenate(self._samples_d)
        Z_all = np.concatenate(self._samples_Z)

        self.get_logger().info(
            f'Fitting with {len(d_all)} points from '
            f'{self._capture_count} captures ...')

        best, all_models = fit_best_model(d_all, Z_all)

        # Log all models
        for model_type, a, b, rmse in all_models:
            marker = ' ← BEST' if model_type == best['model_type'] else ''
            self.get_logger().info(
                f'  {model_type}: a={a:.6f}, b={b:.6f}, '
                f'RMSE={rmse*100:.2f}cm{marker}')

        # Add metadata
        best['n_samples'] = int(len(d_all))
        best['n_captures'] = self._capture_count
        best['created'] = datetime.datetime.now().isoformat()
        best['depth_range_m'] = [
            float(Z_all.min()), float(Z_all.max())]
        best['d_range'] = [
            float(d_all.min()), float(d_all.max())]

        # Save YAML
        os.makedirs(os.path.dirname(self._output_path) or '.', exist_ok=True)
        with open(self._output_path, 'w') as f:
            f.write('# Metric depth calibration for DA V2 Small\n')
            f.write(f'# Created: {best["created"]}\n')
            f.write(f'# Model: Z_metric = a / d_relative + b '
                    f'(if inverse)\n')
            f.write(f'#         Z_metric = a * d_relative + b '
                    f'(if linear)\n')
            yaml.dump(best, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(
            f'Calibration saved to {self._output_path}')
        self.get_logger().info(
            f'Best model: {best["model_type"]}, '
            f'a={best["a"]:.6f}, b={best["b"]:.6f}, '
            f'RMSE={best["rmse_m"]*100:.2f}cm')

        # Save scatter plot as PNG (matplotlib optional)
        if self._debug_dir:
            self._save_plot(d_all, Z_all, best)

        return best

    def _save_plot(
        self, d: np.ndarray, Z: np.ndarray, cal: dict
    ) -> None:
        """Save calibration scatter + fit curve as PNG."""
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn(
                'matplotlib not installed — skipping plot')
            return

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.scatter(d, Z, s=3, alpha=0.3, label='data')

        # Plot fit curve
        d_sorted = np.sort(d)
        if cal['model_type'] == 'linear':
            Z_fit = cal['a'] * d_sorted + cal['b']
        else:
            Z_fit = cal['a'] / np.maximum(d_sorted, 1e-6) + cal['b']
        ax.plot(d_sorted, Z_fit, 'r-', linewidth=2,
                label=f'{cal["model_type"]}: '
                      f'a={cal["a"]:.4f}, b={cal["b"]:.4f}\n'
                      f'RMSE={cal["rmse_m"]*100:.1f}cm')

        ax.set_xlabel('d_relative (DA V2 output)')
        ax.set_ylabel('Z_metric (m)')
        ax.set_title('Depth Calibration')
        ax.legend()
        ax.grid(True, alpha=0.3)

        path = os.path.join(self._debug_dir, 'calibration_plot.png')
        fig.savefig(path, dpi=120, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'Plot saved to {path}')


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Interactive depth calibration with checkerboard')
    parser.add_argument(
        '--board-size', default='9x7',
        help='Number of squares, e.g. 9x7 (cols x rows). '
             'Inner corners = (cols-1) x (rows-1).')
    parser.add_argument(
        '--square-size', type=float, default=0.025,
        help='Square size in meters (default: 0.025 = 25mm)')
    parser.add_argument(
        '--output', default='config/depth_calibration.yaml',
        help='Output YAML path')
    parser.add_argument(
        '--debug-dir', default='/tmp/depth_calibration',
        help='Directory for debug images (default: /tmp/depth_calibration)')
    parser.add_argument(
        '--model-id', default='depth-anything/Depth-Anything-V2-Small-hf',
        help='Depth model HuggingFace ID')
    parser.add_argument(
        '--device', default='cpu',
        help='PyTorch device')
    args = parser.parse_args()

    # board-size is in squares; OpenCV wants inner corners = squares - 1
    sq_cols, sq_rows = [int(x) for x in args.board_size.split('x')]
    cols, rows = sq_cols - 1, sq_rows - 1

    os.makedirs(args.debug_dir, exist_ok=True)

    rclpy.init()
    node = DepthCalibrationNode(
        board_cols=cols,
        board_rows=rows,
        square_size=args.square_size,
        output_path=args.output,
        debug_dir=args.debug_dir,
        depth_model_id=args.model_id,
        depth_device=args.device,
    )

    # Spin in background thread so main thread can run the GUI loop
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for camera
    print('\nWaiting for camera frames and camera_info ...')
    while not node.is_ready():
        time.sleep(0.2)
    print('Camera ready!\n')

    print('=' * 60)
    print('DEPTH CALIBRATION')
    print(f'Board: {sq_cols}x{sq_rows} squares '
          f'({cols}x{rows} inner corners), '
          f'square: {args.square_size*1000:.0f}mm')
    print(f'Output: {args.output}')
    print(f'Debug images: {args.debug_dir}')
    print()
    print('Hold the checkerboard at different distances from the camera.')
    print('Aim for 5-6 different distances covering your workspace range.')
    print()
    print('  SPACE  = capture sample')
    print('  Q      = finish and compute calibration')
    print('  ESC    = abort')
    print('=' * 60)
    print()

    WINDOW_NAME = 'Depth Calibration'
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 800, 600)

    aborted = False
    try:
        while True:
            preview = node.get_preview()
            if preview is not None:
                cv2.imshow(WINDOW_NAME, preview)

            key = cv2.waitKey(100) & 0xFF  # ~10 FPS refresh

            if key == ord(' ') or key == 13:  # SPACE or ENTER
                node.capture()
            elif key == ord('q') or key == ord('Q'):
                if node._capture_count < 2:
                    print('Need at least 2 captures. Keep going.')
                    continue
                break
            elif key == 27:  # ESC
                aborted = True
                break

    except KeyboardInterrupt:
        aborted = True

    cv2.destroyAllWindows()

    if aborted:
        print('\nAborted.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print('\nComputing calibration ...\n')
    result = node.compute_calibration()

    if result:
        print(f'\n{"=" * 60}')
        print(f'CALIBRATION COMPLETE')
        print(f'  Model:  {result["model_type"]}')
        print(f'  a = {result["a"]:.6f}')
        print(f'  b = {result["b"]:.6f}')
        print(f'  RMSE = {result["rmse_m"]*100:.2f} cm')
        print(f'  Saved to: {args.output}')
        print(f'  Plot: {args.debug_dir}/calibration_plot.png')
        print(f'{"=" * 60}')
    else:
        print('Calibration failed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

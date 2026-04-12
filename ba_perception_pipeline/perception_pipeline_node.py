#!/usr/bin/env python3
"""ROS2 perception pipeline node.

Combines depth estimation, VLM target detection, and backprojection
into a single in-process node.  The large depth map (numpy array) is
passed directly between stages — never serialised over DDS — which
avoids the CycloneDDS WSL2 loopback limitation for large messages.

Trigger interface (topic-based):
    Subscribe: ``/perception/detect_request``  (``std_msgs/String``)
               — the text prompt, e.g. "the red cup"
    Publish:   ``/perception/target_pose``     (``geometry_msgs/PoseStamped``)
               — 3D target in the robot frame
    Publish:   ``/perception/status``          (``std_msgs/String``)
               — status/error feedback per request
"""

import os
import threading
import time

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from ba_depth_node.depth_estimator import DepthEstimator
from ba_perception_pipeline.backprojection import backproject, scale_depth
from ba_perception_pipeline.vlm_client import VLMClient


class PerceptionPipelineNode(Node):
    """Single-process perception pipeline."""

    def __init__(self) -> None:
        super().__init__('perception_pipeline')

        # -- parameters --------------------------------------------------
        self.declare_parameter('image_topic',
                               '/ba_overview_camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic',
                               '/ba_overview_camera/camera_info')
        self.declare_parameter('request_topic',
                               '/perception/detect_request')
        self.declare_parameter('pose_topic',
                               '/perception/target_pose')
        self.declare_parameter('status_topic',
                               '/perception/status')
        self.declare_parameter('depth_debug_topic',
                               '/ba_overview_camera/depth/image_raw')

        # Depth model
        self.declare_parameter('depth_model_id',
                               'depth-anything/Depth-Anything-V2-Small-hf')
        self.declare_parameter('depth_device', 'cpu')

        # VLM
        self.declare_parameter('vlm_provider', 'groq')
        self.declare_parameter('vlm_model', 'meta-llama/llama-4-scout-17b-16e-instruct')
        self.declare_parameter('vlm_api_key', '')

        # Rectification
        self.declare_parameter('rectify', True)

        # Hand-eye transform (4x4 column-major, camera→robot)
        self.declare_parameter('hand_eye_transform', [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ])

        # Depth calibration
        self.declare_parameter('depth_calibration_file', '')

        # Debug
        self.declare_parameter('debug_save_path', '')

        # Read parameters
        image_topic = self.get_parameter('image_topic') \
            .get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic') \
            .get_parameter_value().string_value
        request_topic = self.get_parameter('request_topic') \
            .get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic') \
            .get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic') \
            .get_parameter_value().string_value
        depth_debug_topic = self.get_parameter('depth_debug_topic') \
            .get_parameter_value().string_value

        depth_model_id = self.get_parameter('depth_model_id') \
            .get_parameter_value().string_value
        depth_device = self.get_parameter('depth_device') \
            .get_parameter_value().string_value

        vlm_provider = self.get_parameter('vlm_provider') \
            .get_parameter_value().string_value
        vlm_model = self.get_parameter('vlm_model') \
            .get_parameter_value().string_value
        vlm_api_key = self.get_parameter('vlm_api_key') \
            .get_parameter_value().string_value
        # Fall back to environment variable if parameter is empty.
        if not vlm_api_key:
            env_key = 'GROQ_API_KEY' if vlm_provider == 'groq' else 'GEMINI_API_KEY'
            vlm_api_key = os.environ.get(env_key, '')

        self._rectify = self.get_parameter('rectify') \
            .get_parameter_value().bool_value
        self._debug_save_path = self.get_parameter('debug_save_path') \
            .get_parameter_value().string_value

        # Hand-eye transform: 16 floats (column-major) → 4x4 matrix
        he_flat = self.get_parameter('hand_eye_transform') \
            .get_parameter_value().double_array_value
        self._T_robot_cam = np.array(he_flat, dtype=np.float64).reshape(4, 4)

        # -- depth calibration (relative → metric) -----------------------
        depth_cal_file = self.get_parameter('depth_calibration_file') \
            .get_parameter_value().string_value
        self._depth_cal = None
        if depth_cal_file:
            try:
                with open(depth_cal_file, 'r') as f:
                    self._depth_cal = yaml.safe_load(f)
                self.get_logger().info(
                    f'Depth calibration loaded: '
                    f'{self._depth_cal["model_type"]}, '
                    f'a={self._depth_cal["a"]:.6f}, '
                    f'b={self._depth_cal["b"]:.6f}, '
                    f'RMSE={self._depth_cal["rmse_m"]*100:.1f}cm')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load depth calibration: {e}')
        else:
            self.get_logger().warn(
                'No depth_calibration_file set — using relative depth')

        # -- depth estimator (in-process, no DDS) -------------------------
        self._depth_estimator = DepthEstimator(
            model_id=depth_model_id, device=depth_device,
            log_fn=self.get_logger().info)

        # -- VLM client ---------------------------------------------------
        if vlm_api_key:
            self._vlm_client = VLMClient(
                provider=vlm_provider, model=vlm_model,
                api_key=vlm_api_key,
                log_fn=self.get_logger().info)
        else:
            self._vlm_client = None
            self.get_logger().warn(
                'No vlm_api_key set — VLM calls will fail. '
                'Set vlm_api_key in the config YAML.')

        # -- rectification state ------------------------------------------
        self._K: np.ndarray | None = None  # 3x3 intrinsic matrix
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None

        # -- latest frame buffer ------------------------------------------
        self._latest_bgr: np.ndarray | None = None
        self._latest_stamp = None
        self._latest_frame_id: str = ''
        self._frame_lock = threading.Lock()

        # -- busy flag (only one pipeline run at a time) ------------------
        self._busy = False
        self._busy_lock = threading.Lock()

        # -- publishers ---------------------------------------------------
        self._pose_pub = self.create_publisher(
            PoseStamped, pose_topic, 10)
        self._status_pub = self.create_publisher(
            String, status_topic, 10)

        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._depth_pub = self.create_publisher(
            Image, depth_debug_topic, depth_qos)

        # -- subscribers --------------------------------------------------
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_sub = self.create_subscription(
            CompressedImage, image_topic, self._image_cb, image_qos)

        if self._rectify:
            info_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._camera_info_sub = self.create_subscription(
                CameraInfo, camera_info_topic,
                self._camera_info_cb, info_qos)
            self.get_logger().info(
                f'Waiting for camera_info on {camera_info_topic} ...')
        else:
            self._camera_info_sub = None

        # Request trigger
        self._request_sub = self.create_subscription(
            String, request_topic, self._request_cb, 10)

        self.get_logger().info(
            f'Perception pipeline ready. '
            f'Send a prompt to {request_topic} to trigger detection.')

    # -----------------------------------------------------------------
    # camera_info → build rectification LUT + store K
    # -----------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._map_x is not None:
            return  # already computed

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        w, h = msg.width, msg.height

        self._K = K.copy()
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            K, D, None, K, (w, h), cv2.CV_32FC1)

        self.get_logger().info(
            f'Rectification LUT built for {w}x{h} '
            f'(fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, '
            f'cx={K[0,2]:.1f}, cy={K[1,2]:.1f})')

        if self._camera_info_sub is not None:
            self.destroy_subscription(self._camera_info_sub)
            self._camera_info_sub = None

    # -----------------------------------------------------------------
    # image callback → buffer latest frame
    # -----------------------------------------------------------------
    def _image_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        with self._frame_lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp
            self._latest_frame_id = msg.header.frame_id

    # -----------------------------------------------------------------
    # detection request → run full pipeline
    # -----------------------------------------------------------------
    def _request_cb(self, msg: String) -> None:
        prompt = msg.data.strip()
        if not prompt:
            self._publish_status('ERROR: empty prompt')
            return

        with self._busy_lock:
            if self._busy:
                self._publish_status('BUSY: pipeline already running')
                return
            self._busy = True

        try:
            self._run_pipeline(prompt)
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            self._publish_status(f'ERROR: {e}')
        finally:
            with self._busy_lock:
                self._busy = False

    def _run_pipeline(self, prompt: str) -> None:
        t0 = time.perf_counter()

        # 1. Grab latest frame -------------------------------------------
        with self._frame_lock:
            bgr = self._latest_bgr
            stamp = self._latest_stamp
            frame_id = self._latest_frame_id

        if bgr is None:
            self._publish_status('ERROR: no camera frame received yet')
            return

        self.get_logger().info(
            f'Pipeline started: "{prompt}" '
            f'(frame {bgr.shape[1]}x{bgr.shape[0]})')

        # 2. Rectify -----------------------------------------------------
        if self._rectify and self._map_x is not None:
            bgr = cv2.remap(
                bgr, self._map_x, self._map_y, cv2.INTER_LINEAR)

        # 3. Depth estimation (in-process, no DDS) -----------------------
        t_depth = time.perf_counter()
        depth_f32 = self._depth_estimator.estimate(bgr)
        dt_depth = (time.perf_counter() - t_depth) * 1000.0
        self.get_logger().info(f'Depth: {dt_depth:.0f} ms')

        # Publish debug depth image
        self._publish_depth_debug(depth_f32, stamp, frame_id)

        # 4. VLM target detection ----------------------------------------
        if self._vlm_client is None:
            self._publish_status('ERROR: VLM client not configured (no API key)')
            return

        t_vlm = time.perf_counter()
        u, v = self._vlm_client.detect_target(bgr, prompt)
        dt_vlm = (time.perf_counter() - t_vlm) * 1000.0
        self.get_logger().info(f'VLM: ({u}, {v}) in {dt_vlm:.0f} ms')

        # Bounds check
        h, w = depth_f32.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            self._publish_status(
                f'ERROR: VLM pixel ({u}, {v}) out of bounds '
                f'({w}x{h})')
            return

        # 5. Backprojection → camera frame -------------------------------
        d_rel = float(depth_f32[v, u])
        if self._K is None:
            self._publish_status(
                'ERROR: camera intrinsics not available (no camera_info)')
            return

        # Scale to metric depth if calibration is available
        if self._depth_cal:
            Z = scale_depth(
                d_rel,
                self._depth_cal['a'],
                self._depth_cal['b'],
                self._depth_cal['model_type'])
            self.get_logger().info(
                f'Depth: d_rel={d_rel:.4f} → Z_metric={Z:.4f}m')
        else:
            Z = d_rel

        point_cam = backproject(u, v, Z, self._K)
        self.get_logger().info(
            f'Backprojection: camera frame '
            f'[{point_cam[0]:.4f}, {point_cam[1]:.4f}, {point_cam[2]:.4f}]')

        # 6. Hand-eye transform → robot frame ----------------------------
        point_cam_h = np.append(point_cam, 1.0)
        point_robot = (self._T_robot_cam @ point_cam_h)[:3]
        self.get_logger().info(
            f'Hand-eye: robot frame '
            f'[{point_robot[0]:.4f}, {point_robot[1]:.4f}, {point_robot[2]:.4f}]')

        # 7. Publish PoseStamped -----------------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = float(point_robot[0])
        pose_msg.pose.position.y = float(point_robot[1])
        pose_msg.pose.position.z = float(point_robot[2])
        # Orientation: identity (no gripper orientation yet)
        pose_msg.pose.orientation.w = 1.0

        self._pose_pub.publish(pose_msg)

        dt_total = (time.perf_counter() - t0) * 1000.0
        status = (
            f'OK: "{prompt}" → pixel ({u},{v}), '
            f'd_rel={d_rel:.3f}, Z={Z:.4f}{"m" if self._depth_cal else ""}, '
            f'robot [{point_robot[0]:.4f}, {point_robot[1]:.4f}, '
            f'{point_robot[2]:.4f}], '
            f'{dt_total:.0f} ms total')
        self._publish_status(status)
        self.get_logger().info(status)

        # Debug: save annotated image
        if self._debug_save_path:
            self._save_debug_images(bgr, depth_f32, u, v, prompt)

    # -----------------------------------------------------------------
    # helpers
    # -----------------------------------------------------------------
    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'Status: {text}')

    def _publish_depth_debug(
        self,
        depth_f32: np.ndarray,
        stamp,
        frame_id: str,
    ) -> None:
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = frame_id
        depth_msg.height, depth_msg.width = depth_f32.shape[:2]
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = bytes(depth_f32.tobytes())
        self._depth_pub.publish(depth_msg)

    def _save_debug_images(
        self,
        bgr: np.ndarray,
        depth_f32: np.ndarray,
        u: int,
        v: int,
        prompt: str,
    ) -> None:
        # Depth PNG
        depth_uint8 = (depth_f32 * 255.0).astype(np.uint8)
        cv2.imwrite(
            os.path.join(self._debug_save_path, 'depth_latest.png'),
            depth_uint8)

        # Annotated camera image with target marker
        annotated = bgr.copy()
        cv2.circle(annotated, (u, v), 10, (0, 0, 255), 2)
        cv2.circle(annotated, (u, v), 2, (0, 0, 255), -1)
        cv2.putText(
            annotated, f'({u},{v}) "{prompt}"',
            (u + 15, v - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 0, 255), 1)
        cv2.imwrite(
            os.path.join(self._debug_save_path, 'target_latest.png'),
            annotated)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

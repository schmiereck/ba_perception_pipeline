# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**ba_perception_pipeline** is the perception orchestrator for the Bracket-Arm 6DOF robot arm. It runs in a **single ROS2 process** on the WSL2 laptop, combining:

1. **Depth estimation** — Depth Anything V2 Small (imported from `ba_depth_node`)
2. **VLM target detection** — Groq (Llama 4 Scout) or Google Gemini
3. **Backprojection** — pixel + depth → 3D point via pinhole model
4. **Hand-eye transform** — camera frame → robot frame

All heavy data (depth maps, images) stays in-process as numpy arrays. Only small messages cross DDS (compressed JPEG in, PoseStamped out). This avoids the CycloneDDS WSL2 loopback limitation with large messages.

## End-to-End Pipeline

```
USB-Kamera (Pi, ubuntu1)
    ↓ CompressedImage (JPEG, ~50 kB, over WLAN)
WSL2 Laptop: perception_pipeline_node (single process)
    │
    ├── Subscribe latest frame (buffer)
    │
    ├── On /perception/detect_request (String prompt):
    │   1. Grab latest frame from buffer
    │   2. Rectify (camera_info intrinsics)
    │   3. DA V2 → depth_map (numpy, ~0.9s)
    │   4. VLM API → target pixel (u, v) (~1-2s)
    │   5. Backproject → (X,Y,Z) camera frame
    │   6. Hand-eye → (X,Y,Z) robot frame
    │   7. Publish PoseStamped
    │
    ↓ PoseStamped (~100 bytes)
MoveIt / Goal-Generator
```

## Related Projects

- `ba_depth_node` — standalone depth publisher (provides `DepthEstimator` class)
- `ba_arduino_controller` — Arduino-side arm control
- `ba_arduino_controller_moveit_config` — MoveIt configuration
- `ba_camera_bridge` — ROS2 USB camera node on Raspberry Pi

## Package Structure

- `ba_perception_pipeline/perception_pipeline_node.py` — main ROS2 node
- `ba_perception_pipeline/vlm_client.py` — VLM API client (Groq/Gemini)
- `ba_perception_pipeline/backprojection.py` — pixel → 3D math
- `config/perception_pipeline.yaml` — all parameters
- `launch/perception_pipeline.launch.py` — launch file

### Topic Contract

| Topic | Type | Direction |
|---|---|---|
| `/ba_overview_camera/image_raw/compressed` | `CompressedImage` | subscribe |
| `/ba_overview_camera/camera_info` | `CameraInfo` | subscribe (once) |
| `/perception/detect_request` | `String` | subscribe (trigger) |
| `/perception/target_pose` | `PoseStamped` | publish |
| `/perception/status` | `String` | publish |
| `/ba_overview_camera/depth/image_raw` | `Image` (32FC1) | publish (debug) |

## Environment

- **Host**: WSL2 Ubuntu-22.04 on Windows dev laptop (CPU-only)
- **Python venv**: `~/venvs/ba_depth_node/` (shared with ba_depth_node) — `--system-site-packages`
- **Key deps**: torch, transformers, groq, google-genai, opencv-python
- **Depends on**: `ba_depth_node` (same colcon workspace, for `DepthEstimator` import)

## Build & Run (WSL2)

```bash
# Additional pip deps (in the existing venv)
source ~/venvs/ba_depth_node/bin/activate
pip install groq google-genai

# ROS2 environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml

# Build both packages (ba_depth_node must be built first)
cd ~/ros2_ws
colcon build --packages-select ba_depth_node ba_perception_pipeline --symlink-install
source install/setup.bash

# Run
ros2 launch ba_perception_pipeline perception_pipeline.launch.py

# Trigger detection (from another terminal)
ros2 topic pub --once /perception/detect_request std_msgs/String "data: 'the red cup'"

# Check result
ros2 topic echo /perception/target_pose
ros2 topic echo /perception/status
```

### VLM API Key

Set the API key in the config YAML or pass as parameter override:
```bash
ros2 launch ba_perception_pipeline perception_pipeline.launch.py \
  --ros-args -p vlm_api_key:="your-key-here"
```

Do **not** commit API keys to git.

## Debug

Debug images are saved to `/tmp/` (configurable via `debug_save_path`):
- `depth_latest.png` — depth map
- `target_latest.png` — camera image with VLM target marked

Access from Windows: `\\wsl.localhost\Ubuntu-22.04\tmp\`

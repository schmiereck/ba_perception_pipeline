# GEMINI.md - ba_perception_pipeline

Instructional context for the `ba_perception_pipeline` ROS 2 project.

## Project Overview
The `ba_perception_pipeline` is a ROS 2 Python package designed for the **Bracket-Arm (6DOF)** robot. It provides a complete 3D perception pipeline that converts 2D camera images into 3D target coordinates in the robot's base frame using Vision-Language Models (VLMs) and Depth Estimation.

### Key Technologies
- **ROS 2 Humble** (running on WSL2 Ubuntu 22.04).
- **Depth Anything V2 (DA V2)**: For relative depth estimation from single images.
- **VLMs (Groq/Llama 3 or Gemini 2.0)**: For object detection via natural language prompts.
- **Backprojection**: Geometric conversion from pixel coordinates + depth to 3D camera coordinates.
- **Hand-Eye Calibration**: Transformation from camera frame to robot base frame.

### Architecture
To circumvent WSL2 loopback limitations with large messages (CycloneDDS), the entire pipeline runs within a **single process** (one ROS 2 node). This allows passing large depth maps as numpy arrays directly between components without serialization.

```
Camera (Pi) ──► perception_pipeline_node (WSL2)
                 ├── 1. Rectification (CameraInfo)
                 ├── 2. Depth Anything V2 (Depth Map)
                 ├── 3. VLM (Target Pixel u,v)
                 ├── 4. Metric Scaling (Calibration)
                 ├── 5. Backprojection (3D Camera Frame)
                 └── 6. Hand-Eye Transform (3D Robot Frame)
                          │
                          ▼
                 /perception/target_pose (PoseStamped)
```

## Related Projects
- `ba_depth_node`: Standalone depth publisher (provides the `DepthEstimator` class used here).
- `ba_camera_bridge`: ROS 2 USB camera node running on the Raspberry Pi (ubuntu1).
- `ba_arduino_controller`: Arduino-side arm control.
- `ba_arduino_controller_moveit_config`: MoveIt configuration for the arm.

## Directory Structure
- `ba_perception_pipeline/`: Main source code.
  - `perception_pipeline_node.py`: Orchestrator node.
  - `vlm_client.py`: Interface for Groq and Gemini APIs.
  - `backprojection.py`: Math for scaling depth and backprojection.
- `config/`: Configuration files.
  - `perception_pipeline.yaml`: Main node parameters (topics, VLM model, Hand-Eye).
  - `depth_calibration.yaml`: Metric depth scaling parameters (linear/inverse model).
- `launch/`: ROS 2 launch files.
- `scripts/`: Utility scripts.
  - `calibrate_depth.py`: Interactive OpenCV tool for metric depth calibration.
- `Docs/`: Project planning and documentation.

## Building and Running

### Prerequisites
- ROS 2 Humble on WSL2.
- A Python virtual environment at `~/venvs/ba_depth_node/` with `--system-site-packages`.
- **Key dependencies**: `torch`, `transformers`, `groq`, `google-genai`, `opencv-python`, `scipy`.
- `ba_depth_node` package must be built in the same workspace.

### Environment Setup (WSL2)
```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml
source ~/venvs/ba_depth_node/bin/activate
```

### Build
```bash
cd ~/ros2_ws
# Ensure both packages are built
colcon build --packages-select ba_depth_node ba_perception_pipeline --symlink-install
source install/setup.bash
```

### Run
```bash
# Set API keys (Do NOT commit these to git!)
export GROQ_API_KEY="your-key"
# Launch the pipeline
ros2 launch ba_perception_pipeline perception_pipeline.launch.py
```

## Usage
1. **Perception Pipeline & Goal Generator starten**:
```bash
ros2 launch ba_perception_pipeline ba_perception.launch.py
```

2. **Detektion auslösen**:
```bash
ros2 topic pub --once /ba_perception/detect_request std_msgs/String "data: 'the red cup'"
```

3. **Ergebnisse prüfen**:
- `/ba_perception/target_pose` — 3D-Ziel in Roboter-Koordinaten.
- Terminal-Output des `ba_goal_generator` — berechnete Pre-Grasp Pose.

## Nodes
- `perception_pipeline_node` (`ba_perception_pipeline`): Orchestrierter Perception-Prozess.
- `goal_generator_node` (`ba_goal_generator`): Wandelt Target-Pose in MoveIt-Ziele um (+10cm Z-Offset).

## Topics (Prefix: /ba_)
| Topic | Typ | Beschreibung |
|---|---|---|
| `/ba_perception/detect_request` | `String` | Trigger mit Textprompt |
| `/ba_perception/target_pose` | `PoseStamped` | 3D-Ziel in `base_link` |
| `/ba_perception/status` | `String` | Statusmeldungen |
| `/ba_overview_camera/image_raw/compressed` | `CompressedImage` | Kamera-Input |

## Hilfsscripte
- `scripts/generate_hand_eye.py`: Berechnet die Hand-Eye-Matrix aus Kamera-Position und Blick-Ziel.
- `scripts/calibrate_depth.py`: Interaktive Tiefenkalibrierung (relativ → metrisch).

## Roadmap / TODOs
- [x] **Farbe & Tiefe**: Farbe im Stream aktiv, metrische Tiefe (RMSE 1.5cm) verifiziert.
- [x] **Hand-Eye-Transform**: Matrix aktiv, liefert realistische Z-Werte (~0-10cm über Tisch).
- [x] **Goal-Generator**: Sicherheitsbegrenzung (z_min) integriert.
- [ ] **Feinjustierung**: VLM-Präzision optimieren (Marker-Offset beheben).
- [ ] **Echte MoveIt-Ansteuerung**: Action-Call finalisieren.

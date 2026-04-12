# ba_perception_pipeline

ROS2 Perception-Pipeline für den Bracket-Arm (6DOF). Kombiniert Depth Estimation,
VLM-Zielerkennung und Backprojection in einem einzigen Prozess.

## Überblick

```
USB-Kamera (Pi) ──JPEG──► perception_pipeline_node (WSL2, ein Prozess)
                              │
                              ├── 1. Frame aus Buffer
                              ├── 2. Rektifizieren (camera_info)
                              ├── 3. Depth Anything V2 → Tiefenkarte
                              ├── 4. VLM (Groq/Gemini) → Zielpixel (u,v)
                              ├── 5. Metrische Skalierung (Kalibrierung)
                              ├── 6. Backprojection → 3D Kamera-Frame
                              └── 7. Hand-Eye → 3D Roboter-Frame
                                        │
                                        ▼
                              /perception/target_pose (PoseStamped)
```

Alle schweren Daten (Depth Map, Bilder) bleiben als numpy-Arrays im Prozess.
Nur kleine Messages gehen über DDS — das umgeht die CycloneDDS-Loopback-
Limitation auf WSL2.

## Voraussetzungen

- ROS2 Humble auf WSL2 Ubuntu 22.04
- Python venv `~/venvs/ba_depth_node/` mit `--system-site-packages`
- `ba_depth_node` im selben colcon-Workspace (liefert `DepthEstimator`)
- Kamera läuft auf Pi (ubuntu1) mit `ba_camera_bridge`

### Python-Abhängigkeiten (im venv)

```bash
source ~/venvs/ba_depth_node/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install transformers pillow opencv-python
pip install groq google-genai
pip install --ignore-installed scipy
```

## Build

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclonedds.xml

cd ~/ros2_ws
colcon build --packages-select ba_depth_node ba_perception_pipeline --symlink-install
source install/setup.bash
```

## Tiefenkalibrierung

DA V2 liefert relative Tiefe (0–1). Für metrische Werte (Meter) muss einmalig
kalibriert werden. Die Kalibrierung kann jederzeit wiederholt werden (z.B. bei
neuem Kamera-Setup oder am Roboter-Arbeitsplatz).

### Durchführung

```bash
python3 ~/ros2_ws/src/ba_perception_pipeline/scripts/calibrate_depth.py \
    --board-size 9x7 \
    --square-size 0.025 \
    --output ~/ros2_ws/src/ba_perception_pipeline/config/depth_calibration.yaml
```

Ein OpenCV-Fenster zeigt das Live-Kamerabild mit Schachbrett-Erkennung:

- **Grün** + bunte Ecken = Schachbrett erkannt
- **Rot** = nicht erkannt (Position/Beleuchtung anpassen)

**Bedienung:**
- **SPACE** — Sample aufnehmen (Schachbrett muss erkannt sein)
- **Q** — Kalibrierung berechnen und speichern (mind. 2 Samples)
- **ESC** — Abbrechen

**Tipps:**
- 5–6 verschiedene Abstände (z.B. 25cm, 35cm, 45cm, 55cm, 70cm)
- Schachbrett langsam und ruhig halten
- Auf grüne Erkennung warten, dann SPACE drücken
- Debug-Bilder in `/tmp/depth_calibration/`

### Ergebnis

Die Kalibrierung wird als `config/depth_calibration.yaml` gespeichert:

```yaml
model_type: linear      # oder "inverse"
a: -0.722               # Z_metric = a * d_relative + b
b:  0.968
rmse_m: 0.062           # Genauigkeit in Metern
```

## Pipeline starten

### Ohne metrische Tiefe (relative Werte)

```bash
export GROQ_API_KEY="your-key"   # oder GEMINI_API_KEY
ros2 launch ba_perception_pipeline perception_pipeline.launch.py
```

### Mit metrischer Tiefe (nach Kalibrierung)

```bash
export GROQ_API_KEY="your-key"
ros2 launch ba_perception_pipeline perception_pipeline.launch.py \
    --ros-args -p depth_calibration_file:="/home/$USER/ros2_ws/src/ba_perception_pipeline/config/depth_calibration.yaml"
```

## Objekt erkennen

Aus einem zweiten Terminal:

```bash
# Erkennung auslösen
ros2 topic pub --once /perception/detect_request std_msgs/String \
    "data: 'the red cup'"

# Ergebnis lesen
ros2 topic echo /perception/target_pose --once
ros2 topic echo /perception/status --once
```

## Topics

| Topic | Typ | Richtung | Beschreibung |
|---|---|---|---|
| `/ba_overview_camera/image_raw/compressed` | CompressedImage | sub | Kamerabild vom Pi |
| `/ba_overview_camera/camera_info` | CameraInfo | sub | Intrinsics (einmalig) |
| `/perception/detect_request` | String | sub | Trigger mit Textprompt |
| `/perception/target_pose` | PoseStamped | pub | 3D-Ziel in Roboter-Frame |
| `/perception/status` | String | pub | OK/ERROR + Details |
| `/ba_overview_camera/depth/image_raw` | Image (32FC1) | pub | Debug: Tiefenkarte |

## VLM-Provider wechseln

In `config/perception_pipeline.yaml`:

```yaml
vlm_provider: "groq"     # oder "gemini"
vlm_model: "meta-llama/llama-4-scout-17b-16e-instruct"  # Groq
# vlm_model: "gemini-2.0-flash"                          # Gemini
```

API-Keys werden aus Umgebungsvariablen gelesen (`GROQ_API_KEY` / `GEMINI_API_KEY`),
wenn `vlm_api_key` im YAML leer ist.

## Debug

Debug-Bilder werden nach `/tmp/` gespeichert (konfigurierbar via `debug_save_path`):

- `depth_latest.png` — Tiefenkarte
- `target_latest.png` — Kamerabild mit markiertem Zielpixel

Zugriff von Windows: `\\wsl.localhost\Ubuntu-22.04\tmp\`

## Performance

| Schritt | Latenz |
|---|---|
| Depth (DA V2 Small, CPU) | ~1200 ms |
| VLM (Groq, Llama 4 Scout) | ~400–600 ms |
| Backprojection + Hand-Eye | < 1 ms |
| **Gesamt** | **~1800–2500 ms** |

## Aktueller Stand

- Perception Pipeline funktioniert end-to-end (Kamera → Depth → VLM → 3D-Punkt)
- Metrische Tiefenkalibrierung durchgeführt (RMSE 6.2cm über 33–68cm)
- Hand-Eye-Transform ist noch Identity — muss mit kalibrierter 4×4-Matrix befüllt werden

## Nächste Schritte

1. **Hand-Eye-Transform einsetzen** — kalibrierte 4×4-Matrix in `config/perception_pipeline.yaml` unter `hand_eye_transform` eintragen
2. **MoveIt-Goal generieren** — aus den 3D-Roboter-Koordinaten ein MoveIt-Ziel erzeugen
3. **Tiefenkalibrierung am Arbeitsplatz wiederholen** — Script erneut ausführen wenn Kamera umpositioniert wird

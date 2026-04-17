# PLAN_AUTOCALIB.md
# Automatische Kalibrierung (Bracket Arm) — Hand-Eye + Tiefe in einem Durchgang

## Kontext

- **Ziel**: Eine einzige, weitgehend automatische Kalibrierungs-Sequenz ersetzt den bisherigen Zweiklang aus (a) manuellem `generate_hand_eye.py` und (b) Schachbrett-Workflow in `scripts/calibrate_depth.py`.
- **Ansatz**: Farbige Marker am Arm → Kamera sieht sie → TF kennt ihre Weltposition exakt. Daraus fallen *gleichzeitig*:
  - Die Hand-Eye-Matrix `T_cam_base` (4×4) für `config/perception_pipeline.yaml`
  - Die Tiefen-Skalierung `(a, b)` für `config/depth_calibration.yaml`
- **Setup**: Statische Kamera (eye-to-hand), 5-DOF Arm mit `grasp_link` (xyz `0 0.01 0.07` unter `gripper_base_link`). ROS 2 Humble, ubuntu1 (Pi) + WSL2-Laptop mit MoveIt.
- **Topic-Präfix**: `ba_` (Bracket-Arm-Konvention).

---

## Grundprinzip — eine Datenquelle, zwei Solver

Jeder Datenpunkt entsteht, während der Arm eine Pose hält, und enthält:

```
P_base  = Markerposition in base_link-Frame     (aus TF, ground truth)
(u, v)  = Markerpixel im rektifizierten Bild    (aus HSV-Detection)
d_rel   = relative DA-V2-Tiefe bei (u, v)       (aus Depth-Map)
K       = Kamera-Intrinsik                       (aus camera_info, einmalig)
```

Aus N solcher Tripel `{P_base, (u,v), d_rel}` lösen wir **sequenziell**:

```
Schritt 1: solvePnP(P_base, (u,v), K)           →  T_cam_base   (Hand-Eye)
Schritt 2: Mit T_cam_base: P_cam = T · P_base   →  Z_cam  pro Punkt
           Fit Z_cam = a/d_rel + b              →  (a, b)        (Tiefe)
```

Die Hand-Eye-Lösung ist **komplett unabhängig** von der Tiefe (PnP braucht nur 2D-Pixel + 3D-Weltpunkte + Intrinsik). Die Tiefen-Kalibrierung nutzt die gerade gelöste Hand-Eye-Matrix, um Ground-Truth-Z im Kameraframe zu bekommen — deshalb muss Hand-Eye *zuerst* konvergieren.

> **Warum nicht `cv2.calibrateHandEye`?**  Das Verfahren zielt auf *eye-in-hand*-Setups (Kamera am Gripper). Hier ist die Kamera fest, der Marker bewegt sich mit dem Arm. Für diesen Fall ist `solvePnP` mit `P_base` direkt als Weltkoordinate die saubere, einfache Formulierung.

---

## Marker-Strategie

### Physikalische Marker am Arm

Drei Marker, alle als bedruckte/geklebte Farbpunkte (≥ 15 mm Durchmesser für robuste Subpixel-Zentrierung):

| Marker | Farbe | Anbringung | Zweck |
|---|---|---|---|
| `marker_grasp` | **orange/rot** | auf `gripper_base_link` direkt über `grasp_link` | primärer Datenpunkt, bewegt sich durch den ganzen Arbeitsraum |
| `marker_forearm` | **grün** | Unterarm (Mitte zwischen `joint_2` und `joint_3`) | sekundärer Punkt, hebelt PnP-Geometrie |
| `marker_base` | **blau** | fest auf `base_link` in bekanntem Offset | statischer Anker — sofortige Plausibilitätsprüfung der Hand-Eye-Lösung, ohne den Arm zu bewegen |

> Farben werden per HSV-Segmentierung getrennt. Drei klar unterschiedliche Hue-Bereiche (rot-orange ~10°, grün ~60°, blau ~110°) sind in der Praxis unverwechselbar. Bei Mehrdeutigkeit Farben durch AprilTags oder ArUco-Marker ersetzen — die haben zusätzlich Orientierung + Pose pro Frame (Option B, siehe unten).

### URDF-Erweiterung

Marker als **fixed joints** in `BAArduinoController/urdf/bracket_arm.urdf` eintragen, damit TF deren Weltposition pro Frame liefert:

```xml
<joint name="joint_marker_grasp" type="fixed">
  <parent link="gripper_base_link"/>
  <child  link="marker_grasp"/>
  <origin xyz="0 0.01 0.07" rpy="0 0 0"/>   <!-- gemessen mit Messschieber -->
</joint>
<link name="marker_grasp"/>

<joint name="joint_marker_forearm" type="fixed">
  <parent link="link_2"/>
  <child  link="marker_forearm"/>
  <origin xyz="0 0 0.04" rpy="0 0 0"/>
</joint>
<link name="marker_forearm"/>

<joint name="joint_marker_base" type="fixed">
  <parent link="base_link"/>
  <child  link="marker_base"/>
  <origin xyz="0.05 0.0 0.02" rpy="0 0 0"/>
</joint>
<link name="marker_base"/>
```

Die Offsets (`xyz`) werden **einmalig mechanisch ausgemessen** — das ist der einzige manuelle Schritt, der bleibt, und er ist unkritisch auf Millimeterebene.

### Option B: ArUco statt Farbe

Sollten Farb-Fehlklassifikationen (Reflexionen, ähnliche Hintergrundfarben) zum Problem werden: 20×20 mm ArUco-Marker `DICT_4X4_50` an den gleichen Stellen. `cv2.aruco.detectMarkers` liefert direkt Pixel-Eckpunkte + ID — keine HSV-Tuningarbeit. Geometrisch identisch zum Farb-Ansatz.

---

## ROS 2 — Topics & Services

### Inputs (Subscribe)
| Topic | Typ | Beschreibung |
|---|---|---|
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Markerposen → base_link via TF2 Lookup |
| `/ba_overview_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Kamerabild |
| `/ba_overview_camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsik (einmalig gelatched) |
| `/joint_states` | `sensor_msgs/JointState` | Stillstand-Erkennung (Geschwindigkeit ≈ 0) |

### Outputs (Publish)
| Topic | Typ | Beschreibung |
|---|---|---|
| `/ba_calib/hand_eye` | `geometry_msgs/TransformStamped` | aktuelle `T_cam_base` (latched) |
| `/ba_calib/depth_params` | `std_msgs/Float32MultiArray` | `[a, b, rmse_m, n_samples]` (latched) |
| `/ba_calib/reprojection_px` | `std_msgs/Float32` | mittlerer PnP-Reprojektionsfehler in Pixeln |
| `/ba_calib/depth_rmse_m` | `std_msgs/Float32` | RMSE der Tiefenfit in Metern |
| `/ba_calib/status` | `std_msgs/String` | `idle / collecting / solving / good / degraded` |
| `/ba_calib/datapoints` | `std_msgs/Int32` | aktuell verwendete Datenpunkte |
| `/ba_calib/debug/image` | `sensor_msgs/Image` | Bild mit detektierten Markern + Reprojektionen |

### Services
| Service | Typ | Beschreibung |
|---|---|---|
| `/ba_calib/run_sequence` | `std_srvs/Trigger` | aktiven Kalibrierungslauf starten |
| `/ba_calib/recalibrate` | `std_srvs/Trigger` | aus aktuell gesammelten Punkten neu lösen |
| `/ba_calib/reset` | `std_srvs/Trigger` | Datenpunkte verwerfen |
| `/ba_calib/save` | `std_srvs/Trigger` | Hand-Eye + Depth in die existierenden YAML-Dateien schreiben |
| `/ba_calib/load` | `std_srvs/Trigger` | aus Dateien laden und publishen |

---

## Node-Struktur: `ba_autocalib_node`

```
ba_autocalib_node
├── MarkerDetector    HSV/ArUco → {marker_id: (u, v, confidence)}
├── TfSampler         base_link ← marker_X pro Marker pro Frame
├── DataCollector     filtert Datenpunkte nach Qualität, hält Ringpuffer
├── HandEyeSolver     cv2.solvePnP (Iterative oder SQPnP) auf alle Marker+Frames
├── DepthSolver       scipy.optimize.curve_fit für Z_cam = a/d_rel + b
├── QualityMonitor    Reprojektionsfehler + Depth-RMSE → Status/Trigger
└── SequenceRunner    bewegt den Arm durch calib_poses.yaml (siehe unten)
```

### DataCollector — Filterregeln

Sample nur dann speichern, wenn *alle* zutreffen:

- Arm **still** (|q̇| < 0.01 rad/s für ≥ 200 ms auf allen Gelenken)
- Marker **sicher detektiert** (HSV-Blob-Fläche ≥ 30 px, ArUco-Marker vollständig)
- Pose-Diversität: neue Armpose ≥ 3 cm oder ≥ 5° von jeder bereits gespeicherten Markerposition
- Tiefenwert bei `(u, v)` **plausibel** (d_rel in `[0.2, 0.95]`, kein Rand-Ausreißer)
- TF-Lookup für den Marker gelingt innerhalb von 50 ms (keine veralteten Transforms)

Ringpuffer-Kapazität: `N_max = 300` (3 Marker × 100 Posen). Ältester Eintrag fällt raus.

---

## Solver im Detail

### HandEyeSolver — `cv2.solvePnP`

```python
import cv2, numpy as np

# Alle Datenpunkte unabhängig von Marker-ID in zwei Arrays:
object_points = np.array([d.P_base for d in data], dtype=np.float32)   # (N, 3)
image_points  = np.array([d.uv     for d in data], dtype=np.float32)   # (N, 2)
K, dist = camera_info.K.reshape(3,3), np.zeros(5)   # nach rectify = 0

success, rvec, tvec = cv2.solvePnP(
    object_points, image_points, K, dist,
    flags=cv2.SOLVEPNP_SQPNP            # robust, braucht N >= 4
)
R, _ = cv2.Rodrigues(rvec)              # R (3x3) ist T_cam_base.Rotation
T_cam_base = np.eye(4)
T_cam_base[:3,:3] = R
T_cam_base[:3, 3] = tvec.flatten()

# Invers: T_base_cam für config/perception_pipeline.yaml `hand_eye_transform`
T_base_cam = np.linalg.inv(T_cam_base)
```

Reprojektionsfehler pro Punkt (Pixel) als Residuum publishen; Median < 2 px = gesunde Lösung.

### DepthSolver — Kurvenfit

```python
from scipy.optimize import curve_fit

# Z_cam pro Punkt: P_cam = T_cam_base @ [P_base; 1]
P_cam = (T_cam_base @ np.c_[object_points, np.ones(len(data))].T).T[:, :3]
Z_cam = P_cam[:, 2]                     # (N,)
d_rel = np.array([d.d_rel for d in data])

model = lambda d, a, b: a / d + b
(a, b), _ = curve_fit(model, d_rel, Z_cam, p0=[0.1, 0.1])
rmse_m = float(np.sqrt(np.mean((model(d_rel, a, b) - Z_cam)**2)))
```

> Der Fit ist konsistent mit `scripts/calibrate_depth.py` (gleiches Modell `Z = a/d_rel + b`). Ergebnis geht 1:1 in dieselbe `config/depth_calibration.yaml`.

### QualityMonitor — zwei Residuen

```python
publish("/ba_calib/reprojection_px", median_reprojection_error)
publish("/ba_calib/depth_rmse_m",    depth_rmse_m)

if median_reprojection_error > 3.0 or depth_rmse_m > 0.03:
    status = "degraded"
elif n_samples >= 50 and passed_thresholds:
    status = "good"
```

Kennzahlen, die realistisch zu erwarten sind (bei 15 mm Markern, 640×480, ~1 m Abstand):
- Reprojektion: **0.5 – 2 px Median**
- Tiefen-RMSE: **10 – 20 mm** (vergleichbar zu aktuell 15 mm aus `depth_calibration.yaml`)

---

## Aktiver Kalibrierungslauf (`/ba_calib/run_sequence`)

Unverändert im Ablauf, aber pro Pose werden jetzt **alle drei Marker** in einem Rutsch erfasst:

```
ros2 service call /ba_calib/run_sequence std_srvs/srv/Trigger
        ↓
1. (optional) reset
2. Arm fährt Sequenz aus calib_poses.yaml (MoveIt)
3. Pro Pose: warten bis still → Bild+Depth holen → pro sichtbarem Marker einen Datenpunkt
4. Nach Sequenz: HandEyeSolver → DepthSolver
5. Beide Matrizen published + automatisch gespeichert
6. Report: Reprojektionsfehler vorher/nachher, Depth-RMSE vorher/nachher
```

### Pose-Sequenz — Auslegung für PnP + Depth

Gute PnP braucht **Tiefenvielfalt** und **Blickwinkelvielfalt**. Gute Depth braucht **breite d_rel-Spreizung**. Beides decken wir mit:

```yaml
# calib_poses.yaml — ~12 Posen, decken Arbeitsraum + verschiedene Tiefen ab
poses:
  - {name: near_center, joints: [ 0.00, -0.30,  0.80,  0.00,  0.00]}
  - {name: near_left,   joints: [ 0.60, -0.30,  0.80,  0.00,  0.00]}
  - {name: near_right,  joints: [-0.60, -0.30,  0.80,  0.00,  0.00]}
  - {name: mid_center,  joints: [ 0.00,  0.10,  0.50,  0.00,  0.00]}
  - {name: mid_left,    joints: [ 0.60,  0.10,  0.50,  0.00,  0.00]}
  - {name: mid_right,   joints: [-0.60,  0.10,  0.50,  0.00,  0.00]}
  - {name: far_center,  joints: [ 0.00,  0.50,  0.20,  0.00,  0.00]}
  - {name: far_left,    joints: [ 0.40,  0.50,  0.20,  0.00,  0.00]}
  - {name: far_right,   joints: [-0.40,  0.50,  0.20,  0.00,  0.00]}
  - {name: high_center, joints: [ 0.00, -0.60,  0.30,  0.00,  0.30]}
  - {name: low_left,    joints: [ 0.40,  0.30,  0.60,  0.00, -0.20]}
  - {name: low_right,   joints: [-0.40,  0.30,  0.60,  0.00, -0.20]}
```

> Werte sind Platzhalter — müssen an den konkreten Arbeitsraum angepasst werden. Auswahlkriterium: `marker_grasp` ist in allen 12 Posen für die Kamera sichtbar und verteilt sich möglichst gleichmäßig über den Bildbereich.

Bei 12 Posen × ~2 sichtbare Marker = ~24 Datenpunkte — ausreichend für SQPnP (N ≥ 4) und Depth-Fit (N ≥ 10).

---

## Persistenz — in die bestehenden YAMLs schreiben

Kein paralleles Dateischema. Die existierenden Konfigs bleiben die Quelle der Wahrheit:

```
config/
├── perception_pipeline.yaml      ← Hand-Eye-Matrix wird dort überschrieben
│   (Schlüssel: hand_eye_transform, 16 Floats)
└── depth_calibration.yaml        ← a, b, rmse_m, n_samples werden dort überschrieben
```

Vor dem Überschreiben: Backup in `~/.ros/ba_calibration/backup/TIMESTAMP_{perception_pipeline,depth_calibration}.yaml`. Autocalib-Node hat einen eigenen Rohdaten-Cache (`~/.ros/ba_calibration/datapoints.json`) für Replay/Debug.

Beim Start des `ba_autocalib_node`: nichts überschreiben — nur beobachten und `/ba_calib/status` publishen. Überschreiben passiert ausschließlich bei `/ba_calib/save` oder am Ende eines erfolgreichen `run_sequence`.

---

## Integration

```
┌─────────────────────────────────────┐
│  ba_autocalib_node                  │
│  ┌────────────────────────────────┐ │
│  │ solvePnP → T_cam_base          │─┼──► schreibt config/perception_pipeline.yaml
│  │ curve_fit → a, b               │─┼──► schreibt config/depth_calibration.yaml
│  └────────────────────────────────┘ │
└────────────────▲────────────────────┘
                 │ /tf, camera, joint_states
                 │
          ┌──────┴──────┐
          │ Bracket Arm │
          └─────────────┘
```

Die laufende Perception-Pipeline (`perception_pipeline_node`, `goal_generator_node`) bleibt unberührt — sie liest die YAMLs beim Start. Nach einem erfolgreichen `run_sequence` wird sie neu gestartet (oder: später ein Reload-Service, falls gewünscht).

---

## Offene Punkte / Varianten

1. **Online vs. nur aktiv.** Der passive DataCollector (sammelt während normalem Betrieb) kann in Phase 2 dazukommen. Phase 1 reicht mit `run_sequence` + manuellem Trigger — und löst das „Kalibrierung nervt"-Problem bereits komplett.
2. **Joint-Bundle-Adjustment**: Hand-Eye + Depth + (optional) Kamera-Intrinsik in einem Optimierungsproblem zu lösen, ist möglich (`scipy.least_squares` mit gemischten Residuen). Erst sinnvoll, wenn die sequentielle Lösung als ungenau auffällt.
3. **Hot-Reload** der Perception-Pipeline per ROS-Parameter-Update statt Neustart — nice to have, nicht blockierend.
4. **ArUco als Default statt Farbe**, falls die HSV-Robustheit im Experiment enttäuscht.
5. **Gripper-Tip als Marker nutzen** (ohne zusätzlichen Aufkleber): falls die Finger optisch gut genug erkennbar sind, könnte die Fingerspitze selbst als „Marker" dienen. Erstmal nicht verfolgen — Aufkleber sind einfacher und robuster.

---

## Nächste Schritte

1. [ ] Marker physisch anbringen (orange/grün/blau, ≥ 15 mm) + Offsets mit Messschieber bestimmen
2. [ ] URDF erweitern: `marker_grasp`, `marker_forearm`, `marker_base` als fixed joints in `BAArduinoController/urdf/bracket_arm.urdf`
3. [ ] ROS 2 Package `ba_autocalib` anlegen (ament_python), `ba_` Topic-Konvention
4. [ ] `MarkerDetector` (HSV, 3-farbig) implementieren + Debug-Topic mit Overlays
5. [ ] `DataCollector` + `TfSampler` (TF2 Lookup base_link ← marker_*)
6. [ ] `HandEyeSolver` (`cv2.solvePnP SQPNP`) + Reprojektionsfehler-Publikation
7. [ ] `DepthSolver` (`curve_fit`) — schreibt im Format von `calibrate_depth.py`
8. [ ] `SequenceRunner` + `calib_poses.yaml` (MoveIt JointConstraint-Goals, gleiches Pattern wie `goal_generator_node`)
9. [ ] `/ba_calib/save` schreibt Backups + überschreibt `config/perception_pipeline.yaml` und `config/depth_calibration.yaml`
10. [ ] End-to-End-Test: vor/nach-Vergleich von (i) Reprojektionsfehler, (ii) Depth-RMSE, (iii) Greif-Genauigkeit am Objekt

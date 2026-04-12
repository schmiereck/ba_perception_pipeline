# PLAN_ROS2_CAMERA.md
# USB-Kamera Anbindung am Raspberry Pi (ubuntu1) via ROS2

## Kontext

- **Ziel**: 6-DOF Arm mit VLM-gestützter Objekterkennung + Depth Estimation steuern
- **Pi**: ubuntu1 (Ubuntu, ROS2 Humble), verbunden mit WSL2-Laptop via CycloneDDS
- **Laptop**: WSL2 Ubuntu 22.04, ROS2 Humble + MoveIt, ROS_DOMAIN_ID=1
- **Arbeitsbereich**: ~60×60 cm, Objekte auf verschiedenen Höhen/Regalen

---

## Kameraposition

### Empfehlung: Schräg von oben, seitlich versetzt

```
        [Kamera]
            \  ~45-60°
             \
     [Regal]  \
     □ □ □     \
     □ □ □    [Arm-Basis]
```

- **Höhe**: 60–80 cm über Arbeitsfläche
- **Seitlicher Versatz**: 30–40 cm von der Arm-Basis
- **Neigungswinkel**: ~45° nach unten

### Warum nicht andere Positionen

| Position | Problem |
|---|---|
| Gerade von oben | Höhenunterschiede zwischen Regalebenen kaum sichtbar → schlechte Depth-Map |
| Seitlich flach | Objekte verdecken sich, Arm blockiert Sicht |
| Direkt über Basis, schräg vorne | Hintere Regalebenen schlecht sichtbar |

### Tipp
Erstmal provisorisch mit Stativ/Klemme montieren. Optimale Position erst fixieren, wenn erste Depth-Maps zufriedenstellend sind.

---

## ROS2-Integration: usb_cam + image_transport

### Warum compressed Topic (nicht HTTP-Snapshot)

- Direkt in ROS2-Welt — Timestamps, Synchronisation, Wiederverwendbarkeit
- Spätere Portierung auf Jetson: nur Node verschieben
- `rviz2` kann live reinschauen
- Raw-Stream (640×480 @ 30fps) wäre ~220 Mbit/s über WLAN → zu viel
- JPEG-komprimiert → ~5–15 Mbit/s → problemlos über WLAN

### Installation auf ubuntu1

```bash
sudo apt install ros-humble-usb-cam ros-humble-image-transport-plugins
```

### Start

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=10.0
```

`image_transport` publiziert automatisch:
- `/camera/image_raw` (unkomprimiert, nur lokal verwenden)
- `/camera/image_raw/compressed` (JPEG, über Netzwerk)

### Empfang auf Laptop (WSL2)

```bash
# Topic prüfen
ros2 topic list | grep camera

# Metadaten prüfen (ohne Bilddaten)
ros2 topic echo /camera/image_raw/compressed --no-arr

# In rviz2 anzeigen
rviz2  # → Add → Image → Topic: /camera/image_raw/compressed
```

---

## Kamerakalibrierung (einmalig, ~30 Minuten)

Notwendig für die spätere Koordinatenberechnung (fx, fy, cx, cy).

```bash
# Auf Laptop
sudo apt install ros-humble-camera-calibration

ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/image_raw \
  camera:=/camera
```

- Schachbrettmuster (8×6 Felder, 25mm Feldgröße) ausdrucken
- Muster in verschiedenen Positionen/Winkeln vor die Kamera halten
- Ergebnis: `ost.yaml` mit Kameramatrix → in ROS2 `camera_info` Topic

---

## Pipeline-Übersicht (Gesamtbild)

```
USB-Kamera (Pi, ubuntu1)
    ↓
usb_cam Node → /camera/image_raw/compressed (ROS2, über WLAN)
    ↓
Laptop (WSL2):
    ├──→ Depth Anything V2 (Small) → Tiefenkarte (HxW float)
    │
    └──→ VLM (Groq/Llama 4 Scout oder Gemini) → Zielpixel (u, v)
                    ↓
          depth_map[v, u] → Z
          (u - cx) * Z / fx → X
          (v - cy) * Z / fy → Y
                    ↓
          Hand-Eye-Transform → Roboterkoordinaten
                    ↓
                 MoveIt
                    ↓
          Pi Bridge → Arduino → Servos
```

---

## Gripper-Kamera (zweite Kamera, optional, spätere Phase)

- Montiert am Greifer, schaut nach vorne/unten
- Aufgabe: Visuelles Servo in den letzten ~10 cm
- Korrigiert Fehler der Overview-Kalibrierung
- Separates ROS2 Topic: `/gripper_camera/image_raw/compressed`
- Aktivierung: nur wenn Arm sich im Nahbereich des Ziels befindet

---

## Nächste Schritte

1. [ ] USB-Kamera physisch an ubuntu1 anschließen
2. [ ] `usb_cam` installieren und Topic auf Laptop sichtbar machen
3. [ ] Kamerakalibrierung durchführen (Schachbrettmuster)
4. [ ] Depth Anything V2 Small auf Laptop installieren und testen
5. [ ] Ersten ROS2-Node: subscribt compressed Topic, gibt Depth-Map aus
6. [ ] VLM-Call integrieren: Zielpixel aus Bild extrahieren
7. [ ] Koordinatenberechnung: Pixel + Tiefe → 3D-Punkt
8. [ ] Hand-Eye-Kalibrierung
9. [ ] MoveIt-Goal aus 3D-Koordinaten generieren
10. [ ] Gripper-Kamera ergänzen (spätere Phase)

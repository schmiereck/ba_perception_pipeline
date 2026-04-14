# PLAN.md — ba_perception_pipeline

Perception-Pipeline für die Bracket-Arm-Steuerung. Läuft auf dem **WSL2-Laptop**
als einzelner ROS2-Prozess. Kombiniert Depth Estimation, VLM-Zielerkennung und
Backprojection — die große Depth-Map bleibt als numpy-Array im Prozess und geht
nie über DDS.

---

## Phase 3: In-Process Perception Pipeline

### Kontext

Phase 2 (`ba_depth_node`) ist abgeschlossen: der Depth-Estimator-Node empfängt
komprimierte Bilder vom Pi, rektifiziert sie und publiziert eine 32FC1 Depth Map.

**Problem**: CycloneDDS auf WSL2 Loopback kann große Messages (~1.2 MB) nicht
zwischen lokalen Prozessen transportieren. Downstream-Nodes (VLM, Backprojection)
könnten das Depth-Topic nicht subscriben.

**Lösung**: Alle Perception-Schritte in **einem** Python-Prozess. `rclpy`
unterstützt kein Intra-Process-Communication (nur rclcpp), deshalb hilft es
nicht, mehrere rclpy-Nodes in einen Prozess zu packen. Stattdessen: ein
einzelner Node, der intern die Pipeline als direkte Python-Aufrufe orchestriert.

### Entscheidungen

- **Trigger-Mechanismus: Topics (statt Service/Action)**
  Ursprünglich war ein ROS2 Service (`DetectTarget.srv`) geplant, und ein
  Action wäre noch schöner gewesen (non-blocking, Feedback). Aber:
  Custom `.srv`/`.action` Definitionen erfordern `ament_cmake` (oder ein
  separates `_msgs`-Paket) — für ein `ament_python`-Paket zu aufwändig.
  Pragmatischer Ansatz: Topic-basiert.
  - **Trigger**: `std_msgs/String` auf `/perception/detect_request` (Prompt)
  - **Ergebnis**: `geometry_msgs/PoseStamped` auf `/perception/target_pose`
  - **Status**: `std_msgs/String` auf `/perception/status` (OK/ERROR + Details)
  Funktional identisch, ohne Custom-Message-Paket. Upgrade auf Action möglich
  wenn ein `ba_perception_msgs`-Paket angelegt wird.

- **VLM-Provider: Config-Switch (Groq + Gemini)**
  Beide implementiert, per `vlm_provider` Parameter wählbar. Getestet mit
  Groq (Llama 4 Scout) — Latenz ~400-600 ms pro Request.

- **API Key: Environment-Variable als Fallback**
  Wenn `vlm_api_key` im YAML leer ist, liest der Node automatisch
  `GROQ_API_KEY` bzw. `GEMINI_API_KEY` aus der Umgebung.

- **Separates Repo statt Erweiterung von ba_depth_node**
  `ba_depth_node` bleibt ein sauberer, abgeschlossener Depth-Publisher.
  Die Perception-Pipeline importiert `DepthEstimator` als Python-Klasse
  (Cross-Package-Import im selben colcon-Workspace).

- **Metrische Tiefe: noch nicht (Phase 4)**
  Backprojection liefert aktuell relative Koordinaten (DA V2 output 0.0–1.0).
  Metrische Skalierung kommt in Phase 4.

### Architektur

```
Pi (ubuntu1)          WSL2 Laptop: perception_pipeline_node
usb_cam ──JPEG──►     ┌────────────────────────────────────┐
                       │  Subscribe: CompressedImage         │
                       │  Subscribe: CameraInfo (einmalig)   │
                       │  Subscribe: /perception/detect_request │
                       │                                     │
                       │  Intern (kein DDS):                 │
                       │  1. Letztes Frame aus Buffer         │
                       │  2. Rektifizieren                    │
                       │  3. DA V2 → depth_map (np.array)    │
                       │  4. VLM API → (u, v)                │
                       │  5. Backprojection → (X,Y,Z) Kamera │
                       │  6. Hand-Eye → (X,Y,Z) Roboter      │
                       │                                     │
                       │  Publish: /perception/target_pose   │
                       │  Publish: /perception/status        │
                       │  Publish: depth/image_raw (debug)   │
                       └────────────────────────────────────┘
```

### Status

- [x] Phase 3.1: `DepthEstimator`-Klasse aus `depth_estimator_node.py` extrahiert
      (`ba_depth_node/depth_estimator.py` — reine Python-Klasse, kein ROS2)
- [x] Phase 3.2: `backprojection.py` — Pixel + Depth → 3D (Pinhole-Modell)
- [x] Phase 3.3: `vlm_client.py` — Groq + Gemini mit Config-Switch, Response-Parsing
- [x] Phase 3.4: `perception_pipeline_node.py` — orchestriert alles in einem Prozess
- [x] Phase 3.5: Config YAML, Launch File, CLAUDE.md
- [x] Phase 3.6: End-to-End-Test mit laufender Kamera + Groq VLM
      Zwei erfolgreiche Detections: "identify the most prominent object"
      → Pixel (445,283), und "the keyboard" → Pixel (326,243).
      VLM-Latenz ~400-600 ms, Depth ~1200 ms, Gesamt ~1800-2500 ms.
      Debug-Bilder (depth_latest.png, target_latest.png) visuell verifiziert.

### Gemessene Performance (Phase 3)

| Schritt | Latenz |
|---|---|
| Depth (DA V2 Small, CPU, warm) | ~1200 ms |
| VLM (Groq, Llama 4 Scout) | ~400–600 ms |
| Backprojection + Hand-Eye | < 1 ms |
| **Gesamt** | **~1800–2500 ms** |

---

## Phase 4: Metrische Tiefenskalierung

DA V2 Small liefert **relative inverse depth** (0.0–1.0), keine metrischen
Werte. Für MoveIt-Grasp-Planning brauchen wir echte Meter.

**Ansatz**: Interaktives Kalibrierscript (`scripts/calibrate_depth.py`) mit
OpenCV-GUI-Fenster. Schachbrettmuster (9×7 Felder = 8×6 innere Ecken, 25mm
Feldgröße) in verschiedenen Abständen vor die Kamera halten. Per `solvePnP`
wird automatisch der metrische Abstand berechnet — kein Messen von Hand nötig.

### Status

- [x] Phase 4.1: `calibrate_depth.py` — interaktives Script mit Live-Preview
- [x] Phase 4.2: `scale_depth()` in `backprojection.py`
- [x] Phase 4.3: Pipeline-Integration (`depth_calibration_file` Parameter)
- [x] Phase 4.4: Erste Kalibrierung durchgeführt und verifiziert

### Erste Kalibrierung (2026-04-12)

```
model_type: linear
a: -0.722, b: 0.968
RMSE: 6.2 cm
Bereich: 33–68 cm (5 Captures, 240 Datenpunkte)
```

End-to-End verifiziert: Pipeline liefert jetzt metrische Koordinaten
(z.B. Z=0.81m für ein Objekt auf dem Tisch bei schrägem Kamerablick).

Die Kalibrierung kann jederzeit wiederholt werden (z.B. am finalen
Roboter-Arbeitsplatz) — einfach das Script erneut ausführen.

---

## Nächste Schritte (Gesamtpipeline)

1. [x] USB-Kamera physisch an ubuntu1 anschließen
2. [x] `usb_cam` installieren und Topic auf Laptop sichtbar machen
3. [x] Kamerakalibrierung durchführen (Schachbrettmuster)
4. [x] Depth Anything V2 Small auf Laptop installieren und testen
5. [x] ROS2 Depth-Node: subscribt compressed Topic, gibt Depth-Map aus
6. [x] VLM-Call integrieren: Zielpixel aus Bild extrahieren
7. [x] Koordinatenberechnung: Pixel + Tiefe → 3D-Punkt
8. [x] Metrische Tiefenskalierung (Kalibrierung + Pipeline-Integration)
9. [x] **Hand-Eye-Transform einsetzen** (Matrix in Config eingetragen)
10. [x] **MoveIt-Goal aus 3D-Koordinaten generieren** (Basis-Node erstellt)
11. [ ] **Echte MoveIt-Ansteuerung finalisieren** (Action-Client vervollständigen)
12. [ ] **Tiefenkalibrierung am Roboter-Arbeitsplatz wiederholen**
13. [ ] **Gripper-Kamera ergänzen** (spätere Phase)

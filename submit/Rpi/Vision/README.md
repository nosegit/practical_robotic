# Vision System - Real-time Camera Processing & Detection

ZeroMQ-based vision server that captures camera frames, detects ArUco markers (3D pose) and colored shapes, then broadcasts detection data and video stream over the network.

---

## 📁 File Structure

| File | Purpose |
|------|---------|
| `calibration_data.npz` | Camera calibration data (intrinsics + distortion) - required for 3D pose estimation |
| `camera_server_vid.py` | Main vision server: captures frames, runs detection, publishes results |
| `camera_viewer.py` | Live video viewer - displays camera feed with detection overlays |
| `lib/camera_client.py` | Client library - provides simple API for querying vision data |
| `lib/camera_calibration.py` | Calibration tool - generates calibration_data.npz using chessboard pattern |

---

## 🚀 Quick Start

### Step 1: Calibrate Camera (One-time Setup)
```bash
python lib/camera_calibration.py
```
- Edit `CHESSBOARD_SIZE` and `SQUARE_SIZE` in `camera_calibration.py
- Run script and capture 15+ chessboard pattern images (press 's')
- Press 'q' to compute and save `calibration_data.npz`
- Move `calibration_data.npz` to main `Vision/` folder

### Step 2: Start Vision Server (Background)
```bash
python camera_server_vid.py
```
Publishes detection data and video stream continuously.

### Step 3: View Live Feed (Optional)
```bash
python camera_viewer.py
```
Displays real-time video with ArUco markers and color detection overlays.

---

## 📡 Network Architecture

### Data Streams

| Stream | Port | Type | Frequency | Content |
|--------|------|------|-----------|---------|
| **Detection** | 5557 | ZMQ PUB/SUB | 10 Hz | JSON: ArUco markers (3D pose) + colored shapes |
| **Video** | 5558 | ZMQ PUB/SUB | 10 FPS | JPEG: Live camera feed with overlays |

### Message Formats

**Detection JSON (Port 5557):**
```json
{
  "timestamp": 1234567.89,
  "aruco": [
    {"id": 1, "center": [320, 240], "distance": 0.5, "yaw": 45.2}
  ],
  "shapes": [
    {"color": "RED", "label": "Triangle", "center": [100, 150], "area": 2500}
  ]
}
```

**Video Stream (Port 5558):**
- Raw JPEG bytes with overlaid bounding boxes, pose axes, and labels
- Quality: 50% JPEG compression for network efficiency

---

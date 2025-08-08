#!/usr/bin/env python3
"""
YOLO Object Detection Camera Viewer with GStreamer Hardware Acceleration
Optimized for minimal latency on Jetson devices with bottle detection

Usage:
    python3 camera3.py

Requirements:
    - OpenCV with GStreamer support: pip install opencv-python
    - Ultralytics YOLO: pip install ultralytics
    - PyTorch: pip install torch torchvision
    - Camera device at /dev/video0
    - YOLO model: yolo10b_trained.pt
    - For Jetson devices: Hardware acceleration support

Controls:
    - Press 'q' or ESC to quit
    - Press 's' to toggle detection on/off
"""

import cv2
import sys
import os
import time
import numpy as np
from ultralytics import YOLO
from flask import Flask, Response



# --- MJPEG generator for Flask streaming ---
def generate_yolo_frames():
    model_path = "yolov10b_trained.pt"
    if not os.path.exists(model_path):
        print(f"Error: YOLO model not found at {model_path}")
        return
    print(f"Loading YOLO model from {model_path}...")
    try:
        model = YOLO(model_path)
        print("YOLO model loaded successfully!")
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        return
    gst_pipeline_hw = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw(memory:NVMM) ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )
    gst_pipeline_sw = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )
    print("Opening camera with GStreamer pipeline...")
    print("Trying hardware accelerated pipeline...")
    cap = cv2.VideoCapture(gst_pipeline_hw, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Hardware acceleration failed, trying software pipeline...")
        cap = cv2.VideoCapture(gst_pipeline_sw, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("Error: Cannot open camera!")
            print("Make sure /dev/video0 exists and is accessible")
            return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    detection_enabled = True
    show_confidence = True
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break
            # Run YOLO detection if enabled
            if detection_enabled:
                try:
                    results = model(frame, verbose=False, conf=0.5, iou=0.45)
                    for result in results:
                        boxes = result.boxes
                        if boxes is not None:
                            for box in boxes:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                                confidence = box.conf[0].cpu().numpy()
                                class_id = int(box.cls[0].cpu().numpy())
                                class_name = model.names[class_id] if class_id < len(model.names) else f"Class_{class_id}"
                                color = colors[class_id % len(colors)]
                                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                                if show_confidence:
                                    label = f"{class_name}: {confidence:.2f}"
                                else:
                                    label = class_name
                                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                cv2.rectangle(frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                except Exception as e:
                    print(f"Detection error: {e}")
            # Encode frame as JPEG
            ret2, jpeg = cv2.imencode('.jpg', frame)
            if not ret2:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    finally:
        cap.release()
        print("Camera released")

# --- Flask app for MJPEG streaming ---
app = Flask(__name__)

@app.route('/yolo_camera_feed')
def yolo_camera_feed():
    return Response(generate_yolo_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8081, debug=False, threaded=True)

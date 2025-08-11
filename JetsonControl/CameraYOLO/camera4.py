#!/usr/bin/env python3
"""
YOLO Object Detection Camera Viewer with GStreamer Hardware Acceleration
Afiseaza camera cu detectie YOLO intr-o fereastra OpenCV (nu in browser/Flask)

Usage:
    python3 camera4.py

Requirements:
    - OpenCV cu suport GStreamer: pip install opencv-python
    - Ultralytics YOLO: pip install ultralytics
    - PyTorch: pip install torch torchvision
    - Camera la /dev/video0
    - Model YOLO: yolov10b_trained.pt
    - Jetson: suport hardware acceleration

Controale:
    - 'q' sau ESC pentru iesire
    - 's' pentru a activa/dezactiva detectia
"""

import cv2
import sys
import os
import time
import numpy as np
from ultralytics import YOLO

def main():
    model_path = "yolov10b_trained.pt"
    if not os.path.exists(model_path):
        print(f"Eroare: Modelul YOLO nu a fost gasit la {model_path}")
        return
    print(f"Incarc modelul YOLO din {model_path}...")
    try:
        model = YOLO(model_path)
        print("Model YOLO incarcat cu succes!")
    except Exception as e:
        print(f"Eroare la incarcarea modelului YOLO: {e}")
        return
    print("Deschid camera cu OpenCV standard (fara GStreamer)...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Eroare: Nu pot deschide camera cu OpenCV!")
        print("Verifica daca camera este conectata si accesibila.")
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    detection_enabled = True
    show_confidence = True
    window_name = "YOLO Camera (q sau ESC pentru iesire, s pentru toggle detectie)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Eroare: Nu pot captura frame")
                break
            display_frame = frame.copy()
            if detection_enabled:
                try:
                    results = model(display_frame, verbose=False, conf=0.5, iou=0.45)
                    for result in results:
                        boxes = result.boxes
                        if boxes is not None:
                            for box in boxes:
                                class_id = int(box.cls[0].cpu().numpy())
                                class_name = model.names[class_id] if class_id < len(model.names) else f"Class_{class_id}"
                                # Afiseaza doar sticle (bottle)
                                if class_name.lower() != "bottle":
                                    continue
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                                confidence = box.conf[0].cpu().numpy()
                                color = colors[class_id % len(colors)]

                                # Calculeaza centrul imaginii si al sticlei
                                img_h, img_w = display_frame.shape[:2]
                                img_cx, img_cy = img_w // 2, img_h // 2
                                box_cx = (x1 + x2) // 2
                                box_cy = (y1 + y2) // 2
                                threshold = 30
                                move_x = box_cx - img_cx
                                move_y = box_cy - img_cy
                                if abs(move_x) > threshold:
                                    if move_x < 0:
                                        print("Muta camera la STANGA")
                                    else:
                                        print("Muta camera la DREAPTA")
                                if abs(move_y) > threshold:
                                    if move_y < 0:
                                        print("Muta camera in SUS")
                                    else:
                                        print("Muta camera in JOS")

                                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                                if show_confidence:
                                    label = f"{class_name}: {confidence:.2f}"
                                else:
                                    label = class_name
                                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                cv2.rectangle(display_frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                except Exception as e:
                    print(f"Eroare detectie: {e}")
            cv2.imshow(window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("Iesire...")
                break
            elif key == ord('s'):
                detection_enabled = not detection_enabled
                print(f"Detectie {'activata' if detection_enabled else 'dezactivata'}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera eliberata si ferestrele inchise")

if __name__ == "__main__":
    main()

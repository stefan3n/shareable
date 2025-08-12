#!/usr/bin/env python3
"""
YOLO Object Detection Camera Viewer with GStreamer Hardware Acceleration
Optimizat pentru Jetson Orin Nano cu accelerare hardware

Usage:
    python3 camera4.py

Requirements:
    - OpenCV cu suport GStreamer: pip install opencv-python
    - Ultralytics YOLO: pip install ultralytics
    - PyTorch: pip install torch torchvision
    - Camera la /dev/video0
    - Model YOLO: yolov8n.pt (sau yolov10n pentru nano)
    - Jetson: suport hardware acceleration

Controale:
    - 'q' sau ESC pentru iesire
    - 's' pentru a activa/dezactiva detectia
    - 'r' pentru resetare pipeline
"""

import cv2
import sys
import os
import time
import numpy as np
import threading
from queue import Queue
from ultralytics import YOLO
import torch

class ThreadedCamera:
    """Clasa pentru captura video threadad cu optimizari Jetson"""
    
    def __init__(self, src=0, width=640, height=480, fps=30):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_queue = Queue(maxsize=2)
        self.stopped = False
        self.cap = None
        
    def create_gstreamer_pipeline(self):
        """Creeaza pipeline GStreamer optimizat pentru Jetson"""
        # Pipeline pentru camera USB cu accelerare hardware
        pipeline = (
            f"v4l2src device=/dev/video{self.src} ! "
            f"video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1,format=YUY2 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=2"
        )
        return pipeline
        
    def start(self):
        """Porneste captura video"""
        print("Pornesc captura video cu GStreamer...")
        
        # Incearca primul cu GStreamer
        gst_pipeline = self.create_gstreamer_pipeline()
        print(f"Pipeline GStreamer: {gst_pipeline}")
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("GStreamer nu functioneaza, trec la OpenCV standard...")
            self.cap = cv2.VideoCapture(self.src)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        if not self.cap.isOpened():
            raise RuntimeError("Nu pot deschide camera!")
            
        # Setari pentru performanta
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Porneste thread-ul pentru captura
        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True
        self.thread.start()
        return self
        
    def update(self):
        """Thread pentru citirea continua a frame-urilor"""
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Pastreaza doar ultimele 2 frame-uri
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            else:
                try:
                    self.frame_queue.get_nowait()  # Scoate frame-ul vechi
                    self.frame_queue.put(frame)    # Pune frame-ul nou
                except:
                    pass
                    
    def read(self):
        """Citeste ultimul frame disponibil"""
        if not self.frame_queue.empty():
            return True, self.frame_queue.get()
        return False, None
        
    def stop(self):
        """Opreste captura video"""
        self.stopped = True
        if hasattr(self, 'thread'):
            self.thread.join()
        if self.cap:
            self.cap.release()

def setup_torch_optimizations():
    """Configureaza optimizarile PyTorch pentru Jetson"""
    if torch.cuda.is_available():
        print(f"CUDA disponibil: {torch.cuda.get_device_name()}")
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False
        # Optimizare memorie
        torch.cuda.empty_cache()
    else:
        print("CUDA nu este disponibil, folosesc CPU")

def main():
    # Configureaza optimizarile
    setup_torch_optimizations()
    
    # Cauta modele disponibile (in ordine de preferinta pentru performanta)
    model_candidates = [
        "yolov8n.pt",           # Nano - cel mai rapid
        "yolov10n.pt",          # YOLOv10 nano
        "yolov5n.pt",           # YOLOv5 nano
        "yolov8s.pt",           # Small
        "yolov10n_trained.pt",  # Model antrenat custom
        "yolov10b_trained.pt"   # Model antrenat custom
    ]
    
    model_path = None
    for candidate in model_candidates:
        if os.path.exists(candidate):
            model_path = candidate
            break
    
    if not model_path:
        print(f"Eroare: Niciun model YOLO gasit din lista: {model_candidates}")
        print("Downloading yolov8n.pt...")
        model_path = "yolov8n.pt"
    
    print(f"Incarc modelul YOLO din {model_path}...")
    try:
        # Incarca modelul cu optimizari
        model = YOLO(model_path)
        
        # Configureaza device-ul (CUDA daca e disponibil)
        if torch.cuda.is_available():
            model.to('cuda')
            print("Model incarcat pe GPU (CUDA)")
        else:
            print("Model incarcat pe CPU")
            
        print("Model YOLO incarcat cu succes!")
    except Exception as e:
        print(f"Eroare la incarcarea modelului YOLO: {e}")
        return
    
    # Configureaza camera cu rezolutie redusa pentru performanta
    camera_width = 640   # Redus de la default pentru performanta
    camera_height = 480  # Redus de la default pentru performanta
    camera_fps = 30
    
    print(f"Initializez camera cu rezolutia {camera_width}x{camera_height} @ {camera_fps}fps...")
    
    try:
        camera = ThreadedCamera(src=0, width=camera_width, height=camera_height, fps=camera_fps)
        camera.start()
        print("Camera pornita cu succes!")
    except Exception as e:
        print(f"Eroare la pornirea camerei: {e}")
        return
    
    # Variabile pentru control
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    detection_enabled = True
    show_confidence = True
    window_name = "YOLO Camera Jetson (q/ESC=iesire, s=toggle detectie, r=reset)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    # Variabile pentru FPS monitoring
    fps_counter = 0
    fps_start_time = time.time()
    last_inference_time = 0
    
    try:
        print("Pornesc bucla principala...")
        while True:
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
                
            display_frame = frame.copy()
            current_time = time.time()
            
            if detection_enabled:
                try:
                    # Ruleaza inferenta cu parametri optimizati
                    inference_start = time.time()
                    results = model(
                        display_frame, 
                        verbose=False, 
                        conf=0.5,           # Confidence threshold
                        iou=0.45,           # IoU threshold pentru NMS
                        imgsz=416,          # Rezolutie redusa pentru inferenta
                        half=True,          # FP16 pentru performanta pe GPU
                        device='cuda' if torch.cuda.is_available() else 'cpu'
                    )
                    inference_time = time.time() - inference_start
                    last_inference_time = inference_time
                    
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

                                # Deseneaza bounding box
                                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                                
                                # Adauga label cu confidence
                                if show_confidence:
                                    label = f"{class_name}: {confidence:.2f}"
                                else:
                                    label = class_name
                                
                                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                cv2.rectangle(display_frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                
                except Exception as e:
                    print(f"Eroare detectie: {e}")
            
            # Calculeaza si afiseaza FPS
            fps_counter += 1
            if current_time - fps_start_time >= 1.0:
                fps = fps_counter / (current_time - fps_start_time)
                fps_counter = 0
                fps_start_time = current_time
                
                # Afiseaza informatii de performanta
                fps_text = f"FPS: {fps:.1f}"
                if last_inference_time > 0:
                    fps_text += f" | Inference: {last_inference_time*1000:.1f}ms"
                
                cv2.putText(display_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow(window_name, display_frame)
            
            # Gestioneaza input-ul utilizatorului
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q sau ESC
                print("Iesire...")
                break
            elif key == ord('s'):
                detection_enabled = not detection_enabled
                print(f"Detectie {'activata' if detection_enabled else 'dezactivata'}")
            elif key == ord('r'):
                print("Resetare camera...")
                camera.stop()
                time.sleep(0.5)
                camera = ThreadedCamera(src=0, width=camera_width, height=camera_height, fps=camera_fps)
                camera.start()
                print("Camera resetata!")
                
    except KeyboardInterrupt:
        print("Intrerupt de la tastatura, opresc...")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        print("Camera oprita si resurse eliberate")

if __name__ == "__main__":
    main()

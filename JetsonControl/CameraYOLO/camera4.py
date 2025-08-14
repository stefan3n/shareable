#!/usr/bin/env python3
"""
YOLO Object Detection Camera Viewer with GStreamer Hardware Acceleration
Optimized for Jetson Orin Nano with hardware acceleration

Usage:
    python3 camera4.py

Requirements:
    - OpenCV with GStreamer support: pip install opencv-python
    - Ultralytics YOLO: pip install ultralytics
    - PyTorch: pip install torch torchvision
    - Camera at /dev/video0
    - YOLO model: yolov8n.pt (or yolov10n for nano)
    - Jetson: hardware acceleration support

Controls:
    - 'q' or ESC to exit
    - 's' to toggle detection
    - 'r' to reset pipeline
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

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ThreadedCamera:
    """Class for threaded video capture with Jetson optimizations"""
    
    def __init__(self, src=0, width=640, height=480, fps=30):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_queue = Queue(maxsize=2)
        self.stopped = False
        self.cap = None
        
    def create_gstreamer_pipeline(self):
        """Creates GStreamer pipeline optimized for Jetson"""
        # Pipeline for USB camera with hardware acceleration
        pipeline = (
            f"v4l2src device=/dev/video{self.src} ! "
            f"video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1,format=YUY2 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=2"
        )
        return pipeline
        
    def start(self):
        """Starts video capture"""
        print("Starting video capture with GStreamer...")
        
        # Try GStreamer first
        gst_pipeline = self.create_gstreamer_pipeline()
        print(f"GStreamer Pipeline: {gst_pipeline}")
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("GStreamer not working, switching to standard OpenCV...")
            self.cap = cv2.VideoCapture(self.src)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera!")
            
        # Performance settings
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Start capture thread
        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True
        self.thread.start()
        return self
        
    def update(self):
        """Thread for continuous frame reading"""
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Keep only the latest 2 frames
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            else:
                try:
                    self.frame_queue.get_nowait()  # Remove old frame
                    self.frame_queue.put(frame)    # Add new frame
                except:
                    pass
                    
    def read(self):
        """Reads the latest available frame"""
        if not self.frame_queue.empty():
            return True, self.frame_queue.get()
        return False, None
        
    def stop(self):
        """Stops video capture"""
        self.stopped = True
        if hasattr(self, 'thread'):
            self.thread.join()
        if self.cap:
            self.cap.release()

def setup_torch_optimizations():
    """Configure PyTorch optimizations for Jetson"""
    if torch.cuda.is_available():
        print(f"CUDA available: {torch.cuda.get_device_name()}")
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False
        # Memory optimization
        torch.cuda.empty_cache()
    else:
        print("CUDA not available, using CPU")


# ROS2 Node for command/response
class ArmCommandNode(Node):
    def __init__(self):
        super().__init__('arm_command_node')
        self.publisher_ = self.create_publisher(String, 'arm_command_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'arm_response_topic',
            self.listener_callback,
            10)
        self.last_response = None
        self.waiting_for_response = False
        self.last_command = None

    def send_command(self, msg_str):
        if not self.waiting_for_response:
            msg = String()
            msg.data = msg_str
            self.publisher_.publish(msg)
            self.last_command = msg_str
            self.waiting_for_response = True
            self.get_logger().info(f"Sent command: {msg_str}")

    def listener_callback(self, msg):
        self.last_response = msg.data
        if self.waiting_for_response and self.last_command == msg.data:
            self.waiting_for_response = False

def main():
    # Configure optimizations
    setup_torch_optimizations()

    # ROS2 init
    rclpy.init()
    arm_node = ArmCommandNode()

    # Look for available models (in order of preference for performance)
    model_candidates = [
        "yolov8n.pt",           # Nano - fastest
        "yolov10n.pt",          # YOLOv10 nano
        "yolov5n.pt",           # YOLOv5 nano
        "yolov8s.pt",           # Small
        "yolov10n_trained.pt",  # Custom trained model
        "yolov10b_trained.pt"   # Custom trained model
    ]

    model_path = None
    for candidate in model_candidates:
        if os.path.exists(candidate):
            model_path = candidate
            break

    if not model_path:
        print(f"Error: No YOLO model found from list: {model_candidates}")
        print("Downloading yolov8n.pt...")
        model_path = "yolov8n.pt"

    print(f"Loading YOLO model from {model_path}...")
    try:
        # Load model with optimizations
        model = YOLO(model_path)

        # Configure device (CUDA if available)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model.to(device)
        print(f"Model loaded on {device.upper()}")

        print("YOLO model loaded successfully!")
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        rclpy.shutdown()
        return

    # Configure camera with reduced resolution for performance
    camera_width = 640   # Reduced from default for performance
    camera_height = 480  # Reduced from default for performance
    camera_fps = 30

    print(f"Initializing camera with resolution {camera_width}x{camera_height} @ {camera_fps}fps...")

    try:
        camera = ThreadedCamera(src=0, width=camera_width, height=camera_height, fps=camera_fps)
        camera.start()
        print("Camera started successfully!")
    except Exception as e:
        print(f"Error starting camera: {e}")
        rclpy.shutdown()
        return

    # Control variables
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    detection_enabled = True
    show_confidence = True
    window_name = "YOLO Camera Jetson (q/ESC=exit, s=toggle detection, r=reset)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Variables for FPS monitoring
    fps_counter = 0
    fps_start_time = time.time()
    last_inference_time = 0

    try:
        print("Starting main loop...")
        while rclpy.ok():
            rclpy.spin_once(arm_node, timeout_sec=0.01)
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            display_frame = frame.copy()
            current_time = time.time()

            if detection_enabled:
                try:
                    # Run inference with optimized parameters
                    inference_start = time.time()
                    results = model(
                        display_frame,
                        verbose=False,
                        conf=0.3,           # Lower confidence threshold
                        iou=0.45,           # IoU threshold for NMS
                        imgsz=640,          # Use original frame size
                        device=device
                    )
                    inference_time = time.time() - inference_start
                    last_inference_time = inference_time

                    # Process detections
                    for result in results:
                        boxes = result.boxes
                        if boxes is not None and len(boxes) > 0:
                            for box in boxes:
                                # Get detection data
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                                confidence = float(box.conf[0].cpu().numpy())
                                class_id = int(box.cls[0].cpu().numpy())

                                # Get class name
                                if hasattr(model, 'names') and class_id < len(model.names):
                                    class_name = model.names[class_id]
                                else:
                                    class_name = f"Class_{class_id}"

                                # Filter for bottles only (comment out to show all detections)
                                # if class_name.lower() != "bottle":
                                #     continue

                                color = colors[class_id % len(colors)]

                                # Calculate center of image and bottle
                                img_h, img_w = display_frame.shape[:2]
                                img_cx, img_cy = img_w // 2, img_h // 2
                                box_cx = (x1 + x2) // 2
                                box_cy = (y1 + y2) // 2
                                threshold = 30
                                move_x = box_cx - img_cx
                                move_y = box_cy - img_cy

                                # ROS2 movement commands doar pentru 'bottle'
                                if class_name.lower() == "bottle":
                                    if abs(move_x) > threshold:
                                        if move_x < 0:
                                            # LEFT
                                            arm_node.send_command('<6,0>')
                                        else:
                                            # RIGHT
                                            arm_node.send_command('<6,1>')
                                    # (Optional: implement UP/DOWN if needed)
                                # Draw bounding box
                                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                                # Add label with confidence
                                if show_confidence:
                                    label = f"{class_name}: {confidence:.2f}"
                                else:
                                    label = class_name

                                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                cv2.rectangle(display_frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                except Exception as e:
                    print(f"Detection error: {e}")

            # Calculate and display FPS
            fps_counter += 1
            if current_time - fps_start_time >= 1.0:
                fps = fps_counter / (current_time - fps_start_time)
                fps_counter = 0
                fps_start_time = current_time

                # Display performance info
                fps_text = f"FPS: {fps:.1f}"
                if last_inference_time > 0:
                    fps_text += f" | Inference: {last_inference_time*1000:.1f}ms"

                cv2.putText(display_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(window_name, display_frame)

            # Handle user input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                print("Exiting...")
                break
            elif key == ord('s'):
                detection_enabled = not detection_enabled
                print(f"Detection {'enabled' if detection_enabled else 'disabled'}")
            elif key == ord('r'):
                print("Resetting camera...")
                camera.stop()
                time.sleep(0.5)
                camera = ThreadedCamera(src=0, width=camera_width, height=camera_height, fps=camera_fps)
                camera.start()
                print("Camera reset!")

    except KeyboardInterrupt:
        print("Keyboard interrupt, stopping...")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        rclpy.shutdown()
        print("Camera stopped and resources freed")

if __name__ == "__main__":
    main()

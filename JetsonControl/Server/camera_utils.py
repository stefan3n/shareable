
import cv2
import numpy as np
import time
import os
# YOLO imports
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# Model global (lazy init)
yolo_model = None
YOLO_MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'CameraYOLO', 'yolov10b_trained.pt')

def get_yolo_model():
    global yolo_model
    if yolo_model is not None:
        return yolo_model
    if YOLO is None:
        raise ImportError("ultralytics.YOLO nu este instalat!")
    if not os.path.exists(YOLO_MODEL_PATH):
        raise FileNotFoundError(f"Modelul YOLO nu a fost gasit la {YOLO_MODEL_PATH}")
    yolo_model = YOLO(YOLO_MODEL_PATH)
    return yolo_model


class Camera:
    def __init__(self, camera_name):
        self.camera_name = camera_name
        device_path = self.find_video_device_by_name(camera_name)  # Find the video device path
        if not device_path:
            raise Exception(f"Camera with name '{camera_name}' not found.")

        # Create GStreamer pipeline string
        self.gst_str = ( 
            f'v4l2src device={device_path} ! ' 
            'video/x-raw, width=640, height=480, framerate=30/1 ! ' 
            'nvvidconv ! ' 
            'video/x-raw(memory:NVMM), format=NV12 ! '
            'nvvidconv ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=BGR ! '
            'appsink drop=1 max-buffers=1 sync=0'
        )

        print(f"[GStreamer] Initializing camera '{camera_name}' with: {self.gst_str}")
        try:
            self.cap = cv2.VideoCapture(self.gst_str, cv2.CAP_GSTREAMER) 
            if self.cap.isOpened():
                print(f"[Camera] '{camera_name}' opened with GStreamer pipeline.")
            else:
                raise Exception("Failed to open with GStreamer")
        except Exception as e:
            print(f"[GStreamer] ERROR: {e}")
            # Fallback to standard OpenCV capture
            print(f"[GStreamer] Falling back to standard capture for camera '{camera_name}'")
            self.cap = cv2.VideoCapture(device_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.cap.isOpened():
                print(f"[Camera] '{camera_name}' opened with standard OpenCV capture.")
            else:
                print(f"[Camera] ERROR: '{camera_name}' could not be opened with fallback either.")

    @staticmethod
    def find_video_device_by_name(name):
        by_id_path = "/dev/v4l/by-id/"
        if not os.path.exists(by_id_path):
            print("[Camera Utils] /dev/v4l/by-id/ does not exist. Falling back to default device paths.")
            return None

        for device in os.listdir(by_id_path):
            if name in device:  # 
                device_path = os.path.join(by_id_path, device)
                real_path = os.path.realpath(device_path) 
                print(f"[Camera Utils] Found device '{name}' at {real_path}")
                return real_path

        print(f"[Camera Utils] No device found with name '{name}'")
        return None

    def get_frame(self):
        """Capture a single frame from the camera."""
        if not self.cap.isOpened():
            print(f"[Camera] Camera '{self.camera_name}' is not opened.")
            return None
        ret, frame = self.cap.read()
        if not ret:
            print(f"[Camera] Failed to capture frame from '{self.camera_name}'.")
            return None
        # Encode frame as JPEG
        _, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()

    def get_yolo_frame(self, only_bottle=True, show_confidence=True):
        """Capture a frame, run YOLO, draw boxes, return JPEG bytes."""
        if not self.cap.isOpened():
            print(f"[Camera] Camera '{self.camera_name}' is not opened.")
            return None
        ret, frame = self.cap.read()
        if not ret:
            print(f"[Camera] Failed to capture frame from '{self.camera_name}'.")
            return None
        try:
            model = get_yolo_model()
        except Exception as e:
            print(f"[YOLO] Eroare la incarcare model: {e}")
            return None
        display_frame = frame.copy()
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        try:
            results = model(display_frame, verbose=False, conf=0.5, iou=0.45)
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = model.names[class_id] if class_id < len(model.names) else f"Class_{class_id}"
                        if only_bottle and class_name.lower() != "bottle":
                            continue
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        confidence = box.conf[0].cpu().numpy()
                        color = colors[class_id % len(colors)]
                        # Draw box
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                        if show_confidence:
                            label = f"{class_name}: {confidence:.2f}"
                        else:
                            label = class_name
                        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(display_frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                        cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        except Exception as e:
            print(f"[YOLO] Eroare detectie: {e}")
        # Encode frame as JPEG
        _, jpeg = cv2.imencode('.jpg', display_frame)
        return jpeg.tobytes()

# Cache for camera instances to avoid reopening
_camera_cache = {}

def get_camera(camera_id):
    """Get a camera instance from cache or create a new one"""
    if camera_id not in _camera_cache:
        _camera_cache[camera_id] = Camera(camera_id)
    return _camera_cache[camera_id]


def generate_frames(camera_id, detection_status=None):
    """Frame generator for video streaming (fara YOLO), cu adnotare optionala detection_status ('ON'/'OFF')"""
    camera = get_camera(camera_id)
    while True:
        frame = camera.get_frame()
        if frame is None:
            break
        if detection_status is not None:
            # Decodare, adnotare, recodare
            img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                label = f"DETECTION: {'ON' if detection_status else 'OFF'}"
                color = (0, 200, 0) if detection_status else (0, 0, 200)
                cv2.rectangle(img, (0,0), (220, 35), (0,0,0), -1)
                cv2.putText(img, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
                _, frame = cv2.imencode('.jpg', img)
                frame = frame.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def generate_yolo_frames(camera_id, only_bottle=True, show_confidence=True):
    """Frame generator for MJPEG streaming cu YOLO adnotari (doar bottle)"""
    camera = get_camera(camera_id)
    while True:
        frame = camera.get_yolo_frame(only_bottle=only_bottle, show_confidence=show_confidence)
        if frame is None:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def cleanup():
    """Free resources for all cameras"""
    for camera_id, camera in _camera_cache.items():
        print(f"Releasing camera {camera_id}")
        camera.release()
    _camera_cache.clear()
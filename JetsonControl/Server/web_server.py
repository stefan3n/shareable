from flask import Flask, request, send_from_directory, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import atexit
from camera_utils import generate_frames, generate_yolo_frames, cleanup

CAR_CAMERA_NAME = "usb-046d_0825_5AA55590-video-index0" # ID obtained with `ls -l /dev/v4l/by-id`
ARM_CAMERA_NAME = "usb-046d_HD_Webcam_C525_02CCCB50-video-index0" # ID obtained with `ls -l /dev/v4l/by-id`

app = Flask(__name__, static_folder='.')

# ROS2 Node wrapper
class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('web_server_node')
        
        # Create publishers for different topics
        self.wheels_publisher_ = self.create_publisher(String, 'wheels_server_topic', 10)
        self.arm_publisher_ = self.create_publisher(String, 'arm_server_topic', 10)
        self.control_mode_publisher_ = self.create_publisher(String, 'control_mode_topic', 10)

    def publish(self, msg, topic):
        try:
            msg_obj = String()
            msg_obj.data = msg

            if topic == 'wheels_server_topic':
                self.wheels_publisher_.publish(msg_obj)
            elif topic == 'arm_server_topic':
                self.arm_publisher_.publish(msg_obj)
            elif topic == 'control_mode_topic':
                self.control_mode_publisher_.publish(msg_obj)
            else:
                print(f"[ROS2Publisher] Unknown topic: {topic}")
        except Exception as e:
            print(f"[ROS2Publisher] ERROR at publish: {e}")

# Start rclpy and node in a background thread
import threading
rclpy.init()

arm_detection_enabled = False # Detection is initially OFF

ros2_node = ROS2Publisher()
ros2_node.publish("manual", "control_mode_topic")  # Publish 'manual' at startup

ros2_executor = rclpy.executors.SingleThreadedExecutor()
ros2_executor.add_node(ros2_node)

def spin_ros(): 
    ros2_executor.spin()
ros_thread = threading.Thread(target=spin_ros, daemon=True) 

ros_thread.start()

# Flask routes for serving web pages and handling commands
@app.route('/')  # Serve the main HTML page
def index():
    return send_from_directory('.', 'index.html')

@app.route('/car_camera_feed')
def car_video_feed():
    print("[Flask] Serving car video feed")
    return Response(generate_frames(CAR_CAMERA_NAME), 
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/arm_camera_feed')
def arm_video_feed():
    global arm_detection_enabled
    print(f"[Flask] Serving arm video feed (detection={'ON' if arm_detection_enabled else 'OFF'})")
    if arm_detection_enabled:
        # Folosește direct generatorul cu YOLO și labelul DETECTION: ON va fi adăugat în camera_utils.py
        return Response(generate_yolo_frames(ARM_CAMERA_NAME, show_label=True),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return Response(generate_frames(ARM_CAMERA_NAME, detection_status=False),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detectArm', methods=['POST'])
def toggle_arm_detection():
    global arm_detection_enabled
    arm_detection_enabled = not arm_detection_enabled
    print(f"[Flask] Arm camera detection toggled to: {'ON' if arm_detection_enabled else 'OFF'}")
    return {'status': 'OK', 'detection': arm_detection_enabled}

@app.route('/<path:path>')  # Serve static files (CSS, JS, images)
def static_files(path):
    return send_from_directory('.', path)

@app.route('/manual', methods=['POST']) 
def catch_command_manual():
    try:
        ros2_node.publish("manual", "control_mode_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/autonomous', methods=['POST'])
def catch_command_autonomous():
    try:
        ros2_node.publish("autonomous", "control_mode_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/forward/<pwm_value>', methods=['POST'])
def catch_command_forward(pwm_value):
    try:
        msg = f"forward {pwm_value}"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/backward/<pwm_value>', methods=['POST'])
def catch_command_backward(pwm_value):
    try:
        msg = f"backward {pwm_value}"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/turnLeft/<pwm_value>', methods=['POST'])
def catch_command_turn_left(pwm_value):
    try:
        msg = f"turnLeft {pwm_value}"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/turnRight/<pwm_value>', methods=['POST'])
def catch_command_turn_right(pwm_value):
    try:
        msg = f"turnRight {pwm_value}"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/break', methods=['POST'])
def catch_command_break():
    try:
        msg = f"break"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/start', methods=['POST'])
def catch_command_start():
    try:
        msg = f"start"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/emergency', methods=['POST'])
def catch_command_emergency():
    try:
        msg = f"emergency"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/endEmergency', methods=['POST'])
def catch_command_end_emergency():
    try:
        msg = f"endEmergency"
        ros2_node.publish(msg, "wheels_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveOne/<potentiometerValue>', methods=['POST'])
def catch_command_move_one(potentiometerValue):
    try:
        msg = f"moveOne {potentiometerValue}"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveTwo/<potentiometerValue>', methods=['POST'])
def catch_command_move_two(potentiometerValue):
    try:
        msg = f"moveTwo {potentiometerValue}"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveThree/<potentiometerValue>', methods=['POST'])
def catch_command_move_three(potentiometerValue):
    try:
        msg = f"moveThree {potentiometerValue}"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveOneUp', methods=['POST'])
def catch_command_move_one_up():
    try:
        msg = f"moveOneUp"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveTwoUp', methods=['POST'])
def catch_command_move_two_up():
    try:
        msg = f"moveTwoUp"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveThreeUp', methods=['POST'])
def catch_command_move_three_up():
    try:
        msg = f"moveThreeUp"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveOneDown', methods=['POST'])
def catch_command_move_one_down():
    try:
        msg = f"moveOneDown"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveTwoDown', methods=['POST'])
def catch_command_move_two_down():
    try:
        msg = f"moveTwoDown"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/moveThreeDown', methods=['POST'])
def catch_command_move_three_down():
    try:
        msg = f"moveThreeDown"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/clawOpen', methods=['POST'])
def catch_command_claw_open():
    try:
        msg = f"clawOpen"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/clawClose', methods=['POST'])
def catch_command_claw_close():
    try:
        msg = f"clawClose"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/rotateLeft', methods=['POST'])
def catch_command_rotate_left():
    try:
        msg = f"rotateLeft"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/rotateRight', methods=['POST'])
def catch_command_rotate_right():
    try:
        msg = f"rotateRight"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/rotateArmLeft', methods=['POST'])
def catch_command_rotate_arm_left():
    try:
        msg = f"rotateArmLeft"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/rotateArmRight', methods=['POST'])
def catch_command_rotate_arm_right():
    try:
        msg = f"rotateArmRight"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/goxyz/<x>/<y>/<z>', methods=['POST'])
def catch_command_goxyz(x, y, z):
    try:
        msg = f"go {x} {y} {z}"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/initialPos', methods=['POST'])
def catch_command_initial_pos():
    try:
        msg = f"initialPos"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/goObject', methods=['POST'])
def catch_command_go_object():
    try:
        msg = f"goObject"
        ros2_node.publish(msg, "arm_server_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

atexit.register(cleanup)

def run_server():
    app.run(host='0.0.0.0', port=8080)
    
import signal
import sys

def shutdown_server(signal_received, frame): 
    """Handle shutdown signals like Ctrl+C."""
    print("\n[Shutdown] Signal received. Cleaning up...")
    try:
        cleanup()  # Cleanup camera resources
        ros2_executor.shutdown()  # Shutdown ROS2 executor
        rclpy.shutdown()  # Shutdown ROS2
        print("[Shutdown] Resources cleaned up. Exiting.")
    except Exception as e:
        print(f"[Shutdown] Error during cleanup: {e}")
    sys.exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, shutdown_server)

if __name__ == '__main__':
    run_server()
    
    
    


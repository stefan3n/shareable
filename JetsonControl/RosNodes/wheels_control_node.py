
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WheelsControlNode(Node):
    def __init__(self):
        super().__init__('wheels_control_node')
        self.control_mode = None
        self.create_subscription(
            String,
            'control_mode_topic',
            self.control_mode_callback,
            10
        )
        self.create_subscription(
            String,
            'wheels_server_topic',
            self.wheels_command_callback,
            10
        )
        self.wheels_command_publisher = self.create_publisher(
            String,
            'wheels_command_topic',
            10
        )

    def control_mode_callback(self, msg):
        self.control_mode = msg.data
        self.get_logger().info(f"Control mode updated: {self.control_mode}")

    def wheels_command_callback(self, msg):
        if self.control_mode == 'manual':
            self.handle_wheels_command(msg.data)
        else:
            self.get_logger().info("Ignoring wheels command in Autonomous mode.")

    def handle_wheels_command(self, command):
        # Format corect: <dirR,speedR,dirL,speedL>
        msg = None
        cmd = command.strip().split()
        if not cmd:
            self.get_logger().warning("Empty command received!")
            return
        action = cmd[0]
        pwm = int(cmd[1]) if len(cmd) > 1 and cmd[1].isdigit() else 0

        if action == "forward":
            msg = f"<1,{pwm},0,{pwm}>"
        elif action == "backward":
            msg = f"<0,{pwm},1,{pwm}>"
        elif action == "turnLeft":
            msg = f"<1,{pwm},0,0>"
        elif action == "turnRight":
            msg = f"<0,0,0,{pwm}>"
        elif action == "break":
            msg = "<1,0,1,0>"
        elif action == "start":
            msg = "<1,0,1,0>"
        elif action == "emergency":
            msg = "<0,0,0,0>"
        elif action == "endEmergency":
            msg = "<1,0,1,0>"
        else:
            self.get_logger().warning(f"Unknown command: {command}")
            return

        ros_msg = String()
        ros_msg.data = msg
        self.wheels_command_publisher.publish(ros_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = WheelsControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

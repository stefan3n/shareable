import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from forwardKinematicsXZ import get_potentiometer_values_for_position

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.control_mode = None
        self.create_subscription(
            String,
            'control_mode_topic',
            self.control_mode_callback,
            10
        )
        self.create_subscription(
            String,
            'arm_server_topic',
            self.arm_command_callback,
            10
        )
        self.arm_command_publisher = self.create_publisher(
            String,
            'arm_command_topic',
            10
        )

    def control_mode_callback(self, msg):
        self.control_mode = msg.data
        self.get_logger().info(f"Control mode updated: {self.control_mode}")

    def arm_command_callback(self, msg):
        if self.control_mode == 'manual':
            self.handle_arm_command(msg.data)
        else:
            self.get_logger().info("Ignoring arm command in Autonomous mode.")

    def handle_arm_command(self, command):
        # Move selected actuator format: <actuator_number, potentiometer_value>
        # Open/Close claw format: <4, 0/1>
        # Rotate claw format: <5, direction(0/1)>
        msg = None
        cmd = command.strip().split()
        if not cmd:
            self.get_logger().warning("Empty command received!")
            return

        action = cmd[0]
        potentiometerValue = None
        if len(cmd) > 1:
            try:
                potentiometerValue = int(cmd[1])
            except ValueError:
                self.get_logger().warning(f"Invalid potentiometer value: {cmd[1]}")
                return

        if action == "moveOne" and potentiometerValue is not None:
            msg = f"<1,{potentiometerValue}>"
        elif action == "moveTwo" and potentiometerValue is not None:
            msg = f"<2,{potentiometerValue}>"
        elif action == "moveThree" and potentiometerValue is not None:
            msg = f"<3,{potentiometerValue}>"
        elif action == "moveOneUp":
            msg = "<11>"
        elif action == "moveTwoUp":
            msg = "<21>"
        elif action == "moveThreeUp":
            msg = "<31>"
        elif action == "moveOneDown":
            msg = "<12>"
        elif action == "moveTwoDown":
            msg = "<22>"
        elif action == "moveThreeDown":
            msg = "<32>"
        elif action == "clawOpen":
            msg = "<4,0>"
        elif action == "clawClose":
            msg = "<4,1>"
        elif action == "rotateLeft":
            msg = "<5,0>"
        elif action == "rotateRight":
            msg = "<5,1>"
        elif action == "rotateArmLeft":
            msg = "<6,0>"
        elif action == "rotateArmRight":
            msg = "<6,1>"
        elif action == "initialPos":
            actuator1_value = 37  # Initial position for actuator 1 : Retracted
            actuator2_value = 952  # Initial position for actuator 2 : Extended
            actuator3_value = 957  # Initial position for actuator 3 : Extended
            msg = f"<123, {actuator1_value},{actuator2_value},{actuator3_value}>"
        elif action == "go" and len(cmd) == 4:
            try:
                x = int(cmd[1])
                y = int(cmd[2])  # y coordinate (not used in 2D kinematics)
                z = int(cmd[3])
                
                # Calculate potentiometer values for the target position using inverse kinematics
                pot_result = get_potentiometer_values_for_position(-x, z+10)
                
                if pot_result:
                    # Extract potentiometer values for each actuator
                    actuator1_value = pot_result['actuator1']
                    actuator2_value = pot_result['actuator2']
                    actuator3_value = pot_result['actuator3']
                    
                    # Send command with calculated potentiometer values
                    msg = f"<123, {actuator1_value},{actuator2_value},{actuator3_value}>"
                else:
                    self.get_logger().warning(f"Could not find valid actuator positions for target x={x}, z={z}")
                    return
                
            except ValueError:
                self.get_logger().warning("Invalid coordinates for go command.")
                return
        else:
            self.get_logger().warning(f"Unknown or incomplete command: {command}")
            return

        ros_msg = String()
        ros_msg.data = msg
        self.arm_command_publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

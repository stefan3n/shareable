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
        self.arm_position_publisher = self.create_publisher(
            String,
            'arm_position_topic',
            10
        )
        self.create_subscription(
            String,
            'arduino_response_topic',
            self.arduino_response_callback,
            10
        )
        # Send initial message to Arduino
        init_msg = String()
        init_msg.data = "99"
        self.arm_command_publisher.publish(init_msg)
        self.get_logger().info("Sent initial message '99' to Arduino.")
    def arduino_response_callback(self, msg):
        # Wait for response in the form <pot1,pot2,pot3>
        data = msg.data.strip()
        if not (data.startswith('<') and data.endswith('>')):
            self.get_logger().warning(f"Invalid Arduino response format: {data}")
            return
        try:
            pot_values = data[1:-1].split(',')
            if len(pot_values) != 3:
                self.get_logger().warning(f"Expected 3 potentiometer values, got: {pot_values}")
                return
            pot1 = int(pot_values[0])
            pot2 = int(pot_values[1])
            pot3 = int(pot_values[2])
        except Exception as e:
            self.get_logger().warning(f"Error parsing potentiometer values: {e}")
            return

        from forwardKinematicsXZ import potentiometer_values_to_angles, corrected_forward_kinematics
        theta1, theta2, theta3 = potentiometer_values_to_angles(pot1, pot2, pot3)
        _, (x, z) = corrected_forward_kinematics(theta1, theta2, theta3)
        pos_msg = String()
        pos_msg.data = f"{x:.2f},{z:.2f}"
        self.arm_position_publisher.publish(pos_msg)
        self.get_logger().info(f"Published arm position: X={x:.2f}, Z={z:.2f}")

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

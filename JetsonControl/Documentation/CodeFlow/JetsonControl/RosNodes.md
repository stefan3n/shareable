### [< back](JetsonControl.md)

#  **RosNodes** 

These nodes are not executed from the current package; instead, they must be located in a **ROS 2 workspace** on the Jetson.
The following resources may be useful: **[Workspace Creation Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)**, **[Package Creation Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)**.

***

*  **`arm_control_node`** / **`wheels_control_node`**

These nodes communicate with the corresponding Arduino boards by publishing messages to the `arm_command_topic` / `wheels_command_topic`, depending on the **current operating mode** of the vehicle.

The **control mode** is determined by reading the latest message from the `control_mode_topic`, where the `web_server` publishes either **manual** or **autonomous**.

In **manual mode**, the control node decodes the received messages and prepares the final commands that will be sent to the Arduino for execution.

* **`arduino_wheels_writer_node`** / **`arduino_arm_writer_node`**

These nodes subscribe to the `wheels_command_topic` / `arm_command_topic` and forward the received messages via **serial communication** to the corresponding Arduino board.

* **`arduino_wheels_reader_node`** / **`arduino_arm_reader_node`**

These nodes read messages from the Arduino over the **serial connection** and publish them to the `wheels_response_topic` / `arm_response_topic`.
This allows the responses to be monitored and also read by other nodes to confirm the actions performed by the Arduino.

**Note**: The Arduino boards are identified (for serial communication) not by port name (`/dev/ttyUSB0` or `/dev/ttyACM0`) but by **vendor ID** and **product ID**.
This approach is possible because the two boards are **different** — one is an official Arduino Uno, while the other is a clone with different IDs.

***

* **`forwardKinematics`**

TO COMPLETE
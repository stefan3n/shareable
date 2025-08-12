### [< back](JetsonControl.md)

# **Server**

We have created a vehicle control interface to test all commands and to provide the option for manual control of each component.

* **`index.html`**

Contains the HTML markup for the interface.

* **`style.css`**

* **`commands.js`**

Contains JavaScript functions that send HTTP **POST requests** to the server to control various actions of the robot.

* **`web_server.py`**

Starts a Flask-based web server that receives HTTP commands from the web interface and forwards them via **ROS 2 topics** to the robot.
Also provides video streaming from the robot’s cameras, including an optional detection mode.

* **`camera_utils.py`**

Handles video camera access, frame streaming (with or without **YOLO detection**), and image annotation for the robot’s web interface.
Provides functions for capturing frames and running object detection using YOLO.
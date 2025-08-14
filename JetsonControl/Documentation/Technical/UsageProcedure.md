### [< back](technical.md)

# **Usage procedure**

**Password**:
* Jetson Orin Nano (main board): `123jetson`
* Jetson Nano (secondary board): `jetson`

To avoid manually starting all ROS 2 nodes each time, we have created the following **helper scripts**:

* **`startall.sh`** – Starts the entire system: all ROS 2 nodes + the web_server.

* **`startarm.sh`** – Starts only the nodes related to arm control + the web_server.

* **`startwheels.sh`** – Starts only the nodes related to wheel control + the web_server.

> If you wish to manually start a specific node, follow these steps:

```
colcon build                                          # Build the workspace
source /opt/ros/<ros_distribution_name>/setup.bash    # Source the ROS 2 environment
source <workspace_name>/install/setup.bash            # Source your workspace environment
ros2 run <package_name> <node_name>                   # Run the node
```



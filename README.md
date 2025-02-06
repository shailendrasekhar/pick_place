**Pick and Place**

- Prerequisites
    - Ubuntu 22.04
    - ROS2 Humble (https://docs.ros.org/en/humble/index.html)
    - Gazebo (https://classic.gazebosim.org/)

- Usage
    - Create the workspace
        - mkdir -p "workspace_name"
        - git clone the repository.
        - colcon build
    - Visualization
        - ros2 launch robotic_arms_control rviz_bringup.launch.py
        
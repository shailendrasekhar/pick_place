**Pick and Place**

- Prerequisites
    - Ubuntu 22.04
    - ROS2 Humble (https://docs.ros.org/en/humble/index.html)
    - Gazebo (https://classic.gazebosim.org/)
    - Intel Realsense ROS2 (https://github.com/IntelRealSense/realsense-ros)

- Usage
    - Create the workspace
        - mkdir -p "workspace_name"
        - git clone the repository.
        - colcon build
    - Launch
        - ros2 launch master_panda master_launch.py
    
        
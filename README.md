**Pick and Place**

- Prerequisites
    - Ubuntu 22.04
    - ROS2 Humble (https://docs.ros.org/en/humble/index.html)
    - Gazebo (https://classic.gazebosim.org/)
    - Intel Realsense ROS2 (https://github.com/IntelRealSense/realsense-ros)
    - Roboticstoolbox (https://petercorke.github.io/robotics-toolbox-python/intro.html)
    - MoveIT2 Humble (https://moveit.picknik.ai/humble/index.html)

- Usage
    - Create the workspace
        - mkdir -p "workspace_name"
        - git clone the repository.
        - colcon build
    - Launch
        - ros2 launch master_panda master_launch.py
    - Demo with RoboticsToolBox
        - ros2 run master_panda roboticstoolbox 0.7 0.4 0.1
    
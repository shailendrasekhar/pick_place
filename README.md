**Pick and Place**

- Prerequisites
    - Ubuntu 22.04
    - ROS2 Humble (https://docs.ros.org/en/humble/index.html)
    - Gazebo (https://classic.gazebosim.org/)
    - Intel Realsense ROS2 (https://github.com/IntelRealSense/realsense-ros)


Create the workspace
    - mkdir -p "workspace_name"
    - git clone the repository.
    - colcon build

- Usage
    - RoboticsToolBox
        - Launch
            - ros2 launch master_panda master_launch.py
        - Demo with RoboticsToolBox
            - ros2 run master_panda roboticstoolbox 0.7 0.4 0.1
    - Moveit2
        - Launch
            - ros2 launch manipulator gazebo.launch.py
            - ros2 launch manipulator moveit2.launch.py object:="$object_name"
                - $object_name:
                    - square_object_red (default)
                    - square_object_green 
                    - square_object_blue

- Videos
    - Robotics ToolBox
        - Trajectory Control and Camera with RTB
            [![Demo](https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/camera_depth_rgb_rtb.gif)](https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/camera_depth_rgb_rtb.mov)

    - Moveit2
        - Controlling from Rviz
            [![Demo](https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/controlling_gazebo_robot_from_rviz_moveit2.gif)](https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/controlling_gazebo_robot_from_rviz_moveit2.mov)

        - Pick and Place preliminary pipeline

<p align="center">
  <a href="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/green_object_pick_place_moveit2.mov">
    <img src="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/green_object_pick_place_moveit2.gif" width="30%">
  </a>
  <a href="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/blue_object_pick_place_moveit2.mov">
    <img src="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/blue_object_pick_place_moveit2.gif" width="30%">
  </a>
  <a href="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/red_object_pick_place_moveit2.mov">
    <img src="https://raw.githubusercontent.com/shailendrasekhar/pick_place/main/videos/red_object_pick_place_moveit2.gif" width="30%">
  </a>
</p>      


        
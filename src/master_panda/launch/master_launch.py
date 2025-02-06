from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory,get_package_prefix
import os

def generate_launch_description():
    pkg_description = get_package_share_directory('robotic_arms_control')
    urdf_file = os.path.join(pkg_description, "urdf", 'panda_arm.urdf')

    mesh_pkg_share_dir = os.pathsep + os.path.join(get_package_prefix('franka_description'), 'share')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += mesh_pkg_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  mesh_pkg_share_dir

    
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="JSP",
        output="screen",
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="RSP",
        output="screen",
        parameters=[{'robot_description': urdf_content}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_gui",
        output="screen"
    )

    start_gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
    spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            respawn=False,
            arguments=["-topic", "/robot_description", "-entity", "panda_arm"])

    
    joint_state_broadcaster_node = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_state_broadcaster"])
    
    joint_trajectory_controller_node = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_trajectory_controller"])
    
    rviz_bringup = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]

    gazebo_bringup = [
        robot_state_publisher_node,
        start_gazebo,
        spawn_robot,
        joint_state_broadcaster_node,
        joint_trajectory_controller_node
    ]

    return LaunchDescription(gazebo_bringup)

if __name__ == '__main__':
    generate_launch_description()
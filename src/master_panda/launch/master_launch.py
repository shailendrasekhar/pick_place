from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path, get_package_prefix, get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkgPath = get_package_share_directory('master_panda')

    pkg_panda_description = os.path.join(pkgPath, 'urdf' , 'panda.urdf.xacro')
    square_object_sdf_file = os.path.join(pkgPath, 'urdf', 'square.urdf.xacro')
    rviz_config = os.path.join(pkgPath, 'rviz2', 'default.rviz')
    mesh_pkg_share_dir = os.pathsep + os.path.join(pkgPath, 'share', 'master_panda', 'models')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += mesh_pkg_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  mesh_pkg_share_dir


    robot_description = ParameterValue(
        Command(['xacro ', pkg_panda_description]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # output='screen',
        arguments=['-d', rviz_config],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),'launch/gazebo.launch.py')
        )
    )

    spawn_panda = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "panda"])
    
    spawn_square = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_square_object',
            arguments=[
                '-entity', 'square_object',
                '-file', square_object_sdf_file,
                '-x', '0.7', '-y', '0.4', '-z', '0.1'
            ],
            output='screen'
        )
    
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
    
    # Load MoveIt! configuration
    # moveit_config = (
    #     MoveItConfigsBuilder("panda_pick_place")
    #     .robot_description(file_path="urdf/panda.urdf.xacro")
    #     .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )

    # # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    # move_group_capabilities = {
    #     "capabilities": "move_group/ExecuteTaskSolutionCapability"
    # }

    # # Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #         move_group_capabilities,
    #     ],
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_panda,
        spawn_square,
        rviz_node,
        joint_state_broadcaster_node,
        joint_trajectory_controller_node,
        # run_move_group_node
    ]) 
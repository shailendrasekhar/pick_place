from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path, get_package_prefix, get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    objpkgPath = get_package_share_directory('manipulator_moveit_config')
    pkg_panda_description = os.path.join(objpkgPath, 'config', 'manipulator.urdf.xacro')
    square_object_sdf_file = os.path.join(objpkgPath, 'config', 'square.urdf.xacro')

    pkgPath = get_package_share_directory('manipulator')
    rviz_config = os.path.join(pkgPath, 'rviz2', 'default.rviz')
    mesh_pkg_share_dir = os.pathsep + os.path.join(pkgPath, 'share', 'manipulator', 'models')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += mesh_pkg_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] = mesh_pkg_share_dir

    robot_description = ParameterValue(
        Command(['xacro ', pkg_panda_description]),
        value_type=str
    )

    # Enable simulation time for the robot_state_publisher node.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    spawn_panda = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='robot_spawner',
        output='screen',
        arguments=[
            "-topic", "/robot_description",
            "-entity", "panda"
        ]
    )
    
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
        arguments=["joint_state_broadcaster"]
    )

    joint_trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=["manipulator_arm_controller"]
    )
    
    # Load MoveIt! configuration.
    moveit_config = (
        MoveItConfigsBuilder("manipulator")
        .robot_description(file_path="config/manipulator.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Enable simulation time for the move_group node.
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {'use_sim_time': True}
        ],
    )

    # Enable simulation time for RViz.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}
        ],
    )
 
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_panda,
        spawn_square,
        rviz_node,
        joint_state_broadcaster_node,
        joint_trajectory_controller_node,
        run_move_group_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessExit


def generate_launch_description():
#Import all file paths
    objpkgPath = get_package_share_directory('manipulator_moveit_config')
    robot_description_path = os.path.join(objpkgPath, 'config', 'manipulator.urdf.xacro')
    robot_semantic_path = os.path.join(objpkgPath, 'config', 'manipulator.srdf')
    kinematics_yaml_path = os.path.join(objpkgPath, 'config', 'kinematics.yaml') 
    ompl_yaml_path = os.path.join(objpkgPath, 'config', 'ompl_planning.yaml')
    moveit_controller_path = os.path.join(objpkgPath, 'config', 'moveit_controllers.yaml')
    joint_limits_path = os.path.join(objpkgPath, 'config', 'joint_limits.yaml')
    square_red_file = os.path.join(objpkgPath, 'config', 'square_red.urdf.xacro')
    square_blue_file = os.path.join(objpkgPath, 'config', 'square_blue.urdf.xacro')
    square_green_file = os.path.join(objpkgPath, 'config', 'square_green.urdf.xacro')
    warehouse_db_path = "/home/humble/Projects/pick_place/src/manipulator_moveit_config/config/warehouse_db.sqlite"

    pkgPath = get_package_share_directory('manipulator')
    rviz_config = os.path.join(pkgPath, 'rviz2', 'default.rviz')
    mesh_pkg_share_dir = os.pathsep + os.path.join(pkgPath, 'share', 'manipulator', 'models')

    ifra_plugin_path = os.path.join(
    get_package_share_directory('ros2_linkattacher'), 'lib')

    os.environ['GAZEBO_PLUGIN_PATH'] += os.pathsep + ifra_plugin_path
    

# Objects
    colors_positions = [
        ("Red", square_red_file, 0.0, 0.6, 0.0),
        ("Blue", square_blue_file, 0.4, 0.4, 0.0),
        ("Green", square_green_file, 0.6, 0.0, 0.0)
    ]

    squares = []

    for color, sdf_file, x, y, z in colors_positions:
        
        squares.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_square_{color.lower()}',
            arguments=[
                '-entity', f'square_object_{color.lower()}',
                '-file', sdf_file,
                '-x', str(x), '-y', str(y), '-z', str(z),
            ],
            output='screen'
        ))

# Gazebo Relevant 
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += mesh_pkg_share_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] = mesh_pkg_share_dir

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )


    load_state_controller = ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','manipulator_arm_controller'],
        output='screen'
    )

    load_hand_controller = ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','manipulator_hand_controller'],
        output='screen'
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
            "-entity", "manipulator"
        ]
    )

# Moveit Relevant
    with open(kinematics_yaml_path, "r") as file:
        kinematics_dict = yaml.safe_load(file)

    moveit_config = (
        MoveItConfigsBuilder("manipulator")
        .robot_description(file_path = robot_description_path)
        .robot_description_semantic(file_path = robot_semantic_path)
        .trajectory_execution(file_path = moveit_controller_path)
        .robot_description_kinematics(file_path=str(kinematics_yaml_path))
        .joint_limits(file_path = joint_limits_path)
        .planning_scene_monitor(
            { "publish_planning_scene": True, 
             "publish_geometry_updates": True, 
             "publish_state_updates": True, 
             "publish_transforms_updates": True, 
             "publish_robot_description":True, 
             "publish_robot_description_semantic":True
            }
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True, 
                     "use_gui" : False}],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "manipulator_link0"],
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_db_path,  
        "warehouse_port": 0  
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            # warehouse_ros_config,
            {"robot_description_kinematics": kinematics_dict},
            {'use_sim_time': True},
            {"ompl_planning":ompl_yaml_path}
        ],
        arguments=['--ros-args','--log-level','info'],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            # warehouse_ros_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
            {'tf_buffer_duration':10}
        ],
    )

    planning_scene = Node(
        package="manipulator",
        executable="planning_demo",
        output="screen"
    )

    moveit2_demo_node = Node(
        package="manipulator",
        executable="moveit2_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time":True}
        ],
    )

    delayed_rviz_node = TimerAction(
        period=5.0,  # Delay RViz startup by 5 seconds
        actions=[rviz_node]
    )
 
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_panda,
                on_exit=[load_state_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_state_controller,
                on_exit=[load_arm_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_hand_controller]
            )
        ),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_panda]+
        squares
        +[
        
        run_move_group_node,
        # delayed_rviz_node,
        # TimerAction(period=5.0,actions=[planning_scene]),
        TimerAction(period=15.0,actions=[moveit2_demo_node])
        ]
)

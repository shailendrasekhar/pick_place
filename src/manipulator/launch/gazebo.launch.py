from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():
#Import all file paths
    objpkgPath = get_package_share_directory('manipulator_moveit_config')
    robot_description_path = os.path.join(objpkgPath, 'config', 'manipulator.urdf.xacro')
    square_red_file = os.path.join(objpkgPath, 'config', 'square_red.urdf.xacro')
    square_blue_file = os.path.join(objpkgPath, 'config', 'square_blue.urdf.xacro')
    square_green_file = os.path.join(objpkgPath, 'config', 'square_green.urdf.xacro')


    pkgPath = get_package_share_directory('manipulator')
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True, 
                     "use_gui" : False,
                     "update_rate":200,}],
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
    )
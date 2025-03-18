from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    selected_object_arg = DeclareLaunchArgument(
        "object",
        default_value="square_object_red",
    )

    target_pose_x_arg = DeclareLaunchArgument(
        "target_pose_x",
        default_value="0.4",
    )

    target_pose_y_arg = DeclareLaunchArgument(
        "target_pose_y",
        default_value="-0.4",
    )

    target_pose_z_arg = DeclareLaunchArgument(
        "target_pose_z",
        default_value="0.05",
    )

    target_pose_w_arg = DeclareLaunchArgument(
        "target_pose_w",
        default_value="1.0",
    )


#Import all file paths
    objpkgPath = get_package_share_directory('manipulator_moveit_config')
    robot_description_path = os.path.join(objpkgPath, 'config', 'manipulator.urdf.xacro')
    robot_semantic_path = os.path.join(objpkgPath, 'config', 'manipulator.srdf')
    kinematics_yaml_path = os.path.join(objpkgPath, 'config', 'kinematics.yaml') 
    ompl_yaml_path = os.path.join(objpkgPath, 'config', 'ompl_planning.yaml')
    moveit_controller_path = os.path.join(objpkgPath, 'config', 'moveit_controllers.yaml')
    joint_limits_path = os.path.join(objpkgPath, 'config', 'joint_limits.yaml')
    warehouse_db_path = "/home/humble/Projects/pick_place/src/manipulator_moveit_config/config/warehouse_db.sqlite"

    pkgPath = get_package_share_directory('manipulator')
    rviz_config = os.path.join(pkgPath, 'rviz2', 'default.rviz')

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
            {"use_sim_time":True},
            {"selected_object_id": LaunchConfiguration("object")},
            {"target_pose_x": LaunchConfiguration("target_pose_x")},
            {"target_pose_y": LaunchConfiguration("target_pose_y")},
            {"target_pose_z": LaunchConfiguration("target_pose_z")},
            {"target_pose_w": LaunchConfiguration("target_pose_w")}
        ],
    )

    delayed_rviz_node = TimerAction(
        period=5.0,  # Delay RViz startup by 5 seconds
        actions=[rviz_node]
    )
 
    return LaunchDescription([    
        selected_object_arg,
        target_pose_x_arg,
        target_pose_y_arg,
        target_pose_z_arg,
        target_pose_w_arg,
        run_move_group_node,
        # delayed_rviz_node,
        TimerAction(period=5.0,actions=[moveit2_demo_node])
        ]
)

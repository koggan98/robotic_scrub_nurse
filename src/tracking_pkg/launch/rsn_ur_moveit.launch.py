import os
import logging
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    tracking_share = Path(get_package_share_directory("tracking_pkg"))
    urdf_path = tracking_share / "urdf" / "rsn_ur.urdf.xacro"
    srdf_path = tracking_share / "srdf" / "ur.srdf.xacro"
    joint_limits_path = PathJoinSubstitution(
        [FindPackageShare("tracking_pkg"), "config", "ur3e_joint_limits.yaml"]
    )

    previous_logging_disable = logging.root.manager.disable
    logging.disable(logging.WARNING)
    moveit_config_builder = MoveItConfigsBuilder(
        robot_name="ur",
        package_name="ur_moveit_config",
    )
    logging.disable(previous_logging_disable)

    moveit_config = (
        moveit_config_builder
        .robot_description(
            urdf_path,
            {
                "ur_type": ur_type,
                "name": ur_type,
                "robot_ip": robot_ip,
                "tf_prefix": tf_prefix,
                "joint_limit_params": joint_limits_path,
                "safety_limits": safety_limits,
                "safety_pos_margin": safety_pos_margin,
                "safety_k_position": safety_k_position,
            },
        )
        .robot_description_semantic(srdf_path, {"name": ur_type})
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            ),
            DeclareLaunchArgument("launch_servo", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
            ),
            DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            move_group_node,
            rviz_node,
            servo_node,
        ]
    )

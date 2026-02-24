from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    tracking_rviz = LaunchConfiguration("tracking_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "launch_servo": launch_servo,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
                    ),
                    " ur_type:=",
                    ur_type,
                    " name:=ur",
                    " prefix:=",
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
                    ),
                    " ur_type:=",
                    ur_type,
                    " name:=ur",
                ]
            ),
            value_type=str,
        )
    }

    robot_description_kinematics_path = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(tracking_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("tracking_pkg"), "rviz", "view_robot_tracking.rviz"]
            ),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    tracking_loop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tracking_pkg"), "launch", "loop_launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("tracking_rviz", default_value="true"),
            DeclareLaunchArgument("launch_servo", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetEnvironmentVariable("LC_NUMERIC", "en_US.UTF-8"),
            ur_moveit_launch,
            rviz_node,
            tracking_loop_launch,
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    update_rate_config_file = LaunchConfiguration("update_rate_config_file")

    description_launchfile = PathJoinSubstitution(
        [FindPackageShare("tracking_pkg"), "launch", "rsn_ur_rsp.launch.py"]
    )

    ur_control_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": launch_rviz,
            "description_launchfile": description_launchfile,
            "use_mock_hardware": use_mock_hardware,
            "headless_mode": headless_mode,
            "launch_dashboard_client": launch_dashboard_client,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "update_rate_config_file": update_rate_config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("launch_dashboard_client", default_value="true"),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
            ),
            DeclareLaunchArgument("activate_joint_controller", default_value="true"),
            DeclareLaunchArgument(
                "update_rate_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tracking_pkg"), "config", "ur3e_update_rate.yaml"]
                ),
            ),
            ur_control_launch,
        ]
    )

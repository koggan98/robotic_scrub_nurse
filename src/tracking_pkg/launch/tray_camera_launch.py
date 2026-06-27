import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '239222302690')

    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': 'tray_camera',
                'camera_namespace': '',
                'serial_no': f"'{tray_cam_serial}'",
                'enable_color': 'true',
                # Depth disabled: tool_detection / world_model_builder project onto a
                # fixed tool plane (fixed_tool_plane_z_m), so tray depth is unused. Turning
                # it off roughly halves this camera's USB bandwidth and eases the RealSense
                # port crashes on the Spark. Color resolution is unchanged (models trained on it).
                'enable_depth': 'false',
                'rgb_camera.color_profile': '1280,720,30',
                'align_depth.enable': 'false',
                'spatial_filter.enable': 'false',
                'temporal_filter.enable': 'false',
                'hole_filling_filter.enable': 'false',
                'decimation_filter.enable': 'false',
                'enable_sync': 'true',
                'publish_tf': 'false',
            }.items(),
        ),
    ])

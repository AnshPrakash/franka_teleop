#!/usr/bin/env python3
"""
multi_zed_required.launch.py

Launch multiple instances of zed_wrapper/zed_camera.launch.py.

REQUIREMENTS:
 - The launch argument 'cameras' is required and must be provided.
 - 'cameras' value must be a comma-separated list of entries of the form:
       camera_name:serial_number
   Example: cameras:=camera_left:29934236,camera_right:21177909

Usage:
  # REQUIRED: provide cameras
  ros2 launch franka_teleop multi_zed_required.launch.py \
    cameras:=camera_left:29934236,camera_right:21177909

Notes:
 - The script will validate entries and abort with an error if:
   * the cameras arg is missing/empty
   * an entry lacks ':' or has empty name/serial
   * camera_name is duplicated
 - You can still set camera_model (default 'zed2') if desired.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def _setup(context, *args, **kwargs):
    """OpaqueFunction callback: parse cameras list and include zed launch per camera."""
    cameras_raw = context.launch_configurations.get("cameras", "")
    camera_model = context.launch_configurations.get("camera_model", "zed")

    # Enforce required 'cameras' argument
    if not cameras_raw:
        raise RuntimeError(
            "[multi_zed_required] REQUIRED launch argument 'cameras' is missing or empty.\n"
            "Provide a comma-separated list of camera entries in the form:\n"
            "  camera_name:serial_number\n"
            "Example:\n"
            "  ros2 launch franka_teleop multi_zed_required.launch.py "
            "cameras:=camera_left:29934236,camera_right:21177909"
        )

    # find zed_wrapper launch file
    try:
        zed_pkg_share = get_package_share_directory('zed_wrapper')
    except Exception as e:
        raise RuntimeError(f"[multi_zed_required] Could not find package 'zed_wrapper': {e}")

    zed_launch_path = os.path.join(zed_pkg_share, 'launch', 'zed_camera.launch.py')
    if not os.path.exists(zed_launch_path):
        raise RuntimeError(f"[multi_zed_required] Could not find expected launch file: {zed_launch_path}")

    # parse "name:serial,name2:serial2,..."
    cameras = []
    seen_names = set()
    for raw in [e.strip() for e in cameras_raw.split(',') if e.strip()]:
        if ':' not in raw:
            raise RuntimeError(f"[multi_zed_required] Camera entry '{raw}' is invalid. Each entry must be 'camera_name:serial_number'.")
        name, serial = raw.split(':', 1)
        name = name.strip()
        serial = serial.strip()
        if not name:
            raise RuntimeError(f"[multi_zed_required] Camera entry '{raw}' has an empty camera_name.")
        if not serial:
            raise RuntimeError(f"[multi_zed_required] Camera entry '{raw}' has an empty serial_number.")
        if name in seen_names:
            raise RuntimeError(f"[multi_zed_required] Duplicate camera_name detected: '{name}'. Each camera_name must be unique.")
        seen_names.add(name)
        cameras.append((name, serial))

    actions = []
    actions.append(LogInfo(msg=f"[multi_zed_required] Launching {len(cameras)} camera(s): " + ", ".join(f"{n}:{s}" for n, s in cameras)))
    # include the zed_camera launch for each camera, passing camera-specific args.
    for idx, (cam_name, serial) in enumerate(cameras, start=1):
        launch_arguments = {
            'camera_model': camera_model,
            'camera_name': cam_name,
            'serial_number': serial,
            # Add more per-camera defaults here if the included launch supports them.
            # e.g. 'gpu_id': '0', 'publish_tf': 'true', 'depth_mode': 'PERFORMANCE'
        }
        
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch_path),
                launch_arguments=((k, str(v)) for k, v in launch_arguments.items())
            )
        )
        

        actions.append(LogInfo(msg=f"[multi_zed_required] Included zed launch for '{cam_name}' (serial {serial})"))

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    # Declare 'cameras' without a helpful default so it's clearly required.
    ld.add_action(DeclareLaunchArgument(
        'cameras',
        default_value='',
        description="REQUIRED. Comma-separated list of cameras as camera_name:serial_number (e.g. camera_left:29934236,camera_right:21177909)"
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_model',
        default_value='zed2',
        description='Camera model to pass to zed_camera.launch.py (default: zed2)'
    ))

    ld.add_action(OpaqueFunction(function=_setup))

    return ld

#  ros2 launch franka_teleop multi_zed_required.launch.py  cameras:=camera_left:29934236,camera_right:21177909 camera_model:=zed2
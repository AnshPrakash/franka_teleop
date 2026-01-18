#!/usr/bin/env python3
"""
Launch two installed console-script entrypoints (by name) and shut everything down
if any of them exits. Pass --ros-args --log-level to control logging.

Usage:
  # ensure your workspace is sourced so install/.../bin is on PATH
  source install/setup.bash
  ros2 launch franka_teleop oculus_teleop.launch.py

console-scripts are installed into install/<pkg>/lib/<pkg>.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
from launch.events import Shutdown

def generate_launch_description():
    # arguments
    shutdown_on_exit_arg = DeclareLaunchArgument(
        'shutdown_on_exit',
        default_value='true',
        description='If true, the launcher will shut down when any of the two processes exits.'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level (debug, info, warn, error, fatal)'
    )

    ld = LaunchDescription([
        shutdown_on_exit_arg,
        log_level_arg,
        LogInfo(msg=['Starting two-node launcher (entrypoint mode). Use log_level to increase verbosity.'])
    ])

    # entrypoint names (as installed wrappers on PATH)
    entrypoint1 = 'oculus_transforms_vrpolicy'
    entrypoint2 = 'teleop_quest'

    # build ExecuteProcess actions that call the executables by name (rely on PATH)
    proc1 = ExecuteProcess(
        cmd=[entrypoint1, '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        shell=False,
        name='oculus_transforms_proc',
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    proc2 = ExecuteProcess(
        cmd=[entrypoint2, '--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        shell=False,
        name='teleop_quest_proc',
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    # add processes to the launch description
    ld.add_action(proc1)
    ld.add_action(proc2)

    # on-exit behavior
    shutdown_msg = (
        "One of the processes exited. Shutting down the other process.\n"
        "If this was caused by a runtime error, please check your Meta Quest connection and run `adb devices`\n"
        "to ensure the headset is visible to the host. Also inspect the Python stack trace printed above."
    )

    # register event handlers to shut down if any process exits (when shutdown_on_exit is true)
    # Note: OnProcessExit is given the actual action objects (`proc1` and `proc2`).
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=proc1,
            on_exit=[
                LogInfo(msg=["oculus_transforms exited. ", shutdown_msg]),
                EmitEvent(event=Shutdown())
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=proc2,
            on_exit=[
                LogInfo(msg=["teleop_quest exited. ", shutdown_msg]),
                EmitEvent(event=Shutdown())
            ]
        )
    ))

    return ld

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_teleop'),
                'franka_realworld.launch.py'
            ])
        )
    )

    oculus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_teleop'),
                'oculus_teleop.launch.py'
            ])
        )
    )

    return LaunchDescription([
        franka_launch,
        oculus_launch,
    ])

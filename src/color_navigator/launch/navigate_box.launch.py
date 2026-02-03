from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the GAZEBO_MODEL_PATH environment variable to include this package's models
    gazebo_model_path = os.path.join(get_package_share_directory('color_navigator'), 'models')

    world_file = os.path.join(get_package_share_directory('color_navigator'), 'worlds', 'box_world.world')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path
        ),

        # Start gzserver with the provided world file. Using gzserver directly
        # avoids relying on a gazebo wrapper executable from the gazebo_ros package.
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file],
            output='screen'
        ),

        # Start controller and detector console scripts directly from the
        # installed package bin directory. This avoids issues where the
        # launch system expects CMake-style libexec directories.
        ExecuteProcess(
            cmd=[os.path.abspath(os.path.join(get_package_share_directory('color_navigator'), '..', '..', 'bin', 'robot_controller'))],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.abspath(os.path.join(get_package_share_directory('color_navigator'), '..', '..', 'bin', 'color_detector'))],
            output='screen'
        ),
    ])
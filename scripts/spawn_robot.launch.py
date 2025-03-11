import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():
    # Ursprünglicher URDF-Pfad aus deiner vorherigen Version
    urdf_file = os.path.expanduser('~/ros2_ws/src/rviz/argo_drive_robot.urdf')

    return LaunchDescription([
        # Starte Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
        ),

        # Starte den Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file, 'r').read()}],
        ),

        # Spawne den Roboter in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'argo_drive_robot',
                 '-file', urdf_file,
                 '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
    ])

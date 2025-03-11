import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name = 'ros2_ws_robot_launch'

    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(get_package_share_directory('ros2_ws_robot_launch'),'config','argo_drive_robot.urdf.xacro')
    
    doc = xacro.parse(open(xacro_file))

    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    #print(params)

    robot_description = xacro.process_file(xacro_file)

    rviz_config_file =  os.path.join(get_package_share_directory(package_name),'config','rviz_config.rviz')

    controller_manager_params = os.path.join(get_package_share_directory(package_name), 'params', 'control.yaml')

    bridge_params = os.path.join(get_package_share_directory(package_name), 'params', 'gz_bridge.yaml')

    world_file = os.path.join(get_package_share_directory(package_name), 'config', 'obstacles.world')

    laser_merger_params = os.path.join(get_package_share_directory(package_name), 'params', 'laser_scan_merger.yaml')




    # Control Manager Starten



    """
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control",
        output="screen",
        parameters=[controller_manager_params,{'use_sim_time': use_sim_time}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )


    tricycle_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tricycle_controller"],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=30.0, actions=[controller_manager])

    delayed_tricycle_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[tricycle_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    check_controller_manager = ExecuteProcess(
        cmd=['ros2','service','find', '/controller_manager/list_controllers'],
        output= 'screen'
    )
    """
    node_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                    '-name', 'tricycle',
                    '-allow_renaming', 'true'
                    ],
    )


    load_joint_state_controller = ExecuteProcess(
        output = 'screen',
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'] ,   
    )

    load_joint_trajectory_controller = ExecuteProcess(
        output = 'screen',
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'tricycle_controller'],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
    )

    merge_scan = Node(
        package="ros2_ws_robot_launch",
        executable="scan_merger.py",
        name="scan_merger",
        output="screen",
        parameters=[{
            'scan1_topic': '/scan1',
            'scan2_topic': '/scan2',
            'output_topic': '/combined_scan',
            'scan1_frame': 'scan1',
            'scan2_frame': 'scan2',
            'target_frame': 'scan_middle_link',
            'use_sim_time': True
        }]
    )

    slam_params_file = os.path.join(get_package_share_directory(package_name), 'params', 'mapper_params_online_async.yaml')
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'params','twist_mux.yaml')

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/tricycle_controller/cmd_vel')]
    )

    delayed_scan_merger = TimerAction(
        period=10.0,  # Warte 3 Sekunden, damit controller_manager bereit ist
        actions=[merge_scan]
    )

    delayed_slam = TimerAction(
        period=15.0,  # Warte 3 Sekunden, damit controller_manager bereit ist
        actions=[start_async_slam_toolbox_node]
    )



    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        ros_gz_bridge,

        # Gazebo starten

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
                ),
             launch_arguments={'ign_args': ['-r ', world_file], 'on_exit_shutdown': 'true'}.items()
        ),

        #Starten Joint state controller
        RegisterEventHandler(
             event_handler=OnProcessExit(
             target_action=spawn_entity,
                 on_exit=[load_joint_state_controller],
             )
        ),

        #Starten trajectory controller
        RegisterEventHandler(
             event_handler=OnProcessExit(
                 target_action=load_joint_state_controller,
                 on_exit= [load_joint_trajectory_controller],
             )
        ),

        # Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     name="ros2_control",
        #     output="screen",
        #     parameters=[controller_manager_params, {'use_sim_time': use_sim_time}], 
        # ),

        # Node(
        #      package='ros2_ws_robot_launch',
        #      executable='ethercat_node_slave1',
        #      name='ethercat_node_slave1',
        #      output='screen'
        # ),

        #Robot State Publisher starten
        node_robot_state_publisher,
        spawn_entity,
        delayed_scan_merger,
        twist_mux,
        delayed_slam,

        #RViz starten
        Node(
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_config_file],
             output='screen'
        ),


    ])

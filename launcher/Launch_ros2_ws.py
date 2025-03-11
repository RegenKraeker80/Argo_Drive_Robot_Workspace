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
from launch.substitutions import Command

def generate_launch_description():

    package_name = 'ros2_ws_robot_launch'

    rviz_config_file =  os.path.join(get_package_share_directory(package_name),'config','rviz_config.rviz')

    use_sim_time = False 

    xacro_file = os.path.join(get_package_share_directory('ros2_ws_robot_launch'),'config','argo_drive_real_robot.urdf.xacro')
    
    doc = xacro.parse(open(xacro_file))

    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml(), 'use_sim_time': False}

    node_robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    slam_params_file = os.path.join(get_package_share_directory(package_name), 'params', 'mapper_params_online_async.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'params','twist_mux.yaml')

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/tricycle_controller/cmd_vel')]
    )

    load_joint_state_controller = ExecuteProcess(
        output = 'screen',
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'] ,   
    )

    load_joint_trajectory_controller = ExecuteProcess(
        output = 'screen',
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'tricycle_controller'],
    )

    controller_manager_params = os.path.join(get_package_share_directory(package_name), 'params', 'control_robot.yaml')

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control",
        output="screen",
        parameters=[{'robot_description': robot_description},
                    controller_manager_params]
    )
        

    delayed_controller_manager = TimerAction(period=20.0,actions=[controller_manager])

    delayed_joint_broad_spawner = TimerAction(
        period=30.0,  
        actions=[load_joint_state_controller]
    )

    delayed_tricycle_drive_spawner = TimerAction(
        period=30.0,  # Warte 5 Sekunden, um sicherzustellen, dass joint_state_broadcaster geladen ist
        actions=[load_joint_trajectory_controller]
    )


    
    scan_merger= Node(
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
            'use_sim_time': False
        }]
    )
        
    return LaunchDescription([
        
    	
    	#Launch robot_state_publisher Robot 
        node_robot_state_publisher,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_tricycle_drive_spawner,

             
 	    #Launch Sick NanoScan3 / Node : /scan1  
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="scaner1",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan1",
                 "sensor_ip": "192.168.1.2",
                 "host_ip": "192.168.1.4",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            #Remapping des Topings
            remappings=[
                ('/scan','/scan1')
            ]
        ),
        
        #Launch Sick MicroScan3 / Node : /scan2  
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="scaner2",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan2",
                 "sensor_ip": "192.168.1.3",
                 "host_ip": "192.168.1.4",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            #Remapping des Topings
            remappings=[
                ('/scan','/scan2')
            ]
        ),

        scan_merger,

       
        
        #Launch RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

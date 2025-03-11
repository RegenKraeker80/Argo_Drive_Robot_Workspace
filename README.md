Argo Drive Robot Workspace

This workspace is designed to control a tricycle system robot equipped with the ArgoDrive System from ebm-papst, which is controlled via EtherCAT.
Getting Started

Launch the Robot:
Run the following command to start the robot:

    ros2 launch ros2_ws_robot_launch Launch_ros2_ws.py

Launch the Simulation:
To start the simulation, use the following command:

    ros2 launch ros2_ws_robot_launch launch_sim.launch.py

Features:

Scan Merger Node:
This workspace includes a Scan Merger Node that merges two laser scans into a single virtual scanner.

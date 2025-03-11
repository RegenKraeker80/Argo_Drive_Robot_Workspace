from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_ws_robot_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_dir={'': 'scripts'},
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launcher'), glob('launcher/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Walbrunn',
    maintainer_email='awalbrunn@witron.de',
    description='Launch-Paket für den Roboter',
    license='Apache-2.0',
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'scan_merger = ros2_ws_robot_launch.scripts.scan_merger:main',
            'scan_merger = ros2_ws_robot_launch.launcher.scan_merger:main',
            'spawn_robot = ros2_ws_robot_launch.scripts.spawn_robot:main',
            'launch_sim = ros2_ws_robot_launch.launcher.launch_sim:main',
            'launch_robot = ros2_ws_robot_launch.launcher.launch_robot:main',
            'joystick = ros2_ws_robot_launch.launcher.joystick:main',
            'launch_nav2 = ros2_ws_robot_launch.launcher.launch_nav2',
            'spawner = controller_manager.spawner:main',
        ],
    },
)

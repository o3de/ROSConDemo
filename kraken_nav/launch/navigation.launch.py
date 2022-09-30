import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([str(pathlib.Path(__file__).parent.absolute().joinpath('slam.launch.py'))])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([str(pathlib.Path(get_package_share_directory("nav2_bringup")).joinpath('launch', 'navigation_launch.py'))]),
            launch_arguments = {
                'params_file': str(pathlib.Path(__file__).parent.absolute().joinpath('config', 'navigation_params.yaml'))
            }.items()
        ),
        Node(
            package='o3de_kraken_nav',
            executable='twist_to_ackermann',
            name='twist_to_ackermann',
            output={
                'stdout': 'log'
            }
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='slam',
            output={
                'stdout': 'log',
            },
            arguments=[
                '-d', str(pathlib.Path(__file__).parent.absolute().joinpath('config', 'config.rviz')),
            ]
        ),
    ])
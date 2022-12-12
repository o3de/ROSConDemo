#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#


import pathlib
from unicodedata import name
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def substitute_namespace(namespace, value):
    if not namespace:
        return TextSubstitution(text=value)
    else:
        return PythonExpression(['str("', namespace, '")', "+", f"'/{value}'"])
    
def substitute_name(namespace, value):
    if not namespace:
        return TextSubstitution(text=value)
    else:
        return PythonExpression(['str("', namespace, '")', "+", f"'_{value}'"])

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )

    use_slam = LaunchConfiguration('use_slam')
    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
    )
    
    rviz = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
    )

    package_dir = get_package_share_directory("o3de_kraken_nav")
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory("nav2_bringup")

    nav2_params_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'navigation_multi_params.yaml'))
    bt_xml_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'bt.xml'))
    slam_params_file = str(pathlib.Path(package_dir).joinpath('launch', 'config', 'slam_multi_params.yaml'))

    nav_param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml_file,
        'robot_base_frame': substitute_namespace(namespace, "base_link"),
        'local_costmap.local_costmap.ros__parameters.global_frame': substitute_namespace(namespace, "odom"),
        'global_costmap.global_costmap.ros__parameters.global_frame': substitute_namespace(namespace, "map"),
        'bt_navigator.ros__parameters.global_frame': substitute_namespace(namespace, "map"),
        # 'topic': substitute_namespace(namespace, "scan")
    }
    slam_param_substitutions = {
        'base_frame': substitute_namespace(namespace, "base_link"),
        'odom_frame': substitute_namespace(namespace, "odom"),
        'map_frame': substitute_namespace(namespace, "map")
    }
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=nav_param_substitutions,
        convert_types=True)
    
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key=namespace,
        param_rewrites=slam_param_substitutions,
        convert_types=True)
    
    slam = GroupAction(
        condition=IfCondition(use_slam),
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(pathlib.Path(slam_toolbox_dir).joinpath('launch', 'online_async_launch.py'))]),
                launch_arguments = {
                    'slam_params_file': configured_slam_params,
                }.items()
            ),
        ]
    )
    
    local_costmap_scan_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'scan')},
            {'output_topic': substitute_namespace(namespace, 'local_costmap/scan')}
        ]
    )

    global_costmap_scan_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'scan')},
            {'output_topic': substitute_namespace(namespace, 'global_costmap/scan')}
        ]
    )

    pc_relay = Node(
        name="pc_relay",
        package="topic_tools",
        executable="relay",
        parameters=[
            {'input_topic': substitute_namespace(namespace, 'pc')},
            {'output_topic': '/pc'}
        ]
    )

    tf_relay = Node(
        name="tf_relay",
        package="topic_tools",
        executable="relay",
        parameters=[
            {'input_topic': '/tf'},
            {'output_topic': substitute_namespace(namespace, 'tf')}
        ]
    )

    tf_static_relay = Node(
        name="tf_static_relay",
        package="topic_tools",
        executable="relay",
        parameters=[
            {'input_topic': '/tf_static'},
            {'output_topic': substitute_namespace(namespace, 'tf_static')}
        ]
    )

    tf_odom_container = ComposableNodeContainer(
        condition=UnlessCondition(use_slam),
        name='image_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                parameters=[
                    {'frame_id': substitute_namespace(namespace, 'map')},
                    {'child_frame_id': substitute_namespace(namespace, 'odom')}
                ]
            )
        ],
        output='both',
    )

    tf_map_container = ComposableNodeContainer(
        name='image_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                parameters=[
                    {'frame_id': 'map'},
                    {'child_frame_id': substitute_namespace(namespace, 'map')}
                ]
            )
        ],
        output='both',
    )

    nav_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(pathlib.Path(nav2_dir).joinpath('launch', 'navigation_launch.py'))]),
                launch_arguments = {
                    'params_file': configured_nav2_params,
                    'namespace': namespace,
                    'use_sim_time': 'True',
                    'autostart': 'True'
                }.items()
            )
        ]
    )
    
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        namespace=namespace,
        parameters=[{
            'min_height': 0.1,
            'max_height': 5.0,
            'range_min': 0.2,
            'range_max': 60.0
        }],
        remappings=[
            ('cloud_in', 'pc'),
        ]
    )
    
    twist_to_ackermann = Node(
        package='o3de_kraken_nav',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        namespace=namespace,
        parameters=[{
            'wheelbase': 2.2,
            'timeout_control_interval': 0.1,
            'control_timeout': 0.2,
            'publish_zeros_on_timeout': True
        }],
        remappings=[
            ('/ackermann_vel', substitute_namespace(namespace, 'ackermann_vel')),
        ]
    )
    
    rviz = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='slam',
                arguments=[
                    '-d', str(pathlib.Path(package_dir).joinpath('launch', 'config', 'config_multi.rviz')),
                ]
            )
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_slam_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(tf_odom_container)
    ld.add_action(tf_map_container)
    ld.add_action(tf_relay)
    ld.add_action(local_costmap_scan_relay)
    ld.add_action(global_costmap_scan_relay)
    ld.add_action(tf_static_relay)
    ld.add_action(pc_relay)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(twist_to_ackermann)
    ld.add_action(slam)
    ld.add_action(nav_nodes)
    ld.add_action(rviz)
    
    return ld
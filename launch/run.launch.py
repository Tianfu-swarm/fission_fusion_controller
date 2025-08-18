import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_nodes(context, *args, **kwargs):
    number_of_robots = int(float(LaunchConfiguration('numbers').perform(context)))

    nodes = []
    for i in range(number_of_robots):
        nodes.append(
            Node(
                package='fission_fusion',
                executable='fission_fusion',
                name='fission_fusion_controller',
                namespace=f'bot{i}',
                output='screen',
                parameters=[{
                    "use_sim_time": LaunchConfiguration('use_sim_time'),
                    "results_file_path": LaunchConfiguration('results_file_path'),
                    "isMinCommunication": LaunchConfiguration('isMinCommunication'),
                    "isConCommunication": LaunchConfiguration('isConCommunication'),
                    "isModelworks": LaunchConfiguration('isModelworks'),
                    "desired_subgroup_size": LaunchConfiguration('desired_subgroup_size'),
                    "subgroup_size_sigma": LaunchConfiguration('subgroup_size_sigma'),
                    "groupsize_tolerance": LaunchConfiguration('groupsize_tolerance'),
                    "K": LaunchConfiguration('K'),
                    "early_converge_window": LaunchConfiguration('early_converge_window'),
                    "n_groupsize": LaunchConfiguration('numbers'),
                    "follow_range": LaunchConfiguration('follow_range'),
                    "controller_type": "sffm", 
                }],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ]
            )
        )
    return nodes


def generate_launch_description():
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('results_file_path', default_value='../data/output.csv'),
        DeclareLaunchArgument('isMinCommunication', default_value='true'),
        DeclareLaunchArgument('isConCommunication', default_value='true'),
        DeclareLaunchArgument('isModelworks', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('desired_subgroup_size', default_value='14'),
        DeclareLaunchArgument('subgroup_size_sigma', default_value='1'),
        DeclareLaunchArgument('groupsize_tolerance', default_value='0'),
        DeclareLaunchArgument('K', default_value='1000'),
        DeclareLaunchArgument('early_converge_window', default_value='30'),
        DeclareLaunchArgument('numbers', default_value='20.0'),
        DeclareLaunchArgument('follow_range', default_value='2.0'),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', "/rviz/defaul.rviz"],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription(declare_args + [set_qt_platform, OpaqueFunction(function=generate_nodes), rviz_node])

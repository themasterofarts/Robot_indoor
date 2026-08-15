from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Lancer RViz'),

        # Lancer Nav2 avec use_sim_time: True
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'True',
            }.items()
        ),

        # Visualiser la carte et les données de navigation dans RViz
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("indoor_navigation"), "rviz", "bot.rviz"]  # Corrigé le nom de package
                ),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),

        # Lancer mapping.launch.py de indoor_navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('indoor_navigation'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'True',
            }.items()
        ),

        # Nœud frontier_explorer existant
        Node(
            package="indoor_navigation",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            output="screen",
            parameters=[
                {"map_topic": "/map"},
                {"min_cluster_size": 10},
                {"publish_period_s": 1.0},
                {"use_sim_time": True},

                {"global_frame": "map"},
                {"base_frame": "base_link"},
                {"nav_action_name": "/navigate_to_pose"},

                {"min_goal_separation_m": 0.8},
                {"goal_cooldown_s": 3.0},

                {"score_distance_weight": 1.0},
                {"score_size_weight": 0.05},

                {"goal_backoff_m": 0.6},
                {"goal_search_radius_m": 1.5},
                {"goal_clearance_cells": 2},
                {"min_cluster_size": 15},

                {"save_map_on_completion": True},
                #{"map_save_path": PathJoinSubstitution([
                #    FindPackageShare("indoor_navigation"),
                #    "maps",
                #    "explored_map.pgm"
                #])},
            ],
        ),
    ])
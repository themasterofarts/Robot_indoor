import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = "indoor_navigation"

    static_map_path = os.path.join(
        get_package_share_directory(package_name), "config", "map.yaml"
    )
    nav2_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "nav2_params.yaml"
    )

    bringup_dir = get_package_share_directory("nav2_bringup")

    launch_dir = os.path.join(bringup_dir, "launch")

    nav2_bt_path = FindPackageShare(package="nav2_bt_navigator").find(
        "nav2_bt_navigator"
    )
    behavior_tree_xml_path = os.path.join(
        nav2_params_path, "behavior_trees", "follow_me.xml"
    )  # navigate_w_replanning_and_recovery #pas utiliser

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    base_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_map",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "map"],
    )

    # Launch them all!
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            launch.actions.DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    bringup_dir, "rviz", "nav2_default_view.rviz"
                ),
                description="Full path to the RVIZ config file to use",
            ),
            launch.actions.DeclareLaunchArgument(
                name="map",
                default_value=static_map_path,
                description="Full path to map file to load",
            ),
            launch.actions.DeclareLaunchArgument(
                name="params_file",
                default_value=nav2_params_path,
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            launch.actions.DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            # launch.actions.DeclareLaunchArgument(
            #     name="default_bt_xml_filename",
            #     default_value=behavior_tree_xml_path,
            #     description="Full path to the behavior tree xml file to use",
            # ),
            # Launch the ROS 2 Navigation Stack
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    # "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    # "default_bt_xml_filename": LaunchConfiguration(
                    #     "default_bt_xml_filename"
                    # ),
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_config_file,
                }.items(),
            ),
            base_to_map,
        ]
    )

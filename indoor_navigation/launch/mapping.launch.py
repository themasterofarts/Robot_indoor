from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    pkg_path = get_package_share_directory("indoor_navigation")
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(pkg_path, "config", "slam_toolbox_params.yaml"),
    )

    start_async_slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": True}],
    )

    # Configure transition
    activate_service = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"], output="screen"
    )

    activate_service2 = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"], output="screen"
    )

    return LaunchDescription(
        [
            start_async_slam_toolbox_node,
            TimerAction(period=2.0, actions=[activate_service]),
            TimerAction(period=4.0, actions=[activate_service2]),
        ]
    )

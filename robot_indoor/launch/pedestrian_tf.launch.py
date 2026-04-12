from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pedestrian_tf',
            executable='pedestrian_tf_broadcaster',
            name='pedestrian_tf_broadcaster',
            output='screen',
            parameters=[{
                'input_topic': '/actor/pose',
                'parent_frame': 'map',
                'child_frame': 'pedestrian',
                'use_sim_time': True,
            }]
        ),
        
        # 🔹 TF statique map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        )
    ])
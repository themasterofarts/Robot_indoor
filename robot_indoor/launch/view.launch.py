import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution , Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    

    
    pkg_project_gazebo = get_package_share_directory('robot_indoor')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
   
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_indoor'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
   
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([ "-r" ,
            pkg_project_gazebo,
            'worlds',
            'empty_gz.world'
        ]
        )}.items(),
    )

    # Create nodes for robot state publisher, joint state publisher, and controllers
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params],
        remappings=remappings,
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params]

    )
  

     # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'bot',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            
        ])

    # Visualize in RViz
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=[
        '-d', 
        PathJoinSubstitution([
            FindPackageShare('robot_indoor'),
            'rviz',
            'bot.rviz'
        ])
    ],
    condition=IfCondition(LaunchConfiguration('rviz'))
)

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_gazebo, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )


    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image_raw",
            "depth_camera/image", 
            "depth_camera/depth_image"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use sim time if true'),
        
        bridge,
        ros_gz_image_bridge,
        robot_state_publisher,
        joint_state_publisher_node,
        start_gazebo_ros_spawner_cmd,
        rviz
    ])
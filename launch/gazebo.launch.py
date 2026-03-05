import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc  = get_package_share_directory('my_robot_description')
    pkg_gz    = get_package_share_directory('my_robot_gazebo')

    urdf_path  = os.path.join(pkg_desc, 'urdf', 'my_robot_description.urdf')
    world_path = os.path.join(pkg_desc, 'worlds', 'airport_terminal.sdf') #zwischen world.sdf und airport terminal switchbar
    bridge_yaml = os.path.join(pkg_desc, 'config', 'bridge.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. Gazebo Ionic starten
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': f'{world_path} -r --verbose 1'}.items()
    )

    # 2. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 3. Roboter in Gazebo spawnen
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', '/robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # 4. Bridge mit YAML-Konfiguration
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'config_file': bridge_yaml,
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[bridge]),
    ])
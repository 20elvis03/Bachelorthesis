import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    
    # URDF-Pfad
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot_description.urdf')
    
    # Prüfen ob Datei existiert
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    # URDF einlesen
    with open(urdf_path, 'r') as file:
        robot_desc = file.read()
    
    # RVIZ-Konfiguration (optional)
    rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    
    return LaunchDescription([
        # ROBOT STATE PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # JOINT STATE PUBLISHER (GUI Version)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RVIZ2 mit Konfiguration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            output='screen'
        )
    ])
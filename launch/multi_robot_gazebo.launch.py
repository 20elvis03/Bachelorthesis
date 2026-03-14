#!/usr/bin/env python3
"""
Multi-Robot Launch – 3 Roboter mit autonomem Fahren
====================================================
Startet:
  1. Gazebo Simulation
  2. 3 Roboter (robot_1, robot_2, robot_3) mit jeweils:
     - Robot State Publisher
     - Gazebo Spawn
     - Bridge (Gazebo ↔ ROS2)
     - Autonomer Fahrmodus
  3. Emergency Controller (optional)
  4. RViz2 (optional)

Starten:
  ros2 launch my_robot_gazebo gazebo_multi_robot_launch.py

Emergency Stop:
  Einzeln:  ros2 topic pub /robot_1/emergency_stop std_msgs/msg/Bool "data: true" --once
  Alle:     ros2 topic pub /emergency_stop_all      std_msgs/msg/Bool "data: true" --once
  Freigabe: ... "data: false" --once
"""

import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, GroupAction,
                            TimerAction, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════════════
#  KONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# Roboter mit Startpositionen und Arbeitsbereichen
ROBOTS = [
    {
        "name": "robot_1",
        "x": 22.5, "y": -22.5, "z": 0.25, "yaw": 1.5708,
        "priority": 1,
        # Arbeitsbereich (linkes Drittel)
        "area_x_min": -20.0, "area_x_max": -2.0,
        "area_y_min": -20.0, "area_y_max": 14.0,
    },
    {
        "name": "robot_2",
        "x": 17.5, "y": -22.5, "z": 0.25, "yaw": 1.5708,
        "priority": 2,
        # Arbeitsbereich (Mitte)
        "area_x_min": -2.0, "area_x_max": 10.0,
        "area_y_min": -20.0, "area_y_max": 14.0,
    },
    {
        "name": "robot_3",
        "x": 12.5, "y": -22.5, "z": 0.25, "yaw": 1.5708,
        "priority": 3,
        # Arbeitsbereich (rechtes Drittel)
        "area_x_min": 10.0, "area_x_max": 22.0,
        "area_y_min": -20.0, "area_y_max": 14.0,
    },
]

# Bridge-Topics pro Roboter
BRIDGE_TOPICS_PER_ROBOT = [
    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    "/steering@std_msgs/msg/Float64@gz.msgs.Double",
    "/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/camera/front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
]


# ═══════════════════════════════════════════════════════════════════
#  ROBOTER-GRUPPE ERZEUGEN
# ═══════════════════════════════════════════════════════════════════

def make_robot_group(robot: dict, urdf_content: str,
                     all_robots: list) -> GroupAction:
    """Erzeugt alle Nodes für einen Roboter unter seinem Namespace."""
    ns = robot["name"]

    # ── Robot State Publisher ─────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": urdf_content,
            "use_sim_time": True,
            "frame_prefix": ns + "/",
        }],
        remappings=[
            ("/tf", f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
        ]
    )

    # ── Spawn in Gazebo ───────────────────────────────────────────
    spawn_args = [
        "-name",  ns,
        "-topic", f"/{ns}/robot_description",
        "-x",     str(robot["x"]),
        "-y",     str(robot["y"]),
        "-z",     str(robot["z"]),
    ]
    if "yaw" in robot:
        spawn_args += ["-Y", str(robot["yaw"])]

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{ns}",
        output="screen",
        arguments=spawn_args,
    )

    # ── Bridge: Gazebo ↔ ROS2 ────────────────────────────────────
    bridge_args = []
    for topic in BRIDGE_TOPICS_PER_ROBOT:
        topic_name = topic.split("@")[0]
        rest = "@".join(topic.split("@")[1:])
        bridge_args.append(f"/{ns}{topic_name}@{rest}")

    # Clock (global, jeder Roboter braucht use_sim_time)
    bridge_args.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{ns}",
        output="screen",
        arguments=bridge_args,
        parameters=[{"use_sim_time": True}],
    )

    # ── Peer-Namespaces (alle anderen Roboter) ───────────────────
    peers = [r["name"] for r in all_robots if r["name"] != ns]
    peer_str = ",".join(peers)

    # ── Autonomer Fahrmodus ───────────────────────────────────────
    auto_node = Node(
        package="my_robot_gazebo",
        executable="autonomous",
        name="autonomous_drive",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "spawn_x":          float(robot["x"]),
            "spawn_y":          float(robot["y"]),
            "area_x_min":       float(robot.get("area_x_min", -20.0)),
            "area_x_max":       float(robot.get("area_x_max", 20.0)),
            "area_y_min":       float(robot.get("area_y_min", -20.0)),
            "area_y_max":       float(robot.get("area_y_max", 14.0)),
            "lane_width":       3.0,
            "peer_namespaces":  peer_str,
            "robot_priority":   robot.get("priority", 99),
        }],
    )

    return GroupAction(actions=[
        PushRosNamespace(ns),
        rsp,
        spawn,
        bridge,
        # Autonomie mit Verzögerung (damit Bridge steht)
        TimerAction(period=5.0, actions=[auto_node]),
    ])


# ═══════════════════════════════════════════════════════════════════
#  LAUNCH DESCRIPTION
# ═══════════════════════════════════════════════════════════════════

def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot_description")

    # ── URDF laden ────────────────────────────────────────────────
    urdf_path = os.path.join(pkg_share, "urdf", "my_robot_description.urdf")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF nicht gefunden: {urdf_path}")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    # ── 1. Gazebo starten ────────────────────────────────────────
    world_file = os.path.join(pkg_share, "worlds", "airport_terminal_world.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": f"{world_file} -r --verbose 1"
        }.items()
    )

    # ── 2. Roboter-Gruppen (zeitversetzt) ─────────────────────────
    robot_groups = []
    for i, robot in enumerate(ROBOTS):
        delayed = TimerAction(
            period=float(8 + i * 4),  # robot_1 nach 8s, robot_2 nach 12s, ...
            actions=[make_robot_group(robot, robot_desc, ROBOTS)]
        )
        robot_groups.append(delayed)

    # ── 3. Emergency Bridge (global Topic) ────────────────────────
    # Ermöglicht /emergency_stop_all ohne extra Gazebo-Bridge
    # (rein ROS2-seitiges Topic, braucht keine Bridge)

    # ── 4. RViz2 (optional) ──────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "config", "display.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gz_sim,
        *robot_groups,
        TimerAction(period=25.0, actions=[rviz]),
    ])

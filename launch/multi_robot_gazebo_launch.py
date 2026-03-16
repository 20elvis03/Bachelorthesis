#!/usr/bin/env python3
"""
Multi-Robot Launch – 3 Roboter mit autonomem Fahren (FIXED)
============================================================
Startet:
  1. Gazebo Simulation (mit Wartezeit für Erststart)
  2. 3 Roboter (robot_1, robot_2, robot_3) mit jeweils:
     - Robot State Publisher  (eigener Namespace)
     - Gazebo Spawn           (namespaced URDF-Topics)
     - Bridge                 (ALLE Topics: Kameras, LiDAR, Odom, …)
     - Autonomer Fahrmodus    (eigene Route + eigene Sensoren)
  3. Surveillance-Kamera-Bridge (global, 1×)
  4. Emergency Controller (optional)

Topics pro Roboter (Beispiel robot_1):
  /robot_1/cmd_vel               ← Steuerung
  /robot_1/odom                  → Odometrie
  /robot_1/scan                  → LiDAR LaserScan
  /robot_1/scan/points           → LiDAR PointCloud2
  /robot_1/tf                    → TF (odom→base_footprint)
  /robot_1/joint_states          → Gelenkzustände
  /robot_1/steering              ← Lenkung
  /robot_1/camera/front/image    → Frontkamera
  /robot_1/camera/left/image     → Links-Kamera
  /robot_1/camera/right/image    → Rechts-Kamera
  /robot_1/camera/back/image     → Rückkamera
  /robot_1/camera/*/depth_image  → jeweilige Tiefenbilder
  /robot_1/camera/*/camera_info  → jeweilige CameraInfo
  /robot_1/emergency_stop        ← Not-Stopp (einzeln)
  /emergency_stop_all            ← Not-Stopp (global)

Starten:
  ros2 launch my_robot_gazebo multi_robot_gazebo_launch.py

Emergency Stop:
  Einzeln:  ros2 topic pub /robot_1/emergency_stop std_msgs/msg/Bool "data: true" --once
  Alle:     ros2 topic pub /emergency_stop_all      std_msgs/msg/Bool "data: true" --once
  Freigabe: ... "data: false" --once
"""

import os
import re
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, GroupAction,
                            TimerAction, DeclareLaunchArgument,
                            ExecuteProcess, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════════════
#  KONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# Roboter mit verschiedenen Startpositionen und Richtungen (KEINE festen Bereiche!)
ROBOTS = [
    {
        "name": "robot_1",
        "x": 22.5, "y": -22.5, "z": 0.25, "yaw": 1.5708,   # Rechts unten, fährt nach +Y
        "priority": 1,
    },
    {
        "name": "robot_2",
        "x": -22.5, "y": -22.5, "z": 0.25, "yaw": 1.5708,  # Links unten, fährt nach +Y
        "priority": 2,
    },
    {
        "name": "robot_3",
        "x": 0.0, "y": 7.5, "z": 0.25, "yaw": -1.5708,    # Mitte oben, fährt nach -Y
        "priority": 3,
    },
]

# Zeitversätze (in Sekunden)
# Beim ERSTSTART braucht Gazebo länger (Shader-Kompilierung, Modell-Download).
# Bei Problemen: GZ_STARTUP_DELAY erhöhen.
GZ_STARTUP_DELAY  = 22.0   # Warten bis Gazebo läuft (gleich wie Single-Robot!)
ROBOT_SPAWN_GAP   = 8.0    # Abstand zwischen Roboter-Spawns
AUTONOMY_DELAY    = 10.0   # Nach Spawn: warten bis Bridge steht

# ═══════════════════════════════════════════════════════════════════
#  BRIDGE-TOPICS PRO ROBOTER (vollständig!)
# ═══════════════════════════════════════════════════════════════════
# Format: (topic_suffix, ros_type, gz_type, direction)
#   direction: "@" = bidirectional, "[" = GZ→ROS, "]" = ROS→GZ

ROBOT_BRIDGE_TOPICS = [
    # ── Antrieb & Odometrie ──────────────────────────────────────
    ("/cmd_vel",        "geometry_msgs/msg/Twist",      "gz.msgs.Twist",        "@"),
    ("/odom",           "nav_msgs/msg/Odometry",        "gz.msgs.Odometry",     "["),
    ("/steering",       "std_msgs/msg/Float64",         "gz.msgs.Double",       "@"),

    # ── TF & Joints ─────────────────────────────────────────────
    ("/tf",             "tf2_msgs/msg/TFMessage",       "gz.msgs.Pose_V",       "["),
    ("/joint_states",   "sensor_msgs/msg/JointState",   "gz.msgs.Model",        "["),

    # ── LiDAR ───────────────────────────────────────────────────
    ("/scan",           "sensor_msgs/msg/LaserScan",    "gz.msgs.LaserScan",    "["),
    ("/scan/points",    "sensor_msgs/msg/PointCloud2",  "gz.msgs.PointCloudPacked", "["),

    # ── Kamera: Front ───────────────────────────────────────────
    ("/camera/front/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/front/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/front/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),

    # ── Kamera: Front Floor ─────────────────────────────────────
    ("/camera/front/floor/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/front/floor/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/front/floor/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),

    # ── Kamera: Links ───────────────────────────────────────────
    ("/camera/left/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/left/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/left/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),

    # ── Kamera: Rechts ──────────────────────────────────────────
    ("/camera/right/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/right/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/right/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),

    # ── Kamera: Hinten ──────────────────────────────────────────
    ("/camera/back/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/back/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/back/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),

    # ── Kamera: Hinten Floor ────────────────────────────────────
    ("/camera/back/floor/image",       "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/back/floor/depth_image", "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/camera/back/floor/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
]

# Surveillance-Kameras (global, aus der World-SDF – NICHT pro Roboter)
SURVEILLANCE_BRIDGE_TOPICS = [
    ("/surveillance/nw",                 "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/nw/camera_info",     "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/ne",                 "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/ne/camera_info",     "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/sw",                 "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/sw/camera_info",     "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/se",                 "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/se/camera_info",     "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/center/down",              "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/center/down/camera_info",  "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/center/north",             "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/center/north/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/center/south",             "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/center/south/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/center/east",              "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/center/east/camera_info",  "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/center/west",              "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/center/west/camera_info",  "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
    ("/surveillance/garage",                   "sensor_msgs/msg/Image",      "gz.msgs.Image",      "["),
    ("/surveillance/garage/camera_info",       "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "["),
]


# ═══════════════════════════════════════════════════════════════════
#  URDF NAMESPACING
# ═══════════════════════════════════════════════════════════════════

def namespace_urdf(urdf_content: str, ns: str) -> str:
    """Ersetzt alle absoluten Topics und Frame-IDs in der URDF
    mit namespace-prefixed Versionen für Multi-Robot.

    Beispiel (ns="robot_1"):
      <topic>/cmd_vel</topic>  →  <topic>/robot_1/cmd_vel</topic>
      <frame_id>odom</frame_id>  →  <frame_id>robot_1/odom</frame_id>
    """
    result = urdf_content

    # ── Plugin-Topics mit führendem / ersetzen ────────────────────
    # Matcht: <topic>/xxx</topic>, <odom_topic>/xxx</odom_topic>, <tf_topic>/xxx</tf_topic>
    # Ersetzt / am Anfang durch /{ns}/
    tag_names = ['topic', 'odom_topic', 'tf_topic']
    for tag in tag_names:
        pattern = rf'(<{tag}>)/(.+?)(</{tag}>)'
        replacement = rf'\g<1>/{ns}/\2\3'
        result = re.sub(pattern, replacement, result)

    # ── Frame-IDs namespacing ─────────────────────────────────────
    # DiffDrive publiziert TF mit diesen Frames → müssen pro Roboter einzigartig sein
    result = result.replace(
        '<frame_id>odom</frame_id>',
        f'<frame_id>{ns}/odom</frame_id>')
    result = result.replace(
        '<child_frame_id>base_footprint</child_frame_id>',
        f'<child_frame_id>{ns}/base_footprint</child_frame_id>')
    result = result.replace(
        '<frame_id>lidar</frame_id>',
        f'<frame_id>{ns}/lidar</frame_id>')

    return result


# ═══════════════════════════════════════════════════════════════════
#  BRIDGE-ARGUMENTE BAUEN
# ═══════════════════════════════════════════════════════════════════

def build_bridge_args(ns: str) -> list:
    """Erstellt Bridge-Argumente für einen Roboter.

    Da wir die URDF-Topics mit /{ns}/ prefixen, sind Gazebo-Topic
    und ROS2-Topic identisch → einfache @-Syntax reicht.
    """
    args = []
    for (topic_suffix, ros_type, gz_type, direction) in ROBOT_BRIDGE_TOPICS:
        full_topic = f"/{ns}{topic_suffix}"
        args.append(f"{full_topic}@{ros_type}{direction}{gz_type}")

    # Clock (global, jeder Roboter braucht use_sim_time)
    args.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")

    return args


def build_surveillance_bridge_args() -> list:
    """Bridge-Argumente für die Surveillance-Kameras (global, 1×)."""
    args = []
    for (topic, ros_type, gz_type, direction) in SURVEILLANCE_BRIDGE_TOPICS:
        args.append(f"{topic}@{ros_type}{direction}{gz_type}")
    args.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
    return args


# ═══════════════════════════════════════════════════════════════════
#  ROBOTER-GRUPPE ERZEUGEN
# ═══════════════════════════════════════════════════════════════════

def make_robot_group(robot: dict, urdf_content: str,
                     all_robots: list) -> GroupAction:
    """Erzeugt alle Nodes für einen Roboter unter seinem Namespace."""
    ns = robot["name"]

    # ── URDF mit Namespace-Topics versehen ────────────────────────
    robot_urdf = namespace_urdf(urdf_content, ns)

    # ── Robot State Publisher ─────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_urdf,
            "use_sim_time": True,
            "frame_prefix": ns + "/",
        }],
        remappings=[
            ("/tf",        f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
        ]
    )

    # ── Spawn in Gazebo ───────────────────────────────────────────
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{ns}",
        output="screen",
        arguments=[
            "-name",  ns,
            "-topic", f"/{ns}/robot_description",
            "-x",     str(robot["x"]),
            "-y",     str(robot["y"]),
            "-z",     str(robot["z"]),
            "-Y",     str(robot.get("yaw", 0.0)),
        ],
    )

    # ── Bridge: Gazebo ↔ ROS2 (ALLE Topics) ──────────────────────
    bridge_args = build_bridge_args(ns)
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
    # FIX: package ist "my_robot_description" (dort ist autonomous installiert!)
    auto_node = Node(
        package="my_robot_description",
        executable="autonomous",
        name="autonomous_drive",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "spawn_x":          float(robot["x"]),
            "spawn_y":          float(robot["y"]),
            "spawn_yaw":        float(robot.get("yaw", 1.5708)),
            "peer_namespaces":  peer_str,
            "robot_priority":   robot.get("priority", 99),
        }],
    )

    return GroupAction(actions=[
        PushRosNamespace(ns),
        rsp,
        # Spawn nach 3s (RSP muss erst robot_description publishen)
        TimerAction(period=3.0, actions=[spawn]),
        # Bridge nach Spawn
        TimerAction(period=5.0, actions=[bridge]),
        # Autonomie nach Bridge
        TimerAction(period=AUTONOMY_DELAY, actions=[auto_node]),
    ])


# ═══════════════════════════════════════════════════════════════════
#  LAUNCH DESCRIPTION
# ═══════════════════════════════════════════════════════════════════

def generate_launch_description():
    pkg_desc = get_package_share_directory("my_robot_description")

    # ── URDF laden ────────────────────────────────────────────────
    urdf_path = os.path.join(pkg_desc, "urdf", "my_robot_description.urdf")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF nicht gefunden: {urdf_path}")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    # ── 1. Gazebo starten ─────────────────────────────────────────
    world_file = os.path.join(pkg_desc, "worlds", "airport_terminal_world.sdf")
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
        delay = GZ_STARTUP_DELAY + i * ROBOT_SPAWN_GAP
        delayed = TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f"═══ Spawning {robot['name']} at "
                            f"({robot['x']}, {robot['y']}) ═══"),
                make_robot_group(robot, robot_desc, ROBOTS),
            ]
        )
        robot_groups.append(delayed)

    # ── 3. Surveillance-Kameras Bridge (global, 1×) ───────────────
    surv_bridge_args = build_surveillance_bridge_args()
    surv_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_surveillance",
        output="screen",
        arguments=surv_bridge_args,
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        LogInfo(msg="══════════════════════════════════════════════════"),
        LogInfo(msg="  Multi-Robot Launch – 3 Roboter"),
        LogInfo(msg=f"  Gazebo-Wartezeit: {GZ_STARTUP_DELAY}s"),
        LogInfo(msg="══════════════════════════════════════════════════"),

        gz_sim,
        *robot_groups,
        TimerAction(period=GZ_STARTUP_DELAY + 5.0, actions=[surv_bridge]),
    ])

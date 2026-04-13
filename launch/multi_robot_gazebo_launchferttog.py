import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

# ── Robot configurations ─────────────────────────────────────────────────────
# Each robot gets a unique namespace, spawn position, and dock position.
# Dock positions must match the charging pads in your world.
ROBOTS = [
    {"name": "robot_1", "x": 22.5, "y": -28.5, "z": 0.25, "yaw": 1.5708,
     "spawn_gx": 22.5, "spawn_gy": -28.5, "spawn_yaw_deg": 90.0},
    {"name": "robot_2", "x": 18.5, "y": -28.5, "z": 0.25, "yaw": 1.5708,
     "spawn_gx": 18.5, "spawn_gy": -28.5, "spawn_yaw_deg": 90.0},
    {"name": "robot_3", "x": 14.5, "y": -28.5, "z": 0.25, "yaw": 1.5708,
     "spawn_gx": 14.5, "spawn_gy": -28.5, "spawn_yaw_deg": 90.0},
]

GZ_TOPICS_TO_REMAP = [
    "/cmd_vel",
    "/odom",
    "/tf",
    "/steering",
    "/joint_states",
    "/scan",
    "/camera/front",
    "/camera/front/floor",
    "/camera/left",
    "/camera/right",
    "/camera/back",
    "/camera/back/floor",
]

BRIDGE_TOPICS = [
    ("cmd_vel",        "geometry_msgs/msg/Twist",        "gz.msgs.Twist",              ">"),
    ("steering",       "std_msgs/msg/Float64",           "gz.msgs.Double",             ">"),
    ("odom",           "nav_msgs/msg/Odometry",          "gz.msgs.Odometry",           "<"),
    ("scan/points",    "sensor_msgs/msg/PointCloud2",    "gz.msgs.PointCloudPacked",   "<"),
    ("scan",           "sensor_msgs/msg/LaserScan",      "gz.msgs.LaserScan",          "<"),
    ("tf",             "tf2_msgs/msg/TFMessage",         "gz.msgs.Pose_V",             "<"),
    ("joint_states",   "sensor_msgs/msg/JointState",     "gz.msgs.Model",              "<"),
]


def _namespace_urdf(urdf: str, ns: str) -> str:
    """Replace absolute Gazebo topic names in URDF with namespaced versions."""
    result = urdf
    for topic in GZ_TOPICS_TO_REMAP:
        result = result.replace(
            f"<topic>{topic}</topic>",
            f"<topic>/{ns}{topic}</topic>")
        result = result.replace(
            f"<odom_topic>{topic}</odom_topic>",
            f"<odom_topic>/{ns}{topic}</odom_topic>")
        result = result.replace(
            f"<tf_topic>{topic}</tf_topic>",
            f"<tf_topic>/{ns}{topic}</tf_topic>")
    return result


def make_robot_group(robot: dict, urdf_template: str) -> GroupAction:
    ns = robot["name"]
    urdf = _namespace_urdf(urdf_template, ns)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": urdf,
            "use_sim_time": True,
            "frame_prefix": ns + "/",
        }],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{ns}",
        output="screen",
        arguments=[
            "-name",  ns,
            "-topic", f"/{ns}/robot_description",
            "-x", str(robot["x"]),
            "-y", str(robot["y"]),
            "-z", str(robot["z"]),
            "-Y", str(robot["yaw"]),
        ],
    )

    bridge_args = []
    for ros_suffix, ros_type, gz_type, direction in BRIDGE_TOPICS:
        gz_topic = f"/{ns}/{ros_suffix}"
        ros_topic = f"/{ns}/{ros_suffix}"
        if direction == ">":
            bridge_args.append(f"{ros_topic}@{ros_type}@{gz_type}")
        else:
            bridge_args.append(f"{ros_topic}@{ros_type}[{gz_type}")
    bridge_args.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
    bridge_args.append(
        f"/{ns}/world/pose_info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{ns}",
        output="screen",
        arguments=bridge_args,
        parameters=[{"use_sim_time": True}],
    )

    auto_drive = Node(
        package="my_robot_description",
        executable="autonomousbugfertig",
        name="auto_drive",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "spawn_gx": robot["spawn_gx"],
            "spawn_gy": robot["spawn_gy"],
            "spawn_yaw_deg": robot["spawn_yaw_deg"],
        }],
    )

    return GroupAction(actions=[
        PushRosNamespace(ns),
        rsp,
        spawn,
        bridge,
        auto_drive,
    ])


def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot_description")
    urdf_path = os.path.join(pkg_share, "urdf", "my_robot_description.urdf")

    with open(urdf_path, "r") as f:
        urdf_template = f.read()

    world_file = os.path.join(pkg_share, "worlds", "airport_terminal_world.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": f"{world_file} -r --verbose 1"}.items(),
    )

    robot_groups = []
    for i, robot in enumerate(ROBOTS):
        delayed = TimerAction(
            period=float(5 + i * 3),
            actions=[make_robot_group(robot, urdf_template)],
        )
        robot_groups.append(delayed)

    return LaunchDescription([gz_sim, *robot_groups])
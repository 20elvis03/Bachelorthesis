import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

ROBOTS = [
    {"name": "robot_1", "x":  0.0, "y":  0.0, "z": 0.5},
    {"name": "robot_2", "x":  5.0, "y":  0.0, "z": 0.5},
    {"name": "robot_3", "x": -5.0, "y":  0.0, "z": 0.5},
]

BRIDGE_TOPICS_PER_ROBOT = [
    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
]


def make_robot_group(robot: dict, urdf_content: str) -> GroupAction:
    ns = robot["name"]

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
        ]
    )

    bridge_args = []
    for topic in BRIDGE_TOPICS_PER_ROBOT:
        topic_name = topic.split("@")[0]
        rest = "@".join(topic.split("@")[1:])
        bridge_args.append(f"/{ns}{topic_name}@{rest}")

    bridge_args.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{ns}",
        output="screen",
        arguments=bridge_args,
        parameters=[{"use_sim_time": True}]
    )

    return GroupAction(actions=[
        PushRosNamespace(ns),
        rsp,
        spawn,
        bridge,
    ])


def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot_description")
    urdf_path = os.path.join(pkg_share, "urdf", "my_robot_description.urdf")

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF nicht gefunden: {urdf_path}")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

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

    robot_groups = []
    for i, robot in enumerate(ROBOTS):
        delayed = TimerAction(
            period=float(3 + i * 2),
            actions=[make_robot_group(robot, robot_desc)]
        )
        robot_groups.append(delayed)

    rviz_config = os.path.join(pkg_share, "config", "display.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gz_sim,
        *robot_groups,
        TimerAction(period=10.0, actions=[rviz]),
    ])
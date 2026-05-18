import os, re, tempfile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

# ── Robot configurations ─────────────────────────────────────────────────────
#robots will return to the global spawn_gx/y coordinate and start from there so they all drive a different pattern
#even if the normal spawn coordinates x/y are different 
ROBOTS = [
    {"name": "robot_1", "x": 22.5, "y": -22.5, "z": 0.15, "yaw": 1.5708,
     "spawn_gx": 21.0, "spawn_gy": -29.1, "spawn_yaw_deg": 90.0},
    {"name": "robot_2", "x": 2.5, "y": -22.5, "z": 0.15, "yaw": 1.5708,
     "spawn_gx": 15.0, "spawn_gy": -29.1, "spawn_yaw_deg": 90.0},
    {"name": "robot_3", "x": -22.5, "y": -22.5, "z": 0.15, "yaw": 1.5708,
     "spawn_gx": 9.0, "spawn_gy": -29.1, "spawn_yaw_deg": 90.0},
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

    result = re.sub(
        r"<frame_id>(\w+)</frame_id>",
        rf"<frame_id>{ns}/\1</frame_id>",
        result)
    result = re.sub(
        r"<child_frame_id>(\w+)</child_frame_id>",
        rf"<child_frame_id>{ns}/\1</child_frame_id>",
        result)

    return result


def _make_robot_bridge_yaml(template: str, ns: str) -> str:
    """Write a namespaced copy of the per-robot bridge YAML to a temp file."""
    content = template.replace("{ns}", ns)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix=f"bridge_{ns}_", delete=False)
    tmp.write(content)
    tmp.close()
    return tmp.name


def make_robot_group(robot: dict, urdf_template: str,
                     bridge_template: str) -> GroupAction:
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

    # ── Bridge via YAML config (lazy works reliably with config_file) ────
    bridge_yaml = _make_robot_bridge_yaml(bridge_template, ns)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{ns}",
        output="screen",
        parameters=[{
            "config_file": bridge_yaml,
            "use_sim_time": True,
            "lazy": True,
        }],
    )

    auto_drive = Node(
        package="my_robot_description",
        executable="autonomousbug.py",
        name="auto_drive",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_name": ns,
            "spawn_gx": robot["spawn_gx"],
            "spawn_gy": robot["spawn_gy"],
            "spawn_yaw_deg": robot["spawn_yaw_deg"],
            "init_x": robot["x"],
            "init_y": robot["y"],
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

    bridge_per_robot_path = os.path.join(
        pkg_share, "config", "bridge_per_robot_without_camera.yaml") # change to birdge_per_robot for robots with cameras and bridge_per_robot_without_camera for robots without cameras (better performance)
    with open(bridge_per_robot_path, "r") as f:
        bridge_template = f.read()

    world_file = os.path.join(pkg_share, "worlds", "airport_terminal_world.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": f"{world_file} -r --verbose 1"}.items(),
    )

    # ── Timing: let Gazebo load the world before spawning anything ──
    GZ_STARTUP_DELAY = 15.0

    shared_bridge_yaml = os.path.join(
        pkg_share, "config", "bridge_multi_shared_without_camera.yaml") # change to bridge_multi_shared for surveillance cameras and bridge_multi_shared_without_camera for no surveillance cameras (better performance)
    shared_bridge = TimerAction(
        period=GZ_STARTUP_DELAY - 2.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="shared_bridge",
            output="screen",
            parameters=[{
                "config_file": shared_bridge_yaml,
                "use_sim_time": True,
                "lazy": True,
            }],
        )],
    )

    robot_groups = []
    for i, robot in enumerate(ROBOTS):
        delayed = TimerAction(
            period=GZ_STARTUP_DELAY + i * 5,
            actions=[make_robot_group(robot, urdf_template, bridge_template)],
        )
        robot_groups.append(delayed)

    return LaunchDescription([gz_sim, shared_bridge, *robot_groups])

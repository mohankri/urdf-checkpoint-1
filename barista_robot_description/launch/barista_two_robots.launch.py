import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro


PACKAGE_NAME = "barista_robot_description"

# name, chassis color (must exist as a <material> in the xacro), spawn pose
ROBOTS = [
	{"name": "rick",  "color": "red",  "x": 0.0, "y": 0.0, "yaw": 0.0},
	{"name": "morty", "color": "blue", "x": 0.0, "y": 1.5, "yaw": 0.0},
]


def robot_actions(pkg_share, robot, include_laser_value):
    """All per-robot actions: RSP, spawn, bridge, controllers, static TF."""
    name = robot["name"]

    xacro_file = os.path.join(pkg_share, "urdf",
                              "barista_robot_model.urdf.xacro")
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "include_laser": include_laser_value,
            "robot_name": name,
            "robot_color": robot["color"],
            "use_ros2_control": "false",
        },
    )
    params = {
        "robot_description": ParameterValue(doc.toxml(), value_type=str),
        "frame_prefix": name + "/",
        "use_sim_time": True,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        emulate_tty=True,
        parameters=[params],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"{name}_spawn",
        arguments=[
            "-name", name,
            "-allow_renaming", "true",
            "-topic", f"/{name}/robot_description",
            "-x", str(robot["x"]),
            "-y", str(robot["y"]),
            "-z", "0.2",
            "-Y", str(robot["yaw"]),
        ],
        output="screen",
    )

    # Per-robot ROS <-> Gazebo bridge. The plugin topics in the xacro are
    # already namespaced with robot_name, so bridge names match 1:1.
    bridge_args = [
        f"/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        f"/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        f"/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    if include_laser_value.lower() in ("true", "1"):
        bridge_args.append(
            f"/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{name}_gz_bridge",
        arguments=bridge_args,
        # Frames inside the messages are already prefixed (rick/odom,
        # rick/base_link) so both robots can safely merge onto global /tf.
        remappings=[(f"/{name}/tf", "/tf")],
        output="screen",
    )

    # Root both TF trees at a common `world` frame so RViz can display
    # rick and morty simultaneously with fixed frame = world.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{name}_odom_static_tf",
        arguments=["--x", str(robot["x"]), "--y", str(robot["y"]),
                   "--z", "0", "--yaw", str(robot["yaw"]),
                   "--frame-id", "world",
                   "--child-frame-id", f"{name}/odom"],
        output="screen",
    )

    #spawn_controller = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["joint_state_broadcaster"],
    #    output="screen",
    #)

    actions = [robot_state_publisher, spawn_entity, gz_bridge, static_tf]
   # if include_laser_value.lower() in ("true", "1"):
   #     actions.append(delayed_spawners)
    return actions


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    include_laser_value = LaunchConfiguration("include_laser").perform(context)
    use_rviz_value = LaunchConfiguration("use_rviz").perform(context)

    # Gazebo resource paths
    install_dir_path = get_package_prefix(PACKAGE_NAME) + "/share"
    gazebo_models_path = os.path.join(pkg_share, "models")
    gazebo_resource_paths = [install_dir_path, pkg_share, gazebo_models_path]
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["GZ_SIM_RESOURCE_PATH"]:
                os.environ["GZ_SIM_RESOURCE_PATH"] += (":" + resource_path)
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)

    gz_sim_pkg = get_package_share_directory("ros_gz_sim")
    world_file = os.path.join(pkg_share, "worlds", "barista_empty.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    # /clock is global -- bridge it exactly once.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    actions = [SetParameter(name="use_sim_time", value=True),
               gz_sim, clock_bridge]

    for robot in ROBOTS:
        actions += robot_actions(pkg_share, robot, include_laser_value)

    if use_rviz_value.lower() in ("true", "1"):
        rviz_config_dir = os.path.join(pkg_share, "rviz", "two_robots.rviz")
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            name="rviz_node",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", rviz_config_dir],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "include_laser", default_value="true",
            description="Whether to include the laser scanner in the URDF"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Whether to start RViz"),
        OpaqueFunction(function=launch_setup),
    ])

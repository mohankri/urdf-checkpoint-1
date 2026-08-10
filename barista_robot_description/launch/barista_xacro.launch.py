import os
 
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import (Node, SetParameter)
from launch_ros.parameter_descriptions import ParameterValue
 
import xacro
 
 
def launch_setup(context, *args, **kwargs):
 
    package_description = "barista_robot_description"
    pkg_barista_gazebo = get_package_share_directory('barista_robot_description')
 
    # Resolve the include_laser launch argument NOW (inside OpaqueFunction,
    # where LaunchConfiguration can actually be evaluated), then pass it
    # into xacro as a mapping so xacro:if actually sees the right value.
    include_laser_value = LaunchConfiguration('include_laser').perform(context)
 
    xacro_file = os.path.join(pkg_barista_gazebo, 'urdf', 'barista_robot_model.urdf.xacro')
    doc = xacro.process_file(
        xacro_file,
        mappings={'include_laser': include_laser_value}
    )
    robot_description_xml = doc.toxml()
    params = {'robot_description': ParameterValue(robot_description_xml, value_type=str)}
 
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[params],
        output="screen"
    )
 
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')
 
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
 
    # Setup to launch the simulator and Gazebo world
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")
 
    install_dir_path = (get_package_prefix('barista_robot_description') + "/share")
    gazebo_models_path = os.path.join(pkg_barista_gazebo, "models")
    gazebo_resource_paths = [install_dir_path,
                            pkg_barista_gazebo,
                            gazebo_models_path]
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["GZ_SIM_RESOURCE_PATH"]:
                os.environ["GZ_SIM_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))
 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [
            '-r empty.sdf',
        ]}.items(),
    )
 
    declare_spawn_model_name = DeclareLaunchArgument("model_name", default_value="my_robot",
                                                     description="Model Spawn Name")
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.2",
                                            description="Model Spawn Z Axis Value")
    declare_spawn_yaw = DeclareLaunchArgument("yaw", default_value="3.14",
                                            description="Model Spawn Yaw Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )
 
    # ROS-Gazebo Bridge #
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@gz.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[gz.msgs.Odometry",
            "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
        ],
        remappings=[],
        output="screen",
    )
 
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
 
    spawn_controller_traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )
 
    spawn_controller_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen",
    )
 
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[spawn_controller, spawn_controller_traj, spawn_controller_velocity],
        )
    )
 
    return [
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher_node,
        gz_sim,
        rviz_node,
        declare_spawn_model_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        gz_spawn_entity,
        gz_bridge,
        delayed_controller_spawners,
    ]
 
 
def generate_launch_description():
    declare_include_laser = DeclareLaunchArgument(
        "include_laser", default_value="true",
        description="Whether to include the laser scanner in the URDF"
    )
 
    return LaunchDescription([
        declare_include_laser,
        OpaqueFunction(function=launch_setup),
    ])
 
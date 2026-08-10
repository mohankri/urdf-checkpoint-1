import os

#from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.actions import (Node, SetParameter)


# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'barista_robot_model.urdf'
    #xacro_file = "box_bot.xacro"
    package_description = "barista_robot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
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
    pkg_barista_gazebo = get_package_share_directory('barista_robot_description')
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    #pkg_box_bot_description = get_package_share_directory('barista_robot_description')

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
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
            '-r empty.sdf',  # <-- start unpaused
            #PathJoinSubstitution([pkg_barista_gazebo, 'worlds', 'box_bot_empty.world'])
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
        remappings=[
            # there are no remappings for this robot description
        ],
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

    # create and return launch description object
    return LaunchDescription(
        [       
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
            spawn_controller,
            spawn_controller_traj,
            spawn_controller_velocity
        ]
    )
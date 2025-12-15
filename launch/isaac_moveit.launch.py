from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def generate_launch_description():
    # 1. Define Paths
    pkg_name = 'fanuc_isaac_wrapper'

    # 2. Process URDF (Xacro)
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'isaac_control.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_content = doc.toxml()

    # 3. Process SRDF
    srdf_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'm900ib700.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    # 4. Load Configurations
    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # A. MoveIt Controllers Config (Planner Side)
    moveit_controllers_yaml = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'moveit_controllers.yaml'
    )

    # B. ROS 2 Control Config (Driver Side)
    ros2_controllers_yaml = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'ros2_controllers.yaml'
    )

    # 4.1 Load Kinematics Configuration
    kinematics_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'kinematics.yaml')
    try:
        with open(kinematics_file, 'r') as f:
            kinematics_config = yaml.safe_load(f)
    except EnvironmentError:
        print(f"WARNING: Could not load kinematics file: {kinematics_file}")
        kinematics_config = {}

    # IMPORTANT: Servo expects kinematics under 'robot_description_kinematics'
    robot_description_kinematics = {'robot_description_kinematics': kinematics_config}

    # --- FIX: Use the Servo YAML file AS A PARAMETER FILE ---
    servo_yaml_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'servo.yaml')

    # 5. Nodes

    # A. Controller Manager (ROS 2 Control)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_yaml, {'use_sim_time': True}],
        output='screen'
    )

    # B. Robot State Publisher (TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # C. Spawners
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # D. MoveGroup (MoveIt Planning)
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"""
        }
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            {'use_sim_time': True}
        ]
    )

    # E. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True}
        ]
    )

    # F. MoveIt Servo (standalone node)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_yaml_file,                # <-- THIS is the key fix
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # G. Your PID node
    pid_node = Node(
        package=pkg_name,
        executable='tcp_pid_servo.py',
        name='tcp_pid_servo',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        control_node,
        rsp_node,
        spawn_jsb,
        spawn_traj,
        move_group_node,
        rviz_node,
        servo_node,
        pid_node,
    ])

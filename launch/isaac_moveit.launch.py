#!/usr/bin/env python3
"""
fanuc_isaac_wrapper: Isaac Sim + ros2_control + MoveIt + Servo

Control path:

    PID node -> /servo_node/delta_twist_cmds (Twist)
           Servo -> /joint_trajectory_controller/joint_trajectory (JointTrajectory)
 joint_trajectory_controller (ros2_control) -> /joint_commands (JointState)
        Isaac Sim ActionGraph subscribes /joint_commands -> drives articulation

Feedback path:

    Isaac Sim ActionGraph publishes /joint_states (+ /clock)
    ros2_control hardware reads /joint_states
    MoveIt + robot_state_publisher listen to /joint_states

Important: We do NOT spawn joint_state_broadcaster because Isaac is already publishing /joint_states.
"""

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def load_file(pkg_name: str, relative_path: str) -> str:
    pkg_path = get_package_share_directory(pkg_name)
    abs_path = os.path.join(pkg_path, relative_path)
    with open(abs_path, "r") as f:
        return f.read()


def load_yaml(pkg_name: str, relative_path: str):
    pkg_path = get_package_share_directory(pkg_name)
    abs_path = os.path.join(pkg_path, relative_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = "fanuc_isaac_wrapper"

    # --- Robot description (URDF via xacro) ---
    xacro_path = os.path.join(get_package_share_directory(pkg), "config", "isaac_control.xacro")
    robot_description = {"robot_description": xacro.process_file(xacro_path).toxml()}

    # --- Semantic description (SRDF) ---
    robot_description_semantic = {"robot_description_semantic": load_file(pkg, "config/m900ib700.srdf")}

    # --- Kinematics (MoveIt expects robot_description_kinematics) ---
    kinematics_yaml = load_yaml(pkg, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # --- Parameter files ---
    ros2_controllers_yaml = os.path.join(get_package_share_directory(pkg), "config", "ros2_controllers.yaml")
    moveit_controllers_yaml = os.path.join(get_package_share_directory(pkg), "config", "moveit_controllers.yaml")
    servo_yaml = os.path.join(get_package_share_directory(pkg), "config", "servo.yaml")

    use_sim_time = {"use_sim_time": True}

    # 1) robot_state_publisher (also publishes /robot_description topic)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, use_sim_time],
    )

    # 2) controller_manager (ros2_control_node)
    # - do NOT pass robot_description parameter (deprecated)
    # - controller_manager subscribes to /controller_manager/robot_description
    # - robot_state_publisher publishes /robot_description
    #   => we remap so it receives it.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[ros2_controllers_yaml, use_sim_time],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # 3) Spawn ONLY the joint_trajectory_controller (this is what claims joint_*/position)
    spawn_traj = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
    )

    # Delay spawning so controller_manager has robot_description + IsaacSimSystem initialized.
    delayed_spawn_traj = TimerAction(period=2.0, actions=[spawn_traj])

    # 4) MoveIt + Servo + PID (start after the trajectory controller is active)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_controllers_yaml,
            use_sim_time,
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_yaml,
            use_sim_time,
        ],
    )

    tcp_pid_servo_node = Node(
        package=pkg,
        executable="tcp_pid_servo.py",
        name="tcp_pid_servo",
        output="screen",
        parameters=[use_sim_time],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            use_sim_time,
        ],
    )

    start_moveit_after_traj = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_traj,
            on_exit=[
                move_group_node,
                servo_node,
                tcp_pid_servo_node,
                rviz_node,
            ],
        )
    )

    return LaunchDescription(
        [
            rsp_node,
            control_node,
            delayed_spawn_traj,
            start_moveit_after_traj,
        ]
    )

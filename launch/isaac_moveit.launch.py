from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def generate_launch_description():
    pkg_name = "fanuc_isaac_wrapper"
    pkg_share = get_package_share_directory(pkg_name)

    # ---------------------------
    # Launch args
    # ---------------------------
    start_pid_arg = DeclareLaunchArgument(
        "start_pid",
        default_value="false",
        description="Start tcp_pid_servo node automatically (default: false).",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz (default: true).",
    )

    start_pid = LaunchConfiguration("start_pid")
    start_rviz = LaunchConfiguration("start_rviz")

    use_sim_time = {"use_sim_time": True}

    # ---------------------------
    # Robot description (URDF/Xacro)
    # ---------------------------
    xacro_file = os.path.join(pkg_share, "config", "isaac_control.xacro")
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_content}

    # ---------------------------
    # SRDF
    # ---------------------------
    srdf_file = os.path.join(pkg_share, "config", "m900ib700.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # ---------------------------
    # Kinematics
    # ---------------------------
    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")
    with open(kinematics_file, "r") as f:
        kin_yaml = yaml.safe_load(f)
    robot_description_kinematics = {"robot_description_kinematics": kin_yaml}

    # ---------------------------
    # YAML files
    # ---------------------------
    ros2_controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    moveit_controllers_yaml = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    servo_yaml_file = os.path.join(pkg_share, "config", "servo.yaml")

    # ---------------------------
    # Nodes
    # ---------------------------

    # TF publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, use_sim_time],
    )

    # ros2_control controller manager
    # (Yes this still shows the "deprecated robot_description param" warning in some setups.
    # It is not what is blocking you right now. Keeping it stable > chasing warnings.)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, ros2_controllers_yaml, use_sim_time],
    )

    # Spawn ONLY the joint_trajectory_controller (exactly once)
    # This is what CLAIMS joint_i/position.
    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120",
        ],
    )

    # MoveIt move_group (planner/execution)
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
        }
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,
            use_sim_time,
        ],
    )

    # MoveIt Servo (do NOT start until JTC is spawned/active)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_yaml_file,   # <-- IMPORTANT: pass as param FILE
            use_sim_time,
        ],
    )

    # Your PID node (optional; OFF by default so you can move out of singularity first)
    pid_node = Node(
        package=pkg_name,
        executable="tcp_pid_servo.py",
        name="tcp_pid_servo",
        output="screen",
        parameters=[use_sim_time],
        condition=IfCondition(start_pid),
    )

    # RViz (optional)
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
        condition=IfCondition(start_rviz),
    )

    # ---------------------------
    # Event ordering (the key fix)
    # ---------------------------

    # Start spawner only after controller_manager starts
    spawn_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[spawn_jtc],
        )
    )

    # Start Servo only after spawner finished (meaning controller is loaded+configured+activated)
    servo_after_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jtc,
            on_exit=[servo_node, pid_node],
        )
    )

    return LaunchDescription([
        start_pid_arg,
        start_rviz_arg,

        rsp_node,
        control_node,
        spawn_after_control,

        move_group_node,
        servo_after_jtc,

        rviz_node,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

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
    
    # --- UPDATED SECTION START ---
    
    # A. MoveIt Controllers Config (Planner Side)
    # This file should ONLY contain the 'moveit_simple_controller_manager' stuff
    moveit_controllers_yaml = os.path.join(get_package_share_directory(pkg_name), 'config', 'moveit_controllers.yaml')

    # B. ROS 2 Control Config (Driver Side)
    # This file should contain the 'controller_manager' and controller definitions
    ros2_controllers_yaml = os.path.join(get_package_share_directory(pkg_name), 'config', 'ros2_controllers.yaml')

    # --- UPDATED SECTION END ---

    # Load Kinematics Configuration
    kinematics_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'kinematics.yaml')
    try:
        with open(kinematics_file, 'r') as f:
            kinematics_config = yaml.safe_load(f)
    except EnvironmentError:
        print(f"WARNING: Could not load kinematics file: {kinematics_file}")
        kinematics_config = {}
    
    # 5. Nodes
    
    # A. Controller Manager (The "Brain" of ROS 2 Control)
    # CRITICAL CHANGE: Uses ros2_controllers_yaml now
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_yaml],
        output='screen'
    )

    # B. Robot State Publisher (Publishes TF frames)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # C. Spawners (Start the controllers automatically)
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

    # D. MoveGroup (MoveIt)
    # Basic MoveIt configuration
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
            kinematics_config,
            ompl_planning_pipeline_config,
            moveit_controllers_yaml,            # <--- Uses the MoveIt specific YAML
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
            kinematics_config, 
            {'use_sim_time': True}            
        ]
    )

    return LaunchDescription([
        control_node,
        rsp_node,
        spawn_jsb,
        spawn_traj,
        move_group_node,
        rviz_node
    ])
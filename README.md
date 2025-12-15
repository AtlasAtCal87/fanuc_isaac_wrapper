


ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
pkill -f tcp_pid_servo


ros2 topic pub -1 /joint_commands sensor_msgs/msg/JointState "
name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
position: [0.0, 0.3, 0.0, 0.0, 0.0, 0.0]
"

or ~/joint_command_stream.py

Configure controller

rosrun controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --controller-manager-timeout 120


ros2 run controller_manager spawner joint_trajectory_controller --controller-manager /controller_manager --controller-manager-timeout 120
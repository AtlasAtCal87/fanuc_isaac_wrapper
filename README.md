


ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
pkill -f tcp_pid_servo
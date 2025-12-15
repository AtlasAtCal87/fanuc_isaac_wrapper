#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


def quat_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_conjugate(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=float)


def quat_multiply(q1, q2):
    # ROS quats are (x,y,z,w)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)


def quat_to_rotvec(q):
    # Convert unit quaternion to rotation vector (axis * angle)
    q = quat_normalize(q)
    x, y, z, w = q
    v = np.array([x, y, z], dtype=float)
    v_norm = np.linalg.norm(v)

    # Clamp for numerical stability
    w = float(np.clip(w, -1.0, 1.0))

    if v_norm < 1e-12:
        return np.zeros(3)

    angle = 2.0 * math.atan2(v_norm, w)
    axis = v / v_norm
    return axis * angle


def quat_to_rotmat(q):
    x, y, z, w = quat_normalize(q)
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=float)


def apply_tf_to_pose(tf_trans, tf_quat, pose_trans, pose_quat):
    """
    Apply transform (target <- source) to a pose expressed in source.
    Returns pose expressed in target.
    """
    R = quat_to_rotmat(tf_quat)
    t = np.array(tf_trans, dtype=float)

    p = np.array(pose_trans, dtype=float)
    p_out = R.dot(p) + t

    q_out = quat_multiply(tf_quat, pose_quat)
    q_out = quat_normalize(q_out)

    return p_out, q_out


class PID6D:
    def __init__(self, kp, ki, kd, dt, integral_limits=(-0.05, 0.05)):
        self.kp = np.array(kp, dtype=float)
        self.ki = np.array(ki, dtype=float)
        self.kd = np.array(kd, dtype=float)
        self.dt = float(dt)
        self.int_limits = integral_limits
        self.i = np.zeros(6, dtype=float)
        self.prev_e = np.zeros(6, dtype=float)

    def reset(self):
        self.i[:] = 0.0
        self.prev_e[:] = 0.0

    def update(self, e):
        e = np.array(e, dtype=float)

        self.i += e * self.dt
        self.i = np.clip(self.i, self.int_limits[0], self.int_limits[1])

        de = (e - self.prev_e) / self.dt
        self.prev_e = e.copy()

        # Output is a *velocity* command (m/s and rad/s) if kp/ki/kd are tuned that way
        return self.kp * e + self.ki * self.i + self.kd * de


class TcpPidServoNode(Node):
    def __init__(self):
        super().__init__("tcp_pid_servo")

        # ---- Parameters ----
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("desired_frame", "tool0")          # FK frame in TF
        self.declare_parameter("measured_topic", "/tcp_measured") # from Isaac
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("start_servo_service", "/servo_node/start_servo")

        # Gains in *velocity domain* (output is m/s and rad/s)
        self.declare_parameter("kp", [0.35, 0.35, 0.35, 0.10, 0.10, 0.10])
        self.declare_parameter("ki", [0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
        self.declare_parameter("kd", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Anti-windup (keep small; units are (m*s) and (rad*s))
        self.declare_parameter("integral_min", -0.05)
        self.declare_parameter("integral_max", 0.05)

        # Command saturation
        self.declare_parameter("max_linear_speed", 0.05)   # m/s
        self.declare_parameter("max_angular_speed", 0.25)  # rad/s

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate_hz

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.desired_frame = str(self.get_parameter("desired_frame").value)
        self.measured_topic = str(self.get_parameter("measured_topic").value)
        self.servo_twist_topic = str(self.get_parameter("servo_twist_topic").value)
        self.start_servo_service = str(self.get_parameter("start_servo_service").value)

        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        i_min = float(self.get_parameter("integral_min").value)
        i_max = float(self.get_parameter("integral_max").value)

        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.max_ang = float(self.get_parameter("max_angular_speed").value)

        self.pid = PID6D(kp, ki, kd, self.dt, integral_limits=(i_min, i_max))

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS I/O ----
        self.measured_pose = None
        self.create_subscription(PoseStamped, self.measured_topic, self._measured_cb, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.servo_twist_topic, 10)

        # Start Servo automatically
        self.start_client = self.create_client(Trigger, self.start_servo_service)
        self._start_timer = self.create_timer(1.0, self._try_start_servo)

        # Control loop timer
        self.timer = self.create_timer(self.dt, self._control_cb)

        self.get_logger().info(
            f"PID servo node running. measured={self.measured_topic} → cmd={self.servo_twist_topic}, "
            f"desired TF: {self.base_frame} -> {self.desired_frame}"
        )

    def _measured_cb(self, msg: PoseStamped):
        # Strip leading slash if Isaac publishes "/base_link"
        if msg.header.frame_id.startswith("/"):
            msg.header.frame_id = msg.header.frame_id[1:]
        self.measured_pose = msg

    def _try_start_servo(self):
        if not self.start_client.service_is_ready():
            self.get_logger().info("Waiting for MoveIt Servo start service...")
            return
        req = Trigger.Request()
        fut = self.start_client.call_async(req)

        def _done(_):
            self.get_logger().info("Requested Servo start.")
            self._start_timer.cancel()

        fut.add_done_callback(_done)

    def _control_cb(self):
        if self.measured_pose is None:
            return

        now = self.get_clock().now()

        # 1) Desired pose from TF: base_link <- tool0  (FK pose)
        try:
            tf_des = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.desired_frame,
                Time()  # latest
            )
        except TransformException:
            return

        p_des = np.array([
            tf_des.transform.translation.x,
            tf_des.transform.translation.y,
            tf_des.transform.translation.z
        ], dtype=float)
        q_des = np.array([
            tf_des.transform.rotation.x,
            tf_des.transform.rotation.y,
            tf_des.transform.rotation.z,
            tf_des.transform.rotation.w
        ], dtype=float)
        q_des = quat_normalize(q_des)

        # 2) Measured pose → transform into base_frame
        src_frame = self.measured_pose.header.frame_id
        try:
            tf_base_from_src = self.tf_buffer.lookup_transform(
                self.base_frame,
                src_frame,
                Time()  # latest
            )
        except TransformException:
            return

        tf_t = [
            tf_base_from_src.transform.translation.x,
            tf_base_from_src.transform.translation.y,
            tf_base_from_src.transform.translation.z
        ]
        tf_q = [
            tf_base_from_src.transform.rotation.x,
            tf_base_from_src.transform.rotation.y,
            tf_base_from_src.transform.rotation.z,
            tf_base_from_src.transform.rotation.w
        ]

        p_m = [
            self.measured_pose.pose.position.x,
            self.measured_pose.pose.position.y,
            self.measured_pose.pose.position.z
        ]
        q_m = [
            self.measured_pose.pose.orientation.x,
            self.measured_pose.pose.orientation.y,
            self.measured_pose.pose.orientation.z,
            self.measured_pose.pose.orientation.w
        ]

        p_meas, q_meas = apply_tf_to_pose(tf_t, tf_q, p_m, q_m)

        # 3) Error (desired - measured)
        pos_err = p_des - p_meas

        # Orientation error as rotation vector:
        # q_err rotates measured -> desired
        q_meas_inv = quat_conjugate(quat_normalize(q_meas))
        q_err = quat_multiply(q_des, q_meas_inv)
        rot_err = quat_to_rotvec(q_err)

        e6 = np.concatenate([pos_err, rot_err], axis=0)

        # 4) PID → twist cmd
        u = self.pid.update(e6)
        lin = u[0:3]
        ang = u[3:6]

        # Saturate
        ln = np.linalg.norm(lin)
        if ln > self.max_lin and ln > 1e-12:
            lin = lin * (self.max_lin / ln)

        an = np.linalg.norm(ang)
        if an > self.max_ang and an > 1e-12:
            ang = ang * (self.max_ang / an)

        # 5) Publish TwistStamped to Servo
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame

        msg.twist.linear.x = float(lin[0])
        msg.twist.linear.y = float(lin[1])
        msg.twist.linear.z = float(lin[2])

        msg.twist.angular.x = float(ang[0])
        msg.twist.angular.y = float(ang[1])
        msg.twist.angular.z = float(ang[2])

        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = TcpPidServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

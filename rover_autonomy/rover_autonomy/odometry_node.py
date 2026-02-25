"""
Odometry Node for ROS2 Jazzy.

Reads encoder data from /encoder_raw (published by mqtt_bridge) and
computes wheel odometry using differential drive kinematics.

This node has ZERO MQTT dependency.

The ESP32 sends SIGNED quadrature encoder deltas (+ forward, - reverse),
so no direction inference from cmd_vel is needed.

Noise filtering:
    - When motors are stopped (cmd_vel = 0), encoder ticks are IGNORED
      because the encoders pick up EMI noise from the motor driver.
    - A configurable min_delta_ticks deadzone filters residual noise
      even while driving.

Publishes:
    - nav_msgs/Odometry on /odom
    - TF: odom -> base_link

Subscribes:
    - /encoder_raw (std_msgs/String) -- format: "L_total,R_total,L_delta,R_delta" (signed)
    - /cmd_vel (geometry_msgs/Twist) -- only to detect if motors are active (noise filter)
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryNode(Node):
    """Computes and publishes odometry from signed encoder data (pure ROS2)."""

    def __init__(self):
        super().__init__('odometry_node')

        # Declare parameters (calibrate with real measurements)
        self.declare_parameter('wheel_diameter', 0.06)
        self.declare_parameter('wheel_base', 0.40)
        self.declare_parameter('encoder_ticks_per_rev_left', 200)
        self.declare_parameter('encoder_ticks_per_rev_right', 200)
        self.declare_parameter('min_delta_ticks', 3)
        self.declare_parameter('stop_grace_sec', 0.4)

        # Read parameters
        self.wheel_diameter = self.get_parameter(
            'wheel_diameter').get_parameter_value().double_value
        self.wheel_base = self.get_parameter(
            'wheel_base').get_parameter_value().double_value
        self.ticks_per_rev_left = self.get_parameter(
            'encoder_ticks_per_rev_left').get_parameter_value().integer_value
        self.ticks_per_rev_right = self.get_parameter(
            'encoder_ticks_per_rev_right').get_parameter_value().integer_value
        self.min_delta_ticks = self.get_parameter(
            'min_delta_ticks').get_parameter_value().integer_value
        self.stop_grace_sec = self.get_parameter(
            'stop_grace_sec').get_parameter_value().double_value

        # Derived
        self.wheel_radius = self.wheel_diameter / 2.0
        self.meters_per_tick_left = (
            (math.pi * self.wheel_diameter) / self.ticks_per_rev_left
        )
        self.meters_per_tick_right = (
            (math.pi * self.wheel_diameter) / self.ticks_per_rev_right
        )

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Motor state tracking -- filter noise when stopped
        self.motors_active = False
        self.last_nonzero_cmd_time = self.get_clock().now()
        self.noise_filtered_count = 0

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.encoder_debug_pub = self.create_publisher(String, 'encoder', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel ONLY for noise filtering (not direction)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        # Subscribe to encoder data from mqtt_bridge
        self.create_subscription(
            String, 'encoder_raw', self._encoder_callback, 50)

        self.get_logger().info(
            f'Odometry node ready (quadrature encoders, signed deltas). '
            f'wheel_d={self.wheel_diameter}m, base={self.wheel_base}m, '
            f'ticks/rev L={self.ticks_per_rev_left} R={self.ticks_per_rev_right}, '
            f'noise filter: min_delta={self.min_delta_ticks} ticks, '
            f'stop_grace={self.stop_grace_sec}s'
        )

    def _cmd_vel_callback(self, msg):
        """Track whether motors are active (for noise filtering only)."""
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.motors_active = True
            self.last_nonzero_cmd_time = self.get_clock().now()
        else:
            elapsed = (self.get_clock().now() -
                       self.last_nonzero_cmd_time).nanoseconds / 1e9
            if elapsed > self.stop_grace_sec:
                self.motors_active = False

    def _encoder_callback(self, msg):
        """Process signed encoder data from mqtt_bridge."""
        try:
            data = msg.data.strip()

            # Republish as debug topic
            debug_msg = String()
            debug_msg.data = data
            self.encoder_debug_pub.publish(debug_msg)

            # Parse: "L_total,R_total,L_delta,R_delta" (all signed)
            parts = data.split(',')
            if len(parts) != 4:
                return

            # Signed deltas from quadrature encoders (+ forward, - reverse)
            delta_left = int(parts[2])
            delta_right = int(parts[3])

            # ── NOISE FILTER ────────────────────────────────────────
            if not self.motors_active:
                if delta_left != 0 or delta_right != 0:
                    self.noise_filtered_count += 1
                    if self.noise_filtered_count % 25 == 1:
                        self.get_logger().info(
                            f'Noise filtered: L={delta_left} R={delta_right} '
                            f'(motors stopped, {self.noise_filtered_count} '
                            f'total filtered)')
                delta_left = 0
                delta_right = 0
            else:
                # Deadzone: ignore tiny deltas (noise while driving)
                if abs(delta_left) < self.min_delta_ticks:
                    delta_left = 0
                if abs(delta_right) < self.min_delta_ticks:
                    delta_right = 0
                self.noise_filtered_count = 0

            # Convert ticks to meters (sign already correct from ESP32)
            d_left = delta_left * self.meters_per_tick_left
            d_right = delta_right * self.meters_per_tick_right

            # Differential drive odometry
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.wheel_base

            # Update pose
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            self.theta += d_theta
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

            # Compute velocities
            if dt > 0:
                vx = d_center / dt
                vth = d_theta / dt
            else:
                vx = 0.0
                vth = 0.0

            # Publish TF: odom -> base_link
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(self.theta)
            self.tf_broadcaster.sendTransform(t)

            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion_from_yaw(self.theta)

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().debug(f'Encoder parse error: {e}')

    def destroy_node(self):
        """Cleanup."""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Odometry Monitor Node for ROS2 Jazzy.

Subscribes to /odom and prints a clean summary:
    - Current position (x, y)
    - Total distance traveled (odometer-style, accumulated)
    - Current heading in degrees
    - Raw encoder ticks from /encoder_raw

Usage:
    ros2 run rover_autonomy odom_monitor
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class OdomMonitorNode(Node):

    def __init__(self):
        super().__init__('odom_monitor')

        self.declare_parameter('rate_hz', 5.0)
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.heading_deg = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        self.enc_left_total = 0
        self.enc_right_total = 0
        self.enc_left_delta = 0
        self.enc_right_delta = 0

        self.odom_count = 0

        self.create_subscription(Odometry, 'odom', self._odom_callback, 50)
        self.create_subscription(String, 'encoder_raw', self._encoder_callback, 50)
        self.create_timer(1.0 / rate, self._print_status)

        print('\n' + '=' * 60)
        print('  ODOMETRY MONITOR')
        print('=' * 60)
        print('  Waiting for /odom data...')
        print('=' * 60 + '\n')

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.prev_x is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            step = math.sqrt(dx * dx + dy * dy)
            if step < 0.1:
                self.total_distance += step

        self.prev_x = x
        self.prev_y = y
        self.pos_x = x
        self.pos_y = y
        self.heading_deg = math.degrees(yaw)
        self.odom_count += 1

    def _encoder_callback(self, msg):
        try:
            parts = msg.data.strip().split(',')
            if len(parts) == 4:
                self.enc_left_total = int(parts[0])
                self.enc_right_total = int(parts[1])
                self.enc_left_delta = int(parts[2])
                self.enc_right_delta = int(parts[3])
        except Exception:
            pass

    def _print_status(self):
        if self.odom_count == 0:
            return

        print(
            f'\r  pos: ({self.pos_x:+.3f}, {self.pos_y:+.3f})m  '
            f'dist: {self.total_distance:.3f}m  '
            f'heading: {self.heading_deg:+.1f}°  '
            f'enc L:{self.enc_left_total}({self.enc_left_delta:+d}) '
            f'R:{self.enc_right_total}({self.enc_right_delta:+d})  ',
            end='', flush=True
        )

    def destroy_node(self):
        print('\n')
        print('=' * 60)
        print(f'  FINAL: distance={self.total_distance:.3f}m  '
              f'heading={self.heading_deg:+.1f}°')
        print('=' * 60)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Encoder Calibration Node for ROS2 Jazzy.

Spins a single motor for a given number of encoder ticks, then reports
the result. Use this to measure ticks_per_revolution for each wheel.

Usage:
    # Spin right wheel for 500 ticks:
    ros2 run rover_autonomy encoder_calibration --ros-args -p ticks:=500 -p motor:=right

    # Spin left wheel for 300 ticks at PWM 60:
    ros2 run rover_autonomy encoder_calibration --ros-args -p ticks:=300 -p motor:=left -p pwm:=60

Mark the wheel before starting, count how many full revolutions it makes,
then: ticks_per_rev = ticks / revolutions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class EncoderCalibrationNode(Node):

    def __init__(self):
        super().__init__('encoder_calibration')

        self.declare_parameter('ticks', 500)
        self.declare_parameter('motor', 'left')
        self.declare_parameter('pwm', 80)

        self.target_ticks = self.get_parameter('ticks').get_parameter_value().integer_value
        self.motor_side = self.get_parameter('motor').get_parameter_value().string_value.lower()
        self.pwm_value = self.get_parameter('pwm').get_parameter_value().integer_value

        if self.motor_side not in ('left', 'right'):
            self.get_logger().error(f'Invalid motor side: "{self.motor_side}". Use "left" or "right".')
            raise SystemExit(1)

        self.motor_left_pub = self.create_publisher(Int32, 'motor_left', 10)
        self.motor_right_pub = self.create_publisher(Int32, 'motor_right', 10)

        self.create_subscription(String, 'encoder_raw', self._encoder_callback, 50)

        self.start_left = None
        self.start_right = None
        self.running = False
        self.done = False
        self.first_msg = True

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  ENCODER CALIBRATION')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Motor : {self.motor_side.upper()}')
        self.get_logger().info(f'  Target: {self.target_ticks} ticks')
        self.get_logger().info(f'  PWM   : {self.pwm_value}')
        self.get_logger().info('')
        self.get_logger().info('  Mark the wheel now! Waiting for encoder data...')
        self.get_logger().info('=' * 50)

        self.create_timer(0.05, self._control_loop)

    def _encoder_callback(self, msg):
        try:
            parts = msg.data.strip().split(',')
            if len(parts) != 4:
                return

            enc_left = int(parts[0])
            enc_right = int(parts[1])

            if self.first_msg:
                self.start_left = enc_left
                self.start_right = enc_right
                self.running = True
                self.first_msg = False
                self.get_logger().info(f'  Start totals: L={enc_left}  R={enc_right}')
                self.get_logger().info(f'  Spinning {self.motor_side} wheel...')
                return

            if self.done:
                return

            if self.motor_side == 'left':
                elapsed = abs(enc_left - self.start_left)
            else:
                elapsed = abs(enc_right - self.start_right)

            if elapsed % 100 == 0 and elapsed > 0:
                self.get_logger().info(f'  Progress: {elapsed} / {self.target_ticks} ticks')

            if elapsed >= self.target_ticks:
                self.done = True
                self._stop_motors()

                delta_l = enc_left - self.start_left
                delta_r = enc_right - self.start_right

                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('  DONE!')
                self.get_logger().info(f'  Left  encoder moved: {delta_l} ticks')
                self.get_logger().info(f'  Right encoder moved: {delta_r} ticks')
                self.get_logger().info('')
                self.get_logger().info('  Count how many full revolutions the wheel made.')
                self.get_logger().info('  ticks_per_rev = ticks / revolutions')
                self.get_logger().info('=' * 50)

        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')

    def _control_loop(self):
        if not self.running or self.done:
            return

        msg = Int32()
        msg.data = self.pwm_value

        if self.motor_side == 'left':
            self.motor_left_pub.publish(msg)
        else:
            self.motor_right_pub.publish(msg)

    def _stop_motors(self):
        stop = Int32()
        stop.data = 0
        for _ in range(10):
            self.motor_left_pub.publish(stop)
            self.motor_right_pub.publish(stop)

    def destroy_node(self):
        self._stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

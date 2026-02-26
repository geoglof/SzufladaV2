"""
Rover Teleop Node for ROS2 Jazzy.

Continuous keyboard-based teleoperation.
Press a key to SET the driving state -- it keeps going until you
press another key or SPACE to stop. No "one-shot" steps.

Publishes /cmd_vel at 10 Hz continuously.
The cmd_vel_bridge converts it to motor PWM, and mqtt_bridge
sends it to the ESP32. This node has ZERO MQTT dependency.

Controls:
    W = Drive forward (continuous)
    S = Drive backward (continuous)
    A = Turn left (continuous)
    D = Turn right (continuous)
    SPACE = Stop
    L = Toggle LED
    +/- = Increase/Decrease linear speed
    [/] = Decrease/Increase turn speed
    Q = Quit
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RoverTeleopNode(Node):
    """Continuous keyboard teleoperation node (pure ROS2, no MQTT)."""

    def __init__(self):
        super().__init__('rover_teleop')

        self.declare_parameter('default_speed', 0.20)
        self.declare_parameter('turn_speed', 1.2)
        self.declare_parameter('speed_step', 0.03)
        self.declare_parameter('turn_step', 0.15)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_turn', 3.0)
        self.declare_parameter('min_turn', 0.3)

        self.linear_speed = self.get_parameter(
            'default_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter(
            'turn_speed').get_parameter_value().double_value
        self.speed_step = self.get_parameter(
            'speed_step').get_parameter_value().double_value
        self.turn_step = self.get_parameter(
            'turn_step').get_parameter_value().double_value
        self.max_speed = self.get_parameter(
            'max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter(
            'min_speed').get_parameter_value().double_value
        self.max_turn = self.get_parameter(
            'max_turn').get_parameter_value().double_value
        self.min_turn = self.get_parameter(
            'min_turn').get_parameter_value().double_value

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.led_on = False
        self.running = True

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.led_pub = self.create_publisher(String, 'led_cmd', 10)

        self.create_timer(0.1, self._publish_current_state)

        self._print_instructions()

        self.kb_thread = threading.Thread(target=self._read_keyboard, daemon=True)
        self.kb_thread.start()

    def _print_instructions(self):
        print('\n' + '=' * 50)
        print('    SZUFLADA V2 TELEOP (ROS2 Jazzy)')
        print('=' * 50)
        print('  Movement (CONTINUOUS -- holds until changed):')
        print('            W')
        print('        A   S   D       W = Forward')
        print('                        S = Backward')
        print('                        A = Turn Left')
        print('                        D = Turn Right')
        print('')
        print('  SPACE = STOP')
        print('  L     = Toggle LED')
        print('  +/-   = Increase/Decrease linear speed')
        print('  ]/[   = Increase/Decrease turn speed')
        print('  Q     = Quit')
        print('')
        print(f'  Linear speed : {self.linear_speed:.2f} m/s')
        print(f'  Turn speed   : {self.turn_speed:.2f} rad/s')
        print('=' * 50 + '\n')

    def _publish_current_state(self):
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_vel_pub.publish(twist)

    def _read_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self._handle_key(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _handle_key(self, key):
        if key.lower() == 'w':
            self.current_linear = self.linear_speed
            self.current_angular = 0.0
            self.get_logger().info(
                f'FORWARD ({self.linear_speed:.2f} m/s)')

        elif key.lower() == 's':
            self.current_linear = -self.linear_speed
            self.current_angular = 0.0
            self.get_logger().info(
                f'BACKWARD ({self.linear_speed:.2f} m/s)')

        elif key.lower() == 'a':
            self.current_linear = 0.0
            self.current_angular = self.turn_speed
            self.get_logger().info(
                f'TURN LEFT ({self.turn_speed:.2f} rad/s)')

        elif key.lower() == 'd':
            self.current_linear = 0.0
            self.current_angular = -self.turn_speed
            self.get_logger().info(
                f'TURN RIGHT ({self.turn_speed:.2f} rad/s)')

        elif key == ' ':
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.get_logger().info('STOP')

        elif key.lower() == 'l':
            self.led_on = not self.led_on
            command = 'ON' if self.led_on else 'OFF'
            led_msg = String()
            led_msg.data = command
            self.led_pub.publish(led_msg)
            self.get_logger().info(f'LED: {command}')

        elif key in ('+', '='):
            self.linear_speed = min(
                self.max_speed, self.linear_speed + self.speed_step)
            self._update_active_speed()
            self.get_logger().info(
                f'Linear speed: {self.linear_speed:.2f} m/s')

        elif key == '-':
            self.linear_speed = max(
                self.min_speed, self.linear_speed - self.speed_step)
            self._update_active_speed()
            self.get_logger().info(
                f'Linear speed: {self.linear_speed:.2f} m/s')

        elif key == ']':
            self.turn_speed = min(
                self.max_turn, self.turn_speed + self.turn_step)
            self._update_active_turn()
            self.get_logger().info(
                f'Turn speed: {self.turn_speed:.2f} rad/s')

        elif key == '[':
            self.turn_speed = max(
                self.min_turn, self.turn_speed - self.turn_step)
            self._update_active_turn()
            self.get_logger().info(
                f'Turn speed: {self.turn_speed:.2f} rad/s')

        elif key.lower() == 'q' or key == '\x03':
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.running = False
            self.get_logger().info('Exiting...')

    def _update_active_speed(self):
        """If currently driving, update to the new speed immediately."""
        if self.current_linear > 0:
            self.current_linear = self.linear_speed
        elif self.current_linear < 0:
            self.current_linear = -self.linear_speed

    def _update_active_turn(self):
        """If currently turning, update to the new turn speed immediately."""
        if self.current_angular > 0:
            self.current_angular = self.turn_speed
        elif self.current_angular < 0:
            self.current_angular = -self.turn_speed

    def destroy_node(self):
        self.running = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

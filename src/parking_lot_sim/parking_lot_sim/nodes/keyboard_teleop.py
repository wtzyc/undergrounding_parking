import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
Keyboard teleop for parking_lot_sim

  w / s : increase / decrease speed
  a / d : steer left / right
  x     : reverse speed
  space : stop
  q / e : decrease / increase speed step
  z / c : decrease / increase steer step
  Ctrl-C: quit
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed_step', 0.4)
        self.declare_parameter('steer_step', 0.25)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('max_steer', 0.55)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._publisher = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self._speed = 0.0
        self._steer = 0.0
        self._speed_step = float(self.get_parameter('speed_step').value)
        self._steer_step = float(self.get_parameter('steer_step').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._max_steer = float(self.get_parameter('max_steer').value)
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value),
            self._publish_cmd,
        )

    def handle_key(self, key: str) -> bool:
        if key == '\x03':
            return False
        if key == 'w':
            self._speed += self._speed_step
        elif key == 's':
            self._speed -= self._speed_step
        elif key == 'x':
            self._speed = -self._speed
        elif key == 'a':
            self._steer += self._steer_step
        elif key == 'd':
            self._steer -= self._steer_step
        elif key == ' ':
            self._speed = 0.0
            self._steer = 0.0
        elif key == 'q':
            self._speed_step = max(0.1, self._speed_step - 0.1)
        elif key == 'e':
            self._speed_step = min(1.0, self._speed_step + 0.1)
        elif key == 'z':
            self._steer_step = max(0.05, self._steer_step - 0.05)
        elif key == 'c':
            self._steer_step = min(0.55, self._steer_step + 0.05)
        else:
            return True

        self._speed = self._clamp(self._speed, -self._max_speed, self._max_speed)
        self._steer = self._clamp(self._steer, -self._max_steer, self._max_steer)
        self._print_status()
        return True

    def stop(self) -> None:
        self._speed = 0.0
        self._steer = 0.0
        self._publish_cmd()

    def _publish_cmd(self) -> None:
        msg = Twist()
        msg.linear.x = self._speed
        msg.angular.z = self._steer
        self._publisher.publish(msg)

    def _print_status(self) -> None:
        print(
            f'\rspeed={self._speed:+.2f} m/s  steer={self._steer:+.2f} rad  '
            f'speed_step={self._speed_step:.2f}  steer_step={self._steer_step:.2f}   ',
            end='',
            flush=True,
        )

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def _read_key(timeout: float = 0.05) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)

    print(HELP_TEXT)
    node._print_status()
    try:
        tty.setraw(sys.stdin.fileno())
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key()
            if key is not None:
                running = node.handle_key(key)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

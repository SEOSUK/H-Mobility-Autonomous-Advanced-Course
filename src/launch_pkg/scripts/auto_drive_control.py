#!/usr/bin/env python3

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import MotionCommand


class AutoDriveControlNode(Node):
    def __init__(self):
        super().__init__('auto_drive_control_node')

        self.sub_topic = self.declare_parameter(
            'sub_topic', 'auto_drive_command_raw').value
        self.pub_topic = self.declare_parameter(
            'pub_topic', 'topic_control_signal').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.publisher = self.create_publisher(
            MotionCommand, self.pub_topic, qos_profile)
        self.subscription = self.create_subscription(
            MotionCommand, self.sub_topic, self.command_callback, qos_profile)

        self.enabled = True
        self.stop_requested = False
        self.stdin_fd = None
        self.stdin_settings = None
        self.reader_thread = None

        self.publish_stop_command()
        self.get_logger().info(
            'Auto drive is armed. It will drive when valid lane-following commands arrive. Press c to stop.')
        self._start_keyboard_reader()

    def _start_keyboard_reader(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a TTY, so keyboard stop control is unavailable.')
            return

        self.stdin_fd = sys.stdin.fileno()
        self.stdin_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)
        self.reader_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self.reader_thread.start()

    def _keyboard_loop(self):
        while rclpy.ok() and not self.stop_requested:
            readable, _, _ = select.select([self.stdin_fd], [], [], 0.1)
            if not readable:
                continue

            key = sys.stdin.read(1)
            if key.lower() == 'c':
                if self.enabled:
                    self.enabled = False
                    self.publish_stop_command()
                    print('AUTO DRIVE STOPPED', flush=True)
                else:
                    self.publish_stop_command()

    def command_callback(self, msg):
        if not self.enabled:
            return
        self.publisher.publish(msg)

    def publish_stop_command(self):
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.publisher.publish(msg)

    def cleanup(self):
        self.stop_requested = True
        self.publish_stop_command()
        if self.stdin_fd is not None and self.stdin_settings is not None:
            termios.tcsetattr(
                self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
            self.stdin_fd = None
            self.stdin_settings = None


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nAUTO DRIVE STOPPED', flush=True)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Tier 2: Failsafe & Watchdog HIL Test (3-Element Protocol <rawX,rawY,rawW>)
"""

import sys
import time
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MockSerial:
    def __init__(self):
        self.packets = []
        self.is_open = True

    def write(self, data):
        self.packets.append(data.decode('utf-8'))
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False

class JetsonRobotBridgeTestNode(Node):
    def __init__(self, mock_serial):
        super().__init__('test_robot_bridge')
        self.ser = mock_serial
        self.system_enabled = True
        self.last_msg_time = self.get_clock().now()
        self._mock_idle = False
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_w = 0.0

    def enable_callback(self, msg: Bool):
        self.system_enabled = msg.data
        if not self.system_enabled:
            self.send_serial_packet(127, 127, 127)
            self.cur_vx = 0.0
            self.cur_vy = 0.0
            self.cur_w = 0.0

    def cmd_vel_callback(self, msg: Twist):
        self.last_msg_time = self.get_clock().now()
        self._mock_idle = False
        if not self.system_enabled:
            return

        self.cur_vx = msg.linear.x
        self.cur_vy = msg.linear.y
        self.cur_w = msg.angular.z

        forward_pwm = msg.linear.x * (255.0 / 1.2) / 2.0
        lateral_pwm = msg.linear.y * (255.0 / 1.2) / 2.0
        angular_pwm = msg.angular.z * (255.0 / 1.2) / 2.0 * 0.5

        if abs(forward_pwm) < 3: forward_pwm = 0.0
        if abs(lateral_pwm) < 3: lateral_pwm = 0.0
        if abs(angular_pwm) < 3: angular_pwm = 0.0

        val_x = max(0, min(255, int(lateral_pwm + 127)))
        val_y = max(0, min(255, int(forward_pwm + 127)))
        val_w = max(0, min(255, int(angular_pwm + 127)))

        self.send_serial_packet(val_x, val_y, val_w)

    def send_serial_packet(self, x, y, w):
        packet = f"<{x},{y},{w}>\n"
        if self.ser:
            self.ser.write(packet.encode('utf-8'))

    def trigger_watchdog(self, fake_elapsed_seconds):
        if fake_elapsed_seconds > 0.5:
            self.cur_vx = 0.0
            self.cur_vy = 0.0
            self.cur_w = 0.0
            self.send_serial_packet(127, 127, 127)
            self._mock_idle = True

class TestTier2Failsafe(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.mock_ser = MockSerial()
        self.node = JetsonRobotBridgeTestNode(self.mock_ser)

    def tearDown(self):
        self.node.destroy_node()

    def test_active_motion_dispatch(self):
        msg = Twist()
        msg.linear.x = 0.6
        self.node.cmd_vel_callback(msg)
        self.assertTrue(len(self.mock_ser.packets) > 0)
        latest = self.mock_ser.packets[-1]
        self.assertTrue(latest.startswith("<127,"))
        self.assertTrue(latest.endswith(",127>\n"))

    def test_watchdog_timeout_failsafe(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.node.cmd_vel_callback(msg)
        self.node.trigger_watchdog(0.6)
        self.assertEqual(self.mock_ser.packets[-1], "<127,127,127>\n", "Watchdog must enforce <127,127,127> on timeout")
        self.assertEqual(self.node.cur_vx, 0.0)

    def test_system_disable_estop(self):
        enable_msg = Bool()
        enable_msg.data = False
        self.node.enable_callback(enable_msg)
        self.assertEqual(self.mock_ser.packets[-1], "<127,127,127>\n")

if __name__ == '__main__':
    unittest.main(verbosity=2)

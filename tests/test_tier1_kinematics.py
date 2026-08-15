#!/usr/bin/env python3
"""
Tier 1: Kinematics & Protocol Unit Verification Test (3-Element Protocol <rawX,rawY,rawW>)
Conforming to /home/humanoid/roda/esp.ino
"""

import unittest
import re

MAX_LINEAR = 1.2
MAX_ANGULAR = 1.2
ANGULAR_GAIN = 0.5
LINEAR_DEADBAND_PWM = 3
ANGULAR_DEADBAND_PWM = 3

def compute_pwm(vx, vy, w):
    forward_pwm = vx * (255.0 / MAX_LINEAR) / 2.0
    lateral_pwm = vy * (255.0 / MAX_LINEAR) / 2.0
    angular_pwm = w * (255.0 / MAX_ANGULAR) / 2.0 * ANGULAR_GAIN

    if abs(forward_pwm) < LINEAR_DEADBAND_PWM: forward_pwm = 0.0
    if abs(lateral_pwm) < LINEAR_DEADBAND_PWM: lateral_pwm = 0.0
    if abs(angular_pwm) < ANGULAR_DEADBAND_PWM: angular_pwm = 0.0

    val_x = int(lateral_pwm + 127)
    val_y = int(forward_pwm + 127)
    val_w = int(angular_pwm + 127)

    val_x = max(0, min(255, val_x))
    val_y = max(0, min(255, val_y))
    val_w = max(0, min(255, val_w))

    packet = f"<{val_x},{val_y},{val_w}>\n"
    return (val_x, val_y, val_w), packet

def compute_esp_motors(vx, vy, w):
    # Kinematics in esp.ino:
    # M1 = Vx - Vy - W
    # M2 = Vx + Vy + W
    # M3 = Vx + Vy - W
    # M4 = Vx - Vy + W
    m1 = vx - vy - w
    m2 = vx + vy + w
    m3 = vx + vy - w
    m4 = vx - vy + w
    return m1, m2, m3, m4

class TestTier1Kinematics(unittest.TestCase):

    def test_neutral_zero(self):
        pwm, packet = compute_pwm(0.0, 0.0, 0.0)
        self.assertEqual(pwm, (127, 127, 127), "Neutral command must equal (127, 127, 127) so motor is diam total")
        self.assertEqual(packet, "<127,127,127>\n")

    def test_deadband_suppression(self):
        pwm, _ = compute_pwm(0.005, -0.005, 0.005)
        self.assertEqual(pwm, (127, 127, 127), "Small noise must snap to 127")

    def test_forward(self):
        pwm, packet = compute_pwm(1.2, 0.0, 0.0)
        self.assertEqual(pwm[0], 127)
        self.assertGreaterEqual(pwm[1], 254)
        self.assertEqual(pwm[2], 127)

    def test_strafe_right(self):
        pwm, packet = compute_pwm(0.0, 1.2, 0.0)
        self.assertGreaterEqual(pwm[0], 254)
        self.assertEqual(pwm[1], 127)
        self.assertEqual(pwm[2], 127)

    def test_rotation(self):
        pwm, packet = compute_pwm(0.0, 0.0, 0.8)
        self.assertEqual(pwm[0], 127)
        self.assertEqual(pwm[1], 127)
        self.assertGreater(pwm[2], 127)  # CCW > 127

    def test_packet_regex_format(self):
        test_cases = [
            (0.0, 0.0, 0.0),
            (0.5, -0.3, 0.8),
            (2.0, -2.0, -5.0),
            (-2.0, 2.0, 5.0)
        ]
        pattern = re.compile(r"^<\d{1,3},\d{1,3},\d{1,3}>\n$")
        for vx, vy, w in test_cases:
            _, packet = compute_pwm(vx, vy, w)
            self.assertTrue(pattern.match(packet), f"Packet {packet} failed regex check")

    def test_esp_kinematics_pure_forward(self):
        m1, m2, m3, m4 = compute_esp_motors(100.0, 0.0, 0.0)
        self.assertEqual(m1, 100.0)
        self.assertEqual(m2, 100.0)
        self.assertEqual(m3, 100.0)
        self.assertEqual(m4, 100.0)

if __name__ == '__main__':
    unittest.main(verbosity=2)

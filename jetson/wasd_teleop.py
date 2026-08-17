#!/usr/bin/env python3
"""
WASD Keyboard Teleop Node for Omnidirectional Mecanum Robot
Conforming with ROS 2 REP-103 standard and physical robot kinematics

Controls:
  [W] : Maju (Forward)           [Q] : Putar Kiri / CCW
  [S] : Mundur (Backward)        [E] : Putar Kanan / CW
  [A] : Strafe Kiri (Left)       [Space] / [X] : Stop / Rem
  [D] : Strafe Kanan (Right)     [R]/[F] : Kecepatan Linier (+/-)
                                 [T]/[G] : Kecepatan Sudut (+/-)
"""

import os
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
==========================================================
   🕹️  OMNIDIRECTIONAL ROBOT WASD TELEOPERATION          
==========================================================
        [ Q ]               [ W ]               [ E ]
   (Putar Kiri / CCW)       (MAJU)       (Putar Kanan / CW)

        [ A ]               [ S ]               [ D ]
    (STRAFE KIRI)          (MUNDUR)         (STRAFE KANAN)

   [Space] atau [X] : STOP / REM
   [R] / [F] : Ubah Kecepatan Linier (+/- 0.1 m/s)
   [T] / [G] : Ubah Kecepatan Putar (+/- 0.1 rad/s)
   [Ctrl+C]  : Keluar (Exit)
==========================================================
"""

def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_speed = 0.6   # m/s
        self.angular_speed = 0.8  # rad/s
        
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = float(self.vx)
        twist.linear.y = float(self.vy)
        twist.angular.z = float(self.w)
        self.pub.publish(twist)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = WasdTeleop()
    
    os.system('clear')
    print(BANNER)
    print(f"Status Awal: Kecepatan Linier = {node.linear_speed:.2f} m/s | Sudut = {node.angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)
            
            if key == '\x03':  # Ctrl+C
                break
            
            changed = False
            
            if key in ['w', 'W']:
                node.vx = node.linear_speed
                node.vy = 0.0
                node.w = 0.0
                changed = True
            elif key in ['s', 'S']:
                node.vx = -node.linear_speed
                node.vy = 0.0
                node.w = 0.0
                changed = True
            elif key in ['a', 'A']:
                node.vx = 0.0
                node.vy = node.linear_speed   # Strafe Kiri (vy > 0 -> rawX > 127)
                node.w = 0.0
                changed = True
            elif key in ['d', 'D']:
                node.vx = 0.0
                node.vy = -node.linear_speed  # Strafe Kanan (vy < 0 -> rawX < 127)
                node.w = 0.0
                changed = True
            elif key in ['q', 'Q']:
                node.vx = 0.0
                node.vy = 0.0
                node.w = node.angular_speed   # Putar Kiri / CCW (w > 0 -> rawW > 127)
                changed = True
            elif key in ['e', 'E']:
                node.vx = 0.0
                node.vy = 0.0
                node.w = -node.angular_speed  # Putar Kanan / CW (w < 0 -> rawW < 127)
                changed = True
            elif key in [' ', 'x', 'X', 'k', 'K']:
                node.vx = 0.0
                node.vy = 0.0
                node.w = 0.0
                changed = True
            elif key in ['r', 'R']:
                node.linear_speed = min(1.5, node.linear_speed + 0.1)
                print(f"\n⚡ Kecepatan Linier: {node.linear_speed:.2f} m/s")
            elif key in ['f', 'F']:
                node.linear_speed = max(0.1, node.linear_speed - 0.1)
                print(f"\n⚡ Kecepatan Linier: {node.linear_speed:.2f} m/s")
            elif key in ['t', 'T']:
                node.angular_speed = min(2.0, node.angular_speed + 0.1)
                print(f"\n🔄 Kecepatan Putar: {node.angular_speed:.2f} rad/s")
            elif key in ['g', 'G']:
                node.angular_speed = max(0.1, node.angular_speed - 0.1)
                print(f"\n🔄 Kecepatan Putar: {node.angular_speed:.2f} rad/s")

            if changed:
                node.publish_twist()
                state = "STOP"
                if node.vx > 0: state = "⬆️  MAJU (Forward)"
                elif node.vx < 0: state = "⬇️  MUNDUR (Backward)"
                elif node.vy > 0: state = "⬅️  GESER KIRI (Strafe Left)"
                elif node.vy < 0: state = "➡️  GESER KANAN (Strafe Right)"
                elif node.w > 0: state = "↺  PUTAR KIRI (CCW)"
                elif node.w < 0: state = "↻  PUTAR KANAN (CW)"
                
                print(f"\rStatus: {state:<25} | Vx={node.vx:+.2f} Vy={node.vy:+.2f} W={node.w:+.2f}   ", end="", flush=True)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.vx = 0.0
        node.vy = 0.0
        node.w = 0.0
        node.publish_twist()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nTeleop WASD ditutup.")

if __name__ == '__main__':
    main()

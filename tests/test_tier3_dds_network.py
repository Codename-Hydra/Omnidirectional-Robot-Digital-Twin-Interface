#!/usr/bin/env python3
"""
Tier 3: Network DDS Integration Test (Laptop <-> Jetson over ZeroTier)
"""

import time
import os
import sys
import paramiko
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from geometry_msgs.msg import Twist

class DDSIntegrationSubscriber(Node):
    def __init__(self):
        super().__init__('dds_integration_tester')
        self.battery_count = 0
        self.motor_count = 0
        self.vel_count = 0
        self.last_battery = None
        self.last_motors = None
        self.first_time = None
        self.last_time = None

        self.sub_bat = self.create_subscription(BatteryState, '/battery_status', self.bat_cb, 10)
        self.sub_mot = self.create_subscription(JointState, '/motor_status', self.mot_cb, 10)
        self.sub_vel = self.create_subscription(Twist, '/robot_velocity', self.vel_cb, 10)

    def bat_cb(self, msg: BatteryState):
        self.battery_count += 1
        self.last_battery = msg
        now = time.time()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now

    def mot_cb(self, msg: JointState):
        self.motor_count += 1
        self.last_motors = msg

    def vel_cb(self, msg: Twist):
        self.vel_count += 1

def run_jetson_bridge():
    ip = "10.101.143.175"
    username = "humanoid"
    password = "111111"
    
    print("[TEST] Connecting to Jetson to start background bridge...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=username, password=password, timeout=10)
    
    ssh.exec_command("pkill -f jetson_robot_bridge.py")
    time.sleep(1)

    cmd = "nohup bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && export CYCLONEDDS_URI=file:///home/humanoid/digital_twin_jetson/cyclonedds_jetson.xml && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_LOCALHOST_ONLY=0 && python3 /home/humanoid/digital_twin_jetson/jetson_robot_bridge.py' > /tmp/jetson_bridge.log 2>&1 &"
    ssh.exec_command(cmd)
    return ssh

def main():
    print("==================================================")
    print(" Tier 3: ZeroTier DDS Network Integration Test   ")
    print("==================================================")
    
    os.environ["ROS_DOMAIN_ID"] = "30"
    os.environ["CYCLONEDDS_URI"] = "file:///mnt/projects/Digital-Twin-Integrated/configs/cyclonedds_laptop.xml"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DISABLE_TYPE_HASH_CHECK"] = "1"

    ssh = run_jetson_bridge()
    time.sleep(2)

    rclpy.init()
    tester = DDSIntegrationSubscriber()
    
    print("[TEST] Listening for ROS 2 topics from Jetson for 6 seconds...")
    start = time.time()
    while time.time() - start < 6.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    print(f"\n[RESULTS]")
    print(f"  Battery Telemetry Packets Received: {tester.battery_count}")
    print(f"  Motor Telemetry Packets Received:   {tester.motor_count}")
    print(f"  Velocity Packets Received:          {tester.vel_count}")

    if tester.battery_count > 0 and tester.first_time and tester.last_time > tester.first_time:
        duration = tester.last_time - tester.first_time
        rate = (tester.battery_count - 1) / duration if duration > 0 else 0.0
        print(f"  Measured Publishing Rate:           {rate:.2f} Hz (Target: ~5.0 Hz)")
        print(f"  Sample Voltage:                     {tester.last_battery.voltage} V")
        print(f"  Sample Motor Joints:                {tester.last_motors.name}")
        success = True
    else:
        print("  ⚠️ No packets received within timeout.")
        success = False

    tester.destroy_node()
    rclpy.shutdown()
    
    ssh.exec_command("pkill -f jetson_robot_bridge.py")
    ssh.close()
    print("[TEST] Jetson bridge process stopped.")

    if not success:
        sys.exit(1)
    else:
        print("\n✅ Tier 3 Network DDS Test PASSED!")

if __name__ == '__main__':
    main()

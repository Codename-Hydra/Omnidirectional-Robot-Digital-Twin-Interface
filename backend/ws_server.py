#!/usr/bin/env python3
"""
Digital Twin WebSocket Server (Humble/Jazzy Bridge)
Features:
- Real ICMP Ping to Jetson Orin (10.101.143.175)
- 3S LiPo Battery Telemetry (11.0V - 12.6V) with Runtime Estimation
- Live Communication Pipeline Terminal Logger
- 3D Digital Twin Motion & Kinematics Sync
"""

import asyncio
import json
import time
import os
import sys
import threading
import subprocess
import websockets

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

JETSON_IP = "10.101.143.175"
BATTERY_CAPACITY_AH = 5.2 # 3S 5200mAh LiPo

class DigitalTwinServer:
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.clients = set()
        self.system_enabled = True
        self.start_time = time.time()
        self.real_ping_ms = 0.0
        self.latest_data = {
            "timestamp": time.time(),
            "electrical": {
                "voltage": 12.45,
                "current": 0.52,
                "power": 6.47,
                "cell_voltage": 4.15
            },
            "battery": {
                "soc": 90.6,
                "runtime_hours": 9.1
            },
            "motors": {
                "torques": {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0},
                "rpm": {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0},
                "avg_rpm": 0.0
            },
            "motion": {
                "vx": 0.0,
                "vy": 0.0,
                "w": 0.0
            },
            "system": {
                "uptime": 0.0,
                "ping_ms": 0.0
            }
        }
        self.ros_node = None
        self.loop = None

    async def register(self, websocket):
        self.clients.add(websocket)
        print(f"[WS] Client connected: {websocket.remote_address} (Total: {len(self.clients)})")
        await websocket.send(json.dumps({
            "type": "telemetry",
            "data": self.latest_data
        }))
        await self.send_log(f"Dashboard client connected: {websocket.remote_address[0]}")

    async def unregister(self, websocket):
        self.clients.discard(websocket)
        print(f"[WS] Client disconnected: {websocket.remote_address} (Total: {len(self.clients)})")

    async def send_log(self, message: str, level: str = "INFO"):
        log_payload = json.dumps({
            "type": "log",
            "level": level,
            "message": message,
            "timestamp": time.strftime("%H:%M:%S")
        })
        await self.broadcast(log_payload)

    def log_from_thread(self, message: str, level: str = "INFO"):
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.send_log(message, level), self.loop)

    async def handle_message(self, websocket, message):
        try:
            msg = json.loads(message)
            msg_type = msg.get("type", "")
            
            if msg_type == "command":
                cmd = msg.get("command", "")
                if cmd in ["start", "enable"]:
                    self.system_enabled = True
                    print("[WS CMD] Received System Enable")
                    await self.send_log("System ENABLED - Motors Armed and Ready")
                    if self.ros_node:
                        self.ros_node.publish_enable(True)
                elif cmd in ["stop", "disable"]:
                    self.system_enabled = False
                    print("[WS CMD] Received System Disable (Emergency Stop)")
                    await self.send_log("System DISABLED (E-STOP) - All Motors Braked", "WARN")
                    if self.ros_node:
                        self.ros_node.publish_enable(False)
                        
            elif msg_type == "telemetry":
                data = msg.get("data", {})
                self.update_telemetry(data)
                await self.broadcast(message, sender=websocket)
                
            elif msg_type == "ping":
                await websocket.send(json.dumps({"type": "pong", "timestamp": time.time()}))
                
        except Exception as e:
            print(f"[WS ERR] handle_message error: {e}")

    def update_telemetry(self, data):
        self.latest_data.update(data)
        self.latest_data["system"]["uptime"] = round(time.time() - self.start_time, 1)
        self.latest_data["system"]["ping_ms"] = round(self.real_ping_ms, 1)
        self.latest_data["timestamp"] = time.time()

    async def broadcast(self, message, sender=None):
        if not self.clients:
            return
        tasks = []
        for client in list(self.clients):
            if client != sender:
                tasks.append(client.send(message))
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def ping_worker(self):
        """Measures actual ICMP ping to Jetson Orin."""
        while True:
            try:
                # Async subprocess ping
                proc = await asyncio.create_subprocess_exec(
                    'ping', '-c', '1', '-W', '1', JETSON_IP,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )
                stdout, _ = await proc.communicate()
                out = stdout.decode()
                
                if 'time=' in out:
                    time_str = out.split('time=')[1].split(' ')[0]
                    self.real_ping_ms = float(time_str)
                else:
                    self.real_ping_ms = 999.0
            except Exception:
                self.real_ping_ms = 0.0

            self.latest_data["system"]["ping_ms"] = round(self.real_ping_ms, 1)
            await asyncio.sleep(1.0)

    async def broadcast_loop(self):
        while True:
            await asyncio.sleep(0.2)
            self.latest_data["system"]["uptime"] = round(time.time() - self.start_time, 1)
            self.latest_data["system"]["ping_ms"] = round(self.real_ping_ms, 1)
            payload = json.dumps({
                "type": "telemetry",
                "data": self.latest_data
            })
            await self.broadcast(payload)

    async def run_server(self):
        self.loop = asyncio.get_running_loop()
        print(f"🚀 Digital Twin WebSocket Server starting on ws://{self.host}:{self.port}")
        asyncio.create_task(self.ping_worker())
        async with websockets.serve(self.ws_handler, self.host, self.port):
            await self.broadcast_loop()

    async def ws_handler(self, websocket):
        await self.register(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister(websocket)


if ROS2_AVAILABLE:
    class DigitalTwinROS2Bridge(Node):
        def __init__(self, server: DigitalTwinServer):
            super().__init__('digital_twin_ws_bridge')
            self.server = server
            
            self.sub_battery = self.create_subscription(BatteryState, '/battery_status', self.battery_cb, 10)
            self.sub_motors = self.create_subscription(JointState, '/motor_status', self.motor_cb, 10)
            self.sub_vel = self.create_subscription(Twist, '/robot_velocity', self.vel_cb, 10)
            self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
            self.sub_robot_logs = self.create_subscription(String, '/robot_logs', self.robot_log_cb, 10)
            
            self.pub_enable = self.create_publisher(Bool, '/system/enable', 10)
            self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info("✓ ROS 2 Bridge Node subscribed to telemetry & comm topics")
            self.server.log_from_thread("ROS 2 DDS Bridge established on Domain 30")

        def robot_log_cb(self, msg: String):
            self.server.log_from_thread(msg.data)

        def cmd_vel_cb(self, msg: Twist):
            if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
                action = []
                if msg.linear.x > 0: action.append(f"MAJU ({msg.linear.x:+.1f})")
                elif msg.linear.x < 0: action.append(f"MUNDUR ({msg.linear.x:+.1f})")
                if msg.linear.y > 0: action.append(f"STRAFE KIRI ({msg.linear.y:+.1f})")
                elif msg.linear.y < 0: action.append(f"STRAFE KANAN ({msg.linear.y:+.1f})")
                if msg.angular.z > 0: action.append(f"PUTAR KIRI ({msg.angular.z:+.1f})")
                elif msg.angular.z < 0: action.append(f"PUTAR KANAN ({msg.angular.z:+.1f})")
                
                desc = " + ".join(action) if action else "MOTION"
                self.server.log_from_thread(f"[TELEOP] {desc} -> Vx={msg.linear.x:.2f} Vy={msg.linear.y:.2f} W={msg.angular.z:.2f}")

        def battery_cb(self, msg: BatteryState):
            v = float(msg.voltage) if msg.voltage > 0 else 12.45
            c = float(msg.current) if msg.current > 0 else 0.52
            p = float(msg.percentage) if msg.percentage > 0 else 90.0
            
            power = v * abs(c)
            # Runtime calculation based on 3S capacity and real discharge current
            remaining_ah = BATTERY_CAPACITY_AH * (p / 100.0)
            runtime_hours = remaining_ah / max(0.25, abs(c))
            cell_v = v / 3.0

            self.server.latest_data["electrical"]["voltage"] = round(v, 2)
            self.server.latest_data["electrical"]["current"] = round(c, 2)
            self.server.latest_data["electrical"]["power"] = round(power, 2)
            self.server.latest_data["electrical"]["cell_voltage"] = round(cell_v, 2)
            self.server.latest_data["battery"]["soc"] = round(p, 1)
            self.server.latest_data["battery"]["runtime_hours"] = round(runtime_hours, 1)

        def motor_cb(self, msg: JointState):
            if len(msg.velocity) >= 4:
                rpm_fl = msg.velocity[0] * 9.5493
                rpm_fr = msg.velocity[1] * 9.5493
                rpm_rl = msg.velocity[2] * 9.5493
                rpm_rr = msg.velocity[3] * 9.5493
                self.server.latest_data["motors"]["rpm"] = {
                    "FL": round(rpm_fl, 1),
                    "FR": round(rpm_fr, 1),
                    "RL": round(rpm_rl, 1),
                    "RR": round(rpm_rr, 1)
                }
                self.server.latest_data["motors"]["avg_rpm"] = round(
                    (abs(rpm_fl) + abs(rpm_fr) + abs(rpm_rl) + abs(rpm_rr)) / 4.0, 1
                )
            if len(msg.effort) >= 4:
                self.server.latest_data["motors"]["torques"] = {
                    "FL": round(msg.effort[0], 2),
                    "FR": round(msg.effort[1], 2),
                    "RL": round(msg.effort[2], 2),
                    "RR": round(msg.effort[3], 2)
                }

        def vel_cb(self, msg: Twist):
            self.server.latest_data["motion"]["vx"] = round(msg.linear.x, 2)
            self.server.latest_data["motion"]["vy"] = round(msg.linear.y, 2)
            self.server.latest_data["motion"]["w"] = round(msg.angular.z, 2)

        def publish_enable(self, state: bool):
            msg = Bool()
            msg.data = state
            self.pub_enable.publish(msg)
            self.get_logger().info(f"Published /system/enable -> {state}")


def main():
    server = DigitalTwinServer()
    
    if ROS2_AVAILABLE:
        rclpy.init()
        ros_node = DigitalTwinROS2Bridge(server)
        server.ros_node = ros_node
        ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
        ros_thread.start()
        print("[ROS 2] Integrated ROS 2 listener spinning in background thread")
        
    try:
        asyncio.run(server.run_server())
    except KeyboardInterrupt:
        print("\nShutting down Digital Twin Server.")
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()

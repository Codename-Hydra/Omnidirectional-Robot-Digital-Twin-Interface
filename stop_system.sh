#!/bin/bash
################################################################################
# Stop All Digital Twin Processes (Laptop & Jetson)
################################################################################

JETSON_IP="10.101.143.175"
JETSON_USER="humanoid"
JETSON_PASS="111111"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}   STOPPING ALL DIGITAL TWIN PROCESSES (LAPTOP & JETSON)   ${NC}"
echo -e "${BLUE}======================================================${NC}"

# 1. Stop Processes on Laptop
echo -e "\n${YELLOW}[1/2] Stopping processes on Laptop...${NC}"
pkill -9 -f "ws_server.py" 2>/dev/null
fuser -k 8765/tcp 2>/dev/null
pkill -9 -f "vite" 2>/dev/null
fuser -k 5173/tcp 2>/dev/null
pkill -9 -f "wasd_teleop.py" 2>/dev/null
pkill -9 -f "teleop_twist_keyboard" 2>/dev/null
pkill -9 -f "launch_system.sh" 2>/dev/null
echo -e "${GREEN}✓ Laptop processes stopped.${NC}"

# 2. Stop Processes on Jetson Orin via SSH
echo -e "\n${YELLOW}[2/2] Stopping processes on Jetson Orin (${JETSON_IP})...${NC}"

python3 - << 'PYEOF'
import paramiko

ip = "10.101.143.175"
username = "humanoid"
password = "111111"

try:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=username, password=password, timeout=5)
    
    cmd = """
    python3 -c "import serial; ser=serial.Serial('/dev/ttyUSB0',115200,timeout=0.1); ser.write(b'<127,127,127>\\n'); ser.close()" 2>/dev/null || true
    pkill -9 -f "jetson_robot_bridge.py" 2>/dev/null || true
    pkill -9 -f "start_jetson_bridge.sh" 2>/dev/null || true
    pkill -9 -f "wasd_teleop.py" 2>/dev/null || true
    pkill -9 -f "teleop.sh" 2>/dev/null || true
    sudo pkill -9 -f "usb_bridge.py" 2>/dev/null || true
    """
    
    stdin, stdout, stderr = ssh.exec_command(cmd)
    stdout.channel.recv_exit_status()
    print("  ✓ Sent neutral brake <127,127,127> to ESP32")
    print("  ✓ Terminated jetson_robot_bridge.py")
    print("  ✓ Terminated usb_bridge.py")
    print("  ✓ Terminated wasd_teleop.py")
    ssh.close()
    print("\033[0;32m✓ Jetson processes stopped successfully.\033[0m")
except Exception as e:
    print(f"\033[0;31m✗ Could not connect to Jetson ({e})\033[0m")
PYEOF

echo -e "\n${GREEN}======================================================${NC}"
echo -e "${GREEN}   ALL SERVICES AND CONTROLLERS HAVE BEEN STOPPED   ${NC}"
echo -e "${GREEN}======================================================${NC}"

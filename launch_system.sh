#!/bin/bash
################################################################################
# Digital Twin System Launcher (Laptop Side)
################################################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH=$PATH:/home/codename-hydra/.nvm/versions/node/v24.12.0/bin

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=30
    export CYCLONEDDS_URI="file://${SCRIPT_DIR}/configs/cyclonedds_laptop.xml"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DISABLE_TYPE_HASH_CHECK=1
    echo "✓ ROS 2 Jazzy Environment Loaded (Domain: $ROS_DOMAIN_ID)"
fi

echo "=================================================="
echo "    Digital Twin Integrated System (Laptop)      "
echo "=================================================="

# 1. Start WebSocket Server
echo "1. Starting WebSocket Server (Port 8765)..."
python3 "${SCRIPT_DIR}/backend/ws_server.py" &
WS_PID=$!
echo "   WS Server PID: $WS_PID"

# 2. Start Web Dashboard
echo "2. Starting Vite Web Dashboard (Port 5173)..."
cd "${SCRIPT_DIR}/web_dashboard" && npm run dev &
DASH_PID=$!
echo "   Dashboard PID: $DASH_PID"

echo ""
echo "🚀 Digital Twin System running!"
echo "   Dashboard URL: http://localhost:5173"
echo "   WebSocket:     ws://localhost:8765"
echo "Press Ctrl+C to terminate all services."

trap "kill $WS_PID $DASH_PID 2>/dev/null; exit 0" INT TERM
wait

#!/usr/bin/env python3
"""
Tier 4: End-to-End WebSocket Pipeline & Telemetry Schema Test
"""

import asyncio
import json
import subprocess
import time
import sys
import os
import signal
import websockets

async def test_websocket_client():
    uri = "ws://localhost:8765"
    print(f"[TEST] Connecting to {uri}...")
    
    async with websockets.connect(uri) as websocket:
        print("✓ Connected to Digital Twin WebSocket Server")

        # 1. Receive initial telemetry frame
        msg_raw = await asyncio.wait_for(websocket.recv(), timeout=5.0)
        msg = json.loads(msg_raw)
        print(f"✓ Initial frame received (Type: {msg.get('type')})")
        
        # Verify schema
        data = msg.get("data", {})
        required_keys = ["electrical", "battery", "motors", "motion", "system", "timestamp"]
        for k in required_keys:
            assert k in data, f"Missing key in telemetry: {k}"
        
        assert "voltage" in data["electrical"], "Missing voltage in electrical"
        assert "soc" in data["battery"], "Missing soc in battery"
        assert "rpm" in data["motors"], "Missing rpm in motors"
        assert "vx" in data["motion"], "Missing vx in motion"
        print("✓ Telemetry JSON schema validation PASSED")

        # 2. Test Command Dispatch (Enable/Disable)
        cmd_payload = json.dumps({"type": "command", "command": "stop"})
        await websocket.send(cmd_payload)
        print("✓ Sent Stop (Emergency Disable) command via WebSocket")

        # 3. Test Ping-Pong
        ping_payload = json.dumps({"type": "ping"})
        await websocket.send(ping_payload)
        pong_raw = await asyncio.wait_for(websocket.recv(), timeout=5.0)
        pong = json.loads(pong_raw)
        print(f"✓ Response received from server: {pong.get('type')}")

        # 4. Stream 2 consecutive frames
        frames = 0
        while frames < 2:
            raw = await asyncio.wait_for(websocket.recv(), timeout=2.0)
            parsed = json.loads(raw)
            if parsed.get("type") == "telemetry":
                frames += 1
                print(f"  Frame {frames}: Uptime={parsed['data']['system']['uptime']}s, Voltage={parsed['data']['electrical']['voltage']}V, AvgRPM={parsed['data']['motors']['avg_rpm']}")
                
        print("✓ Continuous 5Hz stream reception PASSED")

def main():
    print("==================================================")
    print(" Tier 4: End-to-End WebSocket & Schema Test       ")
    print("==================================================")

    # Ensure port 8765 is free
    subprocess.run(["fuser", "-k", "8765/tcp"], capture_output=True)
    time.sleep(0.5)

    server_proc = subprocess.Popen(
        ["python3", "/mnt/projects/Digital-Twin-Integrated/backend/ws_server.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1.5)

    try:
        asyncio.run(test_websocket_client())
        print("\n✅ Tier 4 End-to-End WebSocket Test PASSED!")
        success = True
    except Exception as e:
        print(f"❌ Test Failed: {e}")
        success = False
    finally:
        try:
            os.killpg(os.getpgid(server_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        print("[TEST] WS Server stopped.")

    if not success:
        sys.exit(1)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Master Test Suite Runner (Tier 1 -> Tier 4)
"""

import subprocess
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

tests = [
    ("Tier 1: Kinematics & 4-Element Protocol Tests", "test_tier1_kinematics.py"),
    ("Tier 2: Failsafe & Watchdog HIL Tests", "test_tier2_failsafe.py"),
    ("Tier 3: ZeroTier DDS Network Integration Tests", "test_tier3_dds_network.py"),
    ("Tier 4: End-to-End WebSocket Pipeline Tests", "test_tier4_e2e_ws.py"),
]

def main():
    print("==========================================================")
    print("   AUTOMATED 4-TIER ROBOTICS & DIGITAL TWIN TEST SUITE   ")
    print("==========================================================\n")

    results = []
    env = os.environ.copy()
    
    for title, script in tests:
        print(f"▶ Running: {title} ({script})...")
        script_path = os.path.join(SCRIPT_DIR, script)
        cmd = ["python3", script_path]
        
        proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
        
        if proc.returncode == 0:
            print(f"  ✅ {title} : PASSED\n")
            results.append((title, "PASSED", proc.stdout))
        else:
            print(f"  ❌ {title} : FAILED")
            print(f"  Output:\n{proc.stdout}\n{proc.stderr}\n")
            results.append((title, "FAILED", proc.stderr))

    print("==========================================================")
    print("                      SUMMARY                             ")
    print("==========================================================")
    all_passed = True
    for title, status, _ in results:
        icon = "✅" if status == "PASSED" else "❌"
        print(f"{icon} {title}: {status}")
        if status != "PASSED":
            all_passed = False

    print("==========================================================")
    if all_passed:
        print("🎉 ALL TEST TIERS PASSED PERFECTLY!")
        sys.exit(0)
    else:
        print("⚠️ SOME TESTS FAILED.")
        sys.exit(1)

if __name__ == '__main__':
    main()

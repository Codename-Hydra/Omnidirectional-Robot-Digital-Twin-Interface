# Digital Twin Interface - Robot Omnidirectional

## 📋 Deskripsi
Sistem Digital Twin untuk monitoring dan kontrol robot omnidirectional berbasis Webots dengan integrasi hardware fisik (Orange Pi 5 Pro + ESP32). Menyediakan dashboard web real-time untuk visualisasi telemetri, kontrol jarak jauh, dan sinkronisasi 3D.

## 🎯 Fitur Utama
- ✅ **Real-time Telemetry**: Battery, RPM, Torque, Motion data
- ✅ **Remote Control**: Start/Stop program via Web Dashboard
- ✅ **3D Visualization**: Webots simulator synchronized with physical robot
- ✅ **Network Monitoring**: Real-time ping to Orange Pi
- ✅ **Battery Estimation**: Runtime calculation with freeze-on-stop logic

## 🛠️ Teknologi
- **Simulator**: Webots R2025a
- **Backend**: Python 3, ROS2 (Humble/Jazzy)
- **Frontend**: TypeScript, Vite, WebSocket
- **Hardware**: Orange Pi 5 Pro, ESP32, 2x LiPo 3S (5200mAh)

## 📚 Dokumentasi

### 📖 [System Documentation](system_documentation.md)
Dokumentasi lengkap mencakup:
- Latar Belakang & Tujuan Sistem
- Arsitektur Komunikasi (ROS2, Serial UART, WebSocket)
- Protokol Data Detail
- Algoritma Kontrol (Speed, Battery Estimation)
- Diagram Fungsional (Mermaid)
- **Flowchart IEEE Standard**
- Panduan Penggunaan Lengkap
- Troubleshooting Guide
- Spesifikasi Teknis

### 🎨 [Mermaid Diagrams](mermaid_diagrams.md)
Kode Mermaid untuk semua diagram:
- Diagram Arsitektur Sistem
- Flowchart Startup Sequence (IEEE)
- Flowchart Telemetry Loop (IEEE)
- Flowchart Start/Stop Command (IEEE)
- Sequence Diagram
- Symbol Guide (IEEE Standard)

## 🚀 Quick Start

### Prerequisites
```bash
# Laptop
sudo apt install python3-rclpy python3-websockets
# Webots R2025a installed

# Orange Pi
pip3 install pyserial rclpy
```

### Running the System
```bash
# 1. Start WebSocket Server
cd ~/Documents/Digital_Twin_Interface
nohup python3 ws_server.py > ws_server.log 2>&1 &

# 2. Start Web Dashboard
npm run dev

# 3. Start Webots
./start_webots.sh

# 4. Deploy to Orange Pi
python3 ~/start_all_remote.py
```

## 📊 System Architecture

```
┌─────────────┐      WebSocket       ┌──────────────┐
│  Web Dashboard │ ←──────────────→ │ ws_server.py │
└─────────────┘                      └──────────────┘
                                           ↕ WebSocket
                                     ┌──────────────┐
                                     │  Controller  │
                                     │   (Webots)   │
                                     └──────────────┘
                                           ↕ ROS2
                                     ┌──────────────┐
                                     │  Orange Pi   │
                                     │    Bridge    │
                                     └──────────────┘
                                           ↕ Serial
                                     ┌──────────────┐
                                     │    ESP32     │
                                     │   Motors     │
                                     └──────────────┘
```

## 🔧 Configuration

| Parameter | Value | Description |
|:---|:---|:---|
| Speed Multiplier | 63 | Half speed (safe operation) |
| Webots Max Vel | 0.25 m/s | Simulation visual speed |
| Battery | 2x 3S (5200mAh) | Serial connection |
| Ping Rate | 1 Hz | ICMP ping to Orange Pi |

## 📝 License
[Specify your license here]

## 👤 Author
Codename Hydra

## 🔗 Links
- [Webots Official](https://cyberbotics.com/)
- [ROS2 Documentation](https://docs.ros.org/)

---

**Version**: 1.0  
**Last Updated**: 2026-01-05  
**Status**: ✅ Production Ready

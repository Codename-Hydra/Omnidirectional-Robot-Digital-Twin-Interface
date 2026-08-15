# Omnidirectional Robot Digital Twin & Control Interface

Sistem Digital Twin terintegrasi untuk monitoring, telemetri *real-time*, dan kontrol robot omnidirectional (Mecanum/Omniwheel) dengan integrasi hardware fisik (NVIDIA Jetson Orin / SBC + ESP32) serta Web Dashboard 3D interaktif.

---

## 🎯 Fitur Utama

- 🟢 **Real-Time Telemetry & Sync**: Sinkronisasi data baterai, arus, daya, RPM roda, torsi, dan kecepatan linier/sudut.
- ⚡ **3S LiPo Battery Management**: Kalibrasi rentang tegangan 3S ($11.0\text{V}$ Cutoff $\leftrightarrow 12.6\text{V}$ Full Charge), pembacaan tegangan per-cell ($3.66\text{V} - 4.20\text{V}$), serta estimasi sisa waktu pemakaian (*remaining runtime*).
- 🌐 **Real Network Ping Monitor**: Pengukuran latensi bolak-balik (*ICMP round-trip latency*) nyata antara Laptop dan Robot melalui ZeroTier DDS Network.
- 🕹️ **WASD Keyboard & Gamepad Teleoperation**: Kontrol presisi gerak holonomik: Maju (`W`), Mundur (`S`), Geser Kiri (`A`), Geser Kanan (`D`), Putar Kiri (`Q`), Putar Kanan (`E`), dan Rem (`Spasi`).
- 📟 **Live Communication Terminal**: Console feed langsung pada dashboard web untuk memantau status komunikasi ROS 2, transmisi paket serial, dan perintah gerak secara *live*.
- 🛡️ **Fail-Safe & Safety Watchdogs**: Rem otomatis dalam $0.5\text{ detik}$ jika komunikasi terputus untuk mencegah putaran liar (*failsafe brake*).

---

## 🏗️ Arsitektur Sistem

```
┌────────────────────────┐         WebSocket          ┌────────────────────────┐
│  Web Dashboard (Vite)  │ ◄────────────────────────► │ ws_server.py (Gateway) │
│  - 3D Model View       │        (Port 8765)         │ - ROS 2 Jazzy Bridge   │
│  - Real Ping & Bat 3S  │                            │ - Real Ping Worker     │
│  - Live Terminal Feed  │                            │ - Telemetry Aggregator │
└────────────────────────┘                            └───────────┬────────────┘
                                                                  │
                                                        ROS 2 DDS (Domain 30)
                                                        CycloneDDS / ZeroTier
                                                                  │
                                                      ┌───────────▼────────────┐
                                                      │ jetson_robot_bridge.py │
                                                      │ - Jetson Orin Nano     │
                                                      │ - 5Hz Telemetry Pub    │
                                                      └───────────┬────────────┘
                                                                  │ Serial UART
                                                            <rawX,rawY,rawW>
                                                                  │ (115200 Baud)
                                                      ┌───────────▼────────────┐
                                                      │  ESP32 Motor Driver    │
                                                      │  - Unified Dual-Mode   │
                                                      │  - 4x BTS7960 Motors   │
                                                      └────────────────────────┘
```

---

## 📂 Struktur Direktori

```
├── backend/
│   └── ws_server.py                  # WebSocket ROS 2 Gateway & Ping Monitor
├── configs/
│   ├── cyclonedds_laptop.xml         # Konfigurasi DDS Cyclone untuk Laptop
│   └── cyclonedds_jetson.xml         # Konfigurasi DDS Cyclone untuk Jetson
├── firmware/
│   └── ESP_Unified_DualMode/
│       └── ESP_Unified_DualMode.ino  # Firmware ESP32 Unified Dual-Protocol
├── jetson/
│   ├── jetson_robot_bridge.py        # ROS 2 Humble bridge ke Serial ESP32
│   ├── usb_bridge.py                 # Driver user-space PL2303 USB UART
│   ├── start_jetson_bridge.sh        # Startup script Jetson
│   └── stop_robot.sh                 # Emergency stop script Jetson
├── tests/
│   ├── run_all_tests.py              # Master 4-tier automated test suite
│   ├── test_tier1_kinematics.py      # Test Tier 1: Kinematika & Protokol
│   ├── test_tier2_failsafe.py        # Test Tier 2: Failsafe & Watchdog
│   ├── test_tier3_dds_network.py     # Test Tier 3: ZeroTier DDS Network
│   └── test_tier4_e2e_ws.py          # Test Tier 4: End-to-End WebSocket
├── web_dashboard/                    # Web Dashboard Frontend (TypeScript/Vite)
├── launch_system.sh                  # 1-Click Launch Script (Laptop)
├── stop_system.sh                    # 1-Click Stop Script (Laptop & Jetson)
├── teleop_wasd.sh                    # Keyboard Teleop Script
└── wasd_teleop.py                    # Node ROS 2 WASD Keyboard Publisher
```

---

## 🚀 Panduan Penggunaan

### 1. Menjalankan Robot di Jetson Orin
```bash
ssh humanoid@10.101.143.175
cd ~/digital_twin_jetson
./start_jetson_bridge.sh
```

### 2. Menjalankan Dashboard di Laptop
```bash
# Menjalankan WebSocket Server & Web Dashboard
./launch_system.sh
```
Buka browser dan kunjungi: **`http://localhost:5173`**

### 3. Mengontrol Robot via Keyboard
```bash
./teleop_wasd.sh
```

### 4. Menghentikan Seluruh Sistem Sekaligus
```bash
./stop_system.sh
```

---

## 🧪 Pengujian Otomatis

Jalankan test suite 4-tier untuk memverifikasi integritas seluruh pipeline:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI=file://$(pwd)/configs/cyclonedds_laptop.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_TYPE_HASH_CHECK=1
python3 tests/run_all_tests.py
```

---

## 👤 Author
**Codename Hydra**  
*Integrated Digital Twin & Omnidirectional Robotics System*

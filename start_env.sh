#!/bin/bash

# 0. 環境大掃除
pkill -f px4
pkill -f gz
pkill -f MicroXRCEAgent
pkill -f ruby
echo "🧹 環境清理完畢..."
sleep 2

# === 1. 啟動 DDS Agents (關鍵修正：3台飛機要3個Agent) ===
echo "📡 啟動通訊中繼站 (DDS Agents)..."
# Agent 1 (Leader): Port 8888
gnome-terminal --tab --title="Agent Leader" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
# Agent 2 (Left): Port 8890
gnome-terminal --tab --title="Agent Left" -- bash -c "MicroXRCEAgent udp4 -p 8890; exec bash"
# Agent 3 (Right): Port 8892
gnome-terminal --tab --title="Agent Right" -- bash -c "MicroXRCEAgent udp4 -p 8892; exec bash"

sleep 3

# 2. 啟動隊長機 (Leader)
echo "🚁 啟動隊長機 (Leader)..."
gnome-terminal --tab --title="Leader (0)" -- bash -c "cd ~/src/PX4-Autopilot && \
PX4_GZ_MODEL_POSE='0,0,0.2,0,0,0' \
make px4_sitl gz_x500_lidar_2d; exec bash"

echo "⏳ 等待 Gazebo 載入 (15秒)..."
sleep 15

# 3. 啟動僚機
echo "🚁 啟動左僚機..."
gnome-terminal --tab --title="Left Wing (1)" -- bash -c "./run_wingman.sh 1 '-2,2,0.2,0,0,0'; exec bash"

sleep 5

echo "🚁 啟動右僚機..."
gnome-terminal --tab --title="Right Wing (2)" -- bash -c "./run_wingman.sh 2 '-2,-2,0.2,0,0,0'; exec bash"

# 4. 啟動 Bridge (這部分你已經修好了，保持原樣)
echo "🌉 啟動感測器橋樑..."
gnome-terminal --tab --title="Super Bridge" -- bash -c "export GZ_VERSION=harmonic && \
ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
--ros-args \
-r /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan \
-r /world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan:=/scan_1 \
-r /world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan:=/scan_2; \
exec bash"

echo "🚀 環境啟動完成！"

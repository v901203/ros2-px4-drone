#!/bin/bash

# 0. 環境大掃除 (解決 Topic 類型衝突的關鍵)
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby
# 🚀 徹底重置 ROS 2 後台緩存，防止 [Pose, TFMessage] 類型衝突
ros2 daemon stop
echo "🧹 環境與 ROS 2 守護進程清理完畢..."
sleep 2
ros2 daemon start

# === 1. 啟動 DDS Agents (三台飛機對應三個連接埠) ===
echo "📡 啟動通訊中繼站 (DDS Agents)..."
gnome-terminal --tab --title="Agent Leader" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
gnome-terminal --tab --title="Agent Left" -- bash -c "MicroXRCEAgent udp4 -p 8890; exec bash"
gnome-terminal --tab --title="Agent Right" -- bash -c "MicroXRCEAgent udp4 -p 8892; exec bash"

sleep 3

# 2. 啟動隊長機 (Leader)
echo "🚀 啟動隊長機 (Leader)..."
gnome-terminal --tab --title="Leader (0)" -- bash -c "cd ~/src/PX4-Autopilot && \
export PX4_GZ_WORLD=default && \
PX4_GZ_MODEL_POSE='0,0,0.2,0,0,0' \
make px4_sitl gz_x500_lidar_2d; exec bash"

echo "⏳ 等待 Gazebo 載入 (20秒，確保物理引擎穩定)..."
sleep 20

# 3. 啟動僚機 (呼叫 run_wingman.sh)
echo "🚀 啟動左僚機..."
gnome-terminal --tab --title="Left Wing (1)" -- bash -c "./run_wingman.sh 1 '-2,2,0.2,0,0,0'; exec bash"
sleep 5
echo "🚀 啟動右僚機..."
gnome-terminal --tab --title="Right Wing (2)" -- bash -c "./run_wingman.sh 2 '-2,-2,0.2,0,0,0'; exec bash"

# 4. 啟動 Bridge (統一使用 TFMessage 格式，適配 Pose_V 向量數據)
echo "🌉 啟動上帝視角數據橋樑..."
gnome-terminal --tab --title="Super Bridge" -- bash -c "export GZ_VERSION=harmonic && \
ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/model/x500_lidar_2d_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
/model/x500_lidar_2d_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
/model/x500_lidar_2d_2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
--ros-args \
-r /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan \
-r /world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan:=/scan_1 \
-r /world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan:=/scan_2 \
-r /model/x500_lidar_2d_0/pose:=/gt_pose_0 \
-r /model/x500_lidar_2d_1/pose:=/gt_pose_1 \
-r /model/x500_lidar_2d_2/pose:=/gt_pose_2; \
exec bash"

echo "✅ 環境啟動完成！"

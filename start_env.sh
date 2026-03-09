#!/bin/bash

# 0. 環境大掃除
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f mono
ros2 daemon stop
echo "豐富環境清理完畢..."
sleep 2
ros2 daemon start

# === 重要：座標與路徑設定 ===
export PX4_HOME_LAT=24.8485
export PX4_HOME_LON=121.0150
export PX4_HOME_ALT=0.0
export MISSION_PLANNER_PATH="$HOME/MissionPlanner/MissionPlanner.exe"

# === ⭐️ Mission Planner 自動安裝與檢查 ===
if [ ! -f "$MISSION_PLANNER_PATH" ]; then
    echo "⚠️  找不到 Mission Planner，嘗試執行安裝..."
    if [ -f "./install_mp.sh" ]; then
        chmod +x install_mp.sh
        ./install_mp.sh
    fi
fi

# 1. 啟動 DDS Agents
echo "📡 啟動 Micro-XRCE-DDS Agents..."
gnome-terminal --tab --title="Agent Leader" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
gnome-terminal --tab --title="Agent Left" -- bash -c "MicroXRCEAgent udp4 -p 8890; exec bash"
gnome-terminal --tab --title="Agent Right" -- bash -c "MicroXRCEAgent udp4 -p 8892; exec bash"

sleep 2

# 2. 啟動隊長機 (Leader, ID=1)
# 預設 MAVLink UDP Port: 14550
echo "🚀 啟動隊長機 (Leader)..."
gnome-terminal --tab --title="Leader (ID 1)" -- bash -c "cd ~/src/PX4-Autopilot && \
PX4_SYS_ID=1 make px4_sitl gz_x500_lidar_2d; exec bash"

sleep 15

# 3. 啟動僚機 (透過修改過的 run_wingman.sh)
# Gazebo ENU: x=東, y=北。 PX4 NED offset (-2,2) → Gazebo (y_ned=2, x_ned=-2) = (2,-2,0.2)
# 僚機 1 (ID 2) -> MAVLink Port 14552
echo "🚀 啟動左僚機..."
gnome-terminal --tab --title="Left Wing (ID 2)" -- bash -c "./run_wingman.sh 1 '2,-2,0.2,0,0,0'; exec bash"

sleep 5

# PX4 NED offset (-2,-2) → Gazebo (-2,-2,0.2) ← 原本就正確
# 僚機 2 (ID 3) -> MAVLink Port 14553
echo "🚀 啟動右僚機..."
gnome-terminal --tab --title="Right Wing (ID 3)" -- bash -c "./run_wingman.sh 2 '-2,-2,0.2,0,0,0'; exec bash"

# 4. 啟動 Mission Planner
if [ -f "$MISSION_PLANNER_PATH" ]; then
    echo "🗺️  正在啟動 Mission Planner (請連線至 UDP 14550)..."
    gnome-terminal --tab --title="Mission Planner" -- bash -c "mono '$MISSION_PLANNER_PATH'; exec bash"
fi

echo "✅ 所有系統已就緒！"

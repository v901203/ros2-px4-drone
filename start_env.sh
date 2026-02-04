#!/bin/bash

# 0. 環境大掃除
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby
ros2 daemon stop
echo "🧹 環境清理完畢..."
sleep 2
ros2 daemon start

# === 重要：統一所有無人機的座標原點 (對齊北方與座標系) ===
export PX4_HOME_LAT=24.8485
export PX4_HOME_LON=121.0150
export PX4_HOME_ALT=0.0

# === 自動部署修改過的模型檔案 (確保 Git 內的設定生效) ===
if [ -f "${PWD}/models/x500_lidar_2d/model.sdf" ]; then
    echo "📂 檢測到客製化模型，正在覆寫 PX4 設定..."
    mkdir -p ~/src/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d
    cp "${PWD}/models/x500_lidar_2d/model.sdf" ~/src/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf
fi

# 1. 啟動 DDS Agents (確認 Port 號分別為 8888, 8890, 8892)
echo "📡 啟動通訊中繼站 (DDS Agents)..."
gnome-terminal --tab --title="Agent Leader" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
gnome-terminal --tab --title="Agent Left" -- bash -c "MicroXRCEAgent udp4 -p 8890; exec bash"
gnome-terminal --tab --title="Agent Right" -- bash -c "MicroXRCEAgent udp4 -p 8892; exec bash"

sleep 3

# 2. 啟動隊長機 (Leader, ID=1)
echo "🚀 啟動隊長機 (Leader)..."
gnome-terminal --tab --title="Leader (ID 1)" -- bash -c "cd ~/src/PX4-Autopilot && \
export PX4_GZ_WORLD=default && \
export PX4_PARAM_COM_ARM_WO_GPS=1 && \
export PX4_PARAM_NAV_RCL_ACT=0 && \
export PX4_PARAM_NAV_DLL_ACT=0 && \
export PX4_PARAM_COM_RCL_EXCEPT=4 && \
export PX4_PARAM_BAT_LOW_THR=0.0 && \
export PX4_PARAM_BAT_CRIT_THR=0.0 && \
export PX4_PARAM_BAT_EMERGEN_THR=0.0 && \
PX4_GZ_MODEL_POSE='0,0,0.2,0,0,0' \
PX4_SYS_ID=1 make px4_sitl gz_x500_lidar_2d; exec bash"

echo "⏳ 等待 Leader 載入 (20秒)..."
sleep 20

# 3. 啟動僚機
# 請確認 run_wingman.sh 內部使用的 Port 號與上面的 Agent 對應
# 1 號僚機 (SYS_ID 2) -> 連接 8890
echo "🚀 啟動左僚機 (物理位置 -2, 2)..."
gnome-terminal --tab --title="Left Wing (ID 2)" -- bash -c "./run_wingman.sh 1 '-2,2,0.2,0,0,0'; exec bash"

sleep 5

# 2 號僚機 (SYS_ID 3) -> 連接 8892
echo "🚀 啟動右僚機 (物理位置 -2, -2)..."
gnome-terminal --tab --title="Right Wing (ID 3)" -- bash -c "./run_wingman.sh 2 '-2,-2,0.2,0,0,0'; exec bash"

echo "✅ 腳本啟動完成！"

#!/bin/bash

# 接收主腳本傳來的參數：$1 是 ID (1或2), $2 是座標
ID=$1
POSE=$2

# === ⭐️ 關鍵路徑設定 (根據你的截圖寫死的) ===
# 這是你電腦裡 PX4 的真實位置
PX4_DIR="$HOME/src/PX4-Autopilot"
MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"

# 告訴 Gazebo：模型跟世界地圖就在這裡！(兩個變數都設，買個保險)
export GZ_SIM_RESOURCE_PATH="$MODELS_DIR:$WORLDS_DIR"
export IGN_GAZEBO_RESOURCE_PATH="$MODELS_DIR:$WORLDS_DIR"

# 設定 PX4 參數
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL=x500_lidar_2d    # 指定要有雷達的模型
export PX4_SIM_MODEL=x500_lidar_2d   # 雙重保險
export PX4_GZ_MODEL_POSE="$POSE"

# === 關閉解鎖檢查與安全防護 ===
export PX4_PARAM_COM_ARM_WO_GPS=1
export PX4_PARAM_NAV_RCL_ACT=0
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_COM_RCL_EXCEPT=4
export PX4_PARAM_BAT_LOW_THR=0.0
export PX4_PARAM_BAT_CRIT_THR=0.0
export PX4_PARAM_BAT_EMERGEN_THR=0.0

# 進入目錄並執行
cd "$PX4_DIR"
echo "🚀 正在啟動僚機 $ID..."
echo "📍 模型路徑: $MODELS_DIR"
echo "📍 座標: $POSE"

# 執行 PX4
./build/px4_sitl_default/bin/px4 -i $ID

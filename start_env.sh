#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1. 啟動 DDS Agent
echo "正在啟動 DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
PID_AGENT=$!
sleep 2

# 2. 啟動 PX4 與 Gazebo (使用已經驗證過的指令)
echo "正在啟動 PX4 SITL..."
cd ~/src/PX4-Autopilot

# --- 修改重點：直接使用指定機型的指令，穩定性最高 ---
make px4_sitl gz_x500

# 當你關閉模擬器視窗後，腳本才會執行到這裡，殺死 Agent
kill $PID_AGENT

cat << 'EOF' > ~/Desktop/drone/start_env.sh
#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- [新增功能] 啟動前強制清理環境 ---
echo "🧹 正在執行強制清理 (Killing old processes)..."
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby
sleep 1 # 休息一秒確保清理乾淨
echo "✅ 環境已清理乾淨！"
# ----------------------------------

# 1. 啟動 DDS Agent
echo "🚀 正在啟動 DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
PID_AGENT=$!
sleep 2

# 2. 啟動 PX4 與 Gazebo
echo "🚁 正在啟動 PX4 SITL (Gazebo Harmonic)..."
cd ~/src/PX4-Autopilot
make px4_sitl gz_x500

# 當模擬器關閉後，殺死 Agent
kill $PID_AGENT
EOF

# 給予執行權限
chmod +x ~/Desktop/drone/start_env.sh

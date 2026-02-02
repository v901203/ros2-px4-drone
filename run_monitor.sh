#!/bin/bash

# 載入 ROS 2 環境
source /opt/ros/humble/setup.bash
if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
else
    echo "⚠️  找不到 ros2_ws/install/setup.bash，請嘗試在 ros2_ws 下執行 colcon build"
fi

# 1. 啟動 ROS GZ Bridge (後台執行以釋放 Terminal)
# 改用個別 Pose 轉發，確保能區分是哪台無人機
echo "🌉 正在啟動 ROS GZ Bridge (個別 Topic 模式)..."
ros2 run ros_gz_bridge parameter_bridge \
    /model/x500_lidar_2d_0/pose@geometry_msgs/msg/Pose@gz.msgs.Pose \
    /model/x500_lidar_2d_1/pose@geometry_msgs/msg/Pose@gz.msgs.Pose \
    /model/x500_lidar_2d_2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose \
    --ros-args \
    -r /model/x500_lidar_2d_0/pose:=/gz_pose_0 \
    -r /model/x500_lidar_2d_1/pose:=/gz_pose_1 \
    -r /model/x500_lidar_2d_2/pose:=/gz_pose_2 &
BRIDGE_PID=$!
sleep 2

# 2. 檢查 Bridge 是否有資料 (短暫 Echo 第一台)
echo "📡 檢查 Gazebo 數據流 (/gz_pose_0)..."
timeout 2s ros2 topic echo /gz_pose_0 | head -n 3
if [ $? -eq 124 ]; then
    echo "✅ 數據流正常 (timeout 是預期的)"
else
    echo "⚠️  警告：可能沒有收到 Gazebo 數據，請確認模擬器是否開啟"
fi

# 3. 啟動 Python 診斷程式
echo "📊 啟動診斷程式 (按 Ctrl+C 結束)..."
python3 ros2_ws/src/compare.py

# 結束時清理後台的 Bridge
kill $BRIDGE_PID

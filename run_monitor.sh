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
    /world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V \
    /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
    /world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
    /world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
    --ros-args \
    -r /world/default/dynamic_pose/info:=/gz_dynamic_pose \
    -r /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/drone_0/scan \
    -r /world/default/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan:=/drone_1/scan \
    -r /world/default/model/x500_lidar_2d_2/link/link/sensor/lidar_2d_v2/scan:=/drone_2/scan &
BRIDGE_PID=$!
sleep 2

# 2. 檢查 Bridge 是否有資料 (短暫 Echo 第一台)
echo "📡 檢查 Gazebo 動態姿態流 (/gz_dynamic_pose)..."
timeout 2s ros2 topic echo /gz_dynamic_pose | head -n 3
if [ $? -eq 124 ]; then
    echo "✅ 動態姿態流正常 (timeout 是預期的)"
else
    echo "⚠️  警告：可能沒有收到 Gazebo 姿態數據，請確認模擬器是否開啟"
fi

echo "📡 檢查雷達數據流 (/drone_0/scan)..."
timeout 2s ros2 topic echo /drone_0/scan | head -n 3
if [ $? -eq 124 ]; then
    echo "✅ 雷達數據流正常 (timeout 是預期的)"
else
    echo "⚠️  警告：可能沒有收到雷達數據，請確認 lidar topic 名稱是否正確"
fi

# 3. 啟動 Python 診斷程式
echo "📊 啟動診斷程式 (按 Ctrl+C 結束)..."
python3 ros2_ws/src/compare.py

# 結束時清理後台的 Bridge
kill $BRIDGE_PID

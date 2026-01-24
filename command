軟體組件 (Component),	軟體名稱 (Software),	版本 (Version),			用途與角色 (Purpose)
OS,			Ubuntu LTS,		22.04 (Jammy Jellyfish),	基礎作業系統 (全英文環境)
Robot Middleware,	ROS 2,			Humble Hawksbill,		機器人通訊核心，負責處理節點溝通與演算法 (大腦)
Simulator,		Gazebo,			Harmonic (v8),			新一代物理模擬器，提供擬真的物理環境與傳感器數據
Autopilot Firmware,	PX4-Autopilot,		Main (v1.14+),			飛行控制固件，負責姿態解算與馬達混控 (小腦)
Comm Bridge,		Micro-XRCE-DDS,		Latest (Agent),			負責將 PX4 的 uORB 訊息翻譯成 ROS 2 的 DDS 訊息
Simulation Bridge,	ros_gz_bridge,		Harmonic,			負責 Gazebo (模擬器) 與 ROS 2 之間的影像/雷達數據傳輸
Programming,		Python,			3.10,				"撰寫上層控制演算法 (Offboard Control, RL)"

------------------------------------------------------------------------------------------------------------------------------------------

路徑 (Path),				說明 (Description)
~/Desktop/drone/,			專案主目錄 (所有實驗的起點)
~/Desktop/drone/start_env.sh,		一鍵啟動腳本 (修正版：直接呼叫 gz_x500 避免閃退)
~/Desktop/drone/ros2_ws/,		ROS 2 工作空間 (放置你的 Python 控制程式碼)
~/Desktop/drone/ros2_ws/src/test_flight.py, 自動飛行程式 (修正版：包含強制解鎖 param2=21196)
~/src/PX4-Autopilot/,			PX4 韌體原始碼 (已編譯過的引擎核心)

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟一：啟動模擬環境 (Simulation)
cd ~/Desktop/drone
./start_env.sh

# 【重要步驟】關閉安全檢查 (只需做一次，除非重啟模擬器)
# 當你看見 pxh> 提示符號後，請依序輸入以下兩行指令並按 Enter：
# pxh> param set NAV_RCL_ACT 0   (說明：沒有遙控器也不會觸發降落保護)
# pxh> param set NAV_DLL_ACT 0   (說明：沒有地面站也不會觸發降落保護)
commander arm -f 或是嘗試這個繞過大多數的安全檢查（模擬用）
------------------------------------------------------------------------------------------------------------------------------------------

# 步驟二：執行控制程式 (Control Node)
開啟 第二個 終端機 (Terminal)。

# 1. 載入 ROS 2 環境變數 (必做，否則會找不到指令)
source /opt/ros/humble/setup.bash
source ~/Desktop/drone/ros2_ws/install/local_setup.bash

# 2. 執行 Python 控制程式
python3 ~/Desktop/drone/ros2_ws/src/test_flight.py

------------------------------------------------------------------------------------------------------------------------------------------

# 附錄：除錯指令 (Troubleshooting)
# 如果 Gazebo 卡住或開不起來，請執行以下指令強制清理：
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby

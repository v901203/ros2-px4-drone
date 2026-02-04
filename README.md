軟體組件 (Component),	軟體名稱 (Software),	版本 (Version),			用途與角色 (Purpose)
OS,			Ubuntu LTS,		22.04 (Jammy Jellyfish),	基礎作業系統
Robot Middleware,	ROS 2,			Humble Hawksbill,		機器人通訊核心 (大腦)
Simulator,		Gazebo,			Harmonic (v8),			物理模擬器 (Sim)
Autopilot Firmware,	PX4-Autopilot,		Main (v1.14+),			飛控固件 (小腦)
Comm Bridge,		Micro-XRCE-DDS,		Latest (3 Agents),		負責 PX4 <-> ROS 2 的通訊 (Port 8888, 8890, 8892)
Bridge Package,     ros-humble-ros-gzharmonic,  Harmonic Bridge,        	負責 Lidar 數據轉換 (Gazebo -> ROS 2)

------------------------------------------------------------------------------------------------------------------------------------------

路徑 (Path),				說明 (Description)
~/Desktop/drone/,			專案主目錄
~/Desktop/drone/start_env.sh,		環境啟動腳本 (自動開啟 3 個 Agent + 3 台無人機 + Bridge)
~/Desktop/drone/run_wingman.sh,		僚機啟動腳本 (被 start_env.sh 呼叫)
~/Desktop/drone/formation_flight.py,	編隊飛行主程式 (包含暴力解鎖功能，無需手動輸入指令)
https://github.com/v901203/ros2-px4-drone,	GitHub 遠端倉庫

------------------------------------------------------------------------------------------------------------------------------------------

# 程式功能詳解 (Scripts Detail)

| 檔案名稱 (File) | 所在路徑 (Path) | 功能說明 (Function) |
|---|---|---|
| `run_monitor.sh` | Project Root | **診斷啟動腳本**。自動建立 ROS-GZ Bridge 連線，並執行 `compare.py` 監控位置誤差。 |
| `compare.py` | `ros2_ws/src/` | **座標診斷節點**。同時訂閱 PX4 的估測位置 (Local Position) 與 Gazebo 的真實位置 (Ground Truth)，計算並顯示兩者誤差。 |
| `test_flight.py` | `ros2_ws/src/` | **起飛測試節點**。強制解鎖所有無人機並控制其起飛至指定高度，用於測試通訊鏈路與 Offboard 控制權。 |
| `test_circular.py` | `ros2_ws/src/` | **圓形編隊節點**。控制三台無人機 (Leader + 2 Wingman) 進行協同圓形繞飛，驗證多機控制邏輯。 |

------------------------------------------------------------------------------------------------------------------------------------------
修改了x500_lidar_2d模型設定，強迫 Gazebo 的物理引擎在每一幀計算後，將無人機的真實座標發布到 /model/x500_lidar_2d_x/pose 這個 Topic 上。
code ~/src/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf
加入<plugin
      filename="gz-sim-pose-publisher-system"
      name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>true</static_publisher>
      <static_update_frequency>10</static_update_frequency>
    </plugin>
------------------------------------------------------------------------------------------------------------------------------------------

# 步驟一：啟動多機模擬環境
cd ~/Desktop/drone
./start_env.sh

# 【檢查點】
# 啟動後，請輸入以下指令確認 3 台飛機都已連上 ROS 2：
# ros2 topic list | grep vehicle_status
# 預期看到：/fmu/..., /px4_1/..., /px4_2/... 三組開頭的 Topic 全部出現才算成功。

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟二：執行編隊飛行 (Python控制節點)
# 確保你已經將 Python 腳本更新為包含 `_v1` Topic 名稱與 `211965` 解鎖金鑰的版本
source /opt/ros/humble/setup.bash
# 如果你有編譯 workspace，請 source install/setup.bash，如果是直接執行腳本則不需要
python3 formation_flight.py

# 程式啟動後：
# 1. 會自動嘗試將所有飛機切換為 Offboard 模式。
# 2. 如果飛機未解鎖，會自動發送 Force Arm (211965) 指令。
# 3. 飛機將起飛並執行正方形編隊巡航。

------------------------------------------------------------------------------------------------------------------------------------------
# 步驟三：監聽真實位置 (Ground Truth) 數據流分析
# 用來計算編隊誤差 (Reward Function 基礎)
# 檢測 Gazebo 底層原始數據：
gz topic -e -t /model/x500_lidar_2d_0/pose

# 檢測 ROS 2 轉換後數據 (TF 格式)：
ros2 topic echo /gt_pose_0
------------------------------------------------------------------------------------------------------------------------------------------

# 常用 PX4 Console 指令 (Manual Commands)
# 如果自動腳本失效，可以直接在 PX4 Terminal 輸入以下指令：

# 1. 關閉所有安全檢查
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set BAT_LOW_THR 0.0

# 2. 強制解鎖並起飛
commander arm -f
commander takeoff

------------------------------------------------------------------------------------------------------------------------------------------

# 常見問題除錯 (Troubleshooting)

# Q1: 終端機顯示 "Package 'ros_gz_bridge' not found" ?
# A1: 請安裝正確版本的 Bridge:
sudo apt install ros-humble-ros-gzharmonic

# Q2: 執行 Python 腳本後飛機沒反應，一直顯示 "嘗試切換..." ?
# A2: 檢查 Topic 名稱是否匹配。新版 PX4 可能使用 `vehicle_status_v1`。
# 使用 `ros2 topic list` 確認名稱後，修改 Python 腳本中的訂閱字串。

# Q3: 如何強制清理卡住的環境 ?
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby

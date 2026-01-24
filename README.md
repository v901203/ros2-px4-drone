軟體組件 (Component),	軟體名稱 (Software),	版本 (Version),			用途與角色 (Purpose)
OS,			Ubuntu LTS,		22.04 (Jammy Jellyfish),	基礎作業系統 (全英文環境)
Robot Middleware,	ROS 2,			Humble Hawksbill,		機器人通訊核心，負責處理節點溝通與演算法 (大腦)
Simulator,		Gazebo,			Harmonic (v8),			新一代物理模擬器，提供擬真的物理環境與傳感器數據
Autopilot Firmware,	PX4-Autopilot,		Main (v1.14+),			飛行控制固件，負責姿態解算與馬達混控 (小腦)
Comm Bridge,		Micro-XRCE-DDS,		Latest (Agent),			負責將 PX4 的 uORB 訊息翻譯成 ROS 2 的 DDS 訊息
Version Control,	Git,			2.34.1,				代碼版本控制與備份工具 (使用 GitHub)

------------------------------------------------------------------------------------------------------------------------------------------

路徑 (Path),				說明 (Description)
~/Desktop/drone/,			專案主目錄 (所有實驗的起點)
~/Desktop/drone/start_env.sh,		一鍵啟動腳本 (修正版：直接呼叫 gz_x500 避免閃退)
~/Desktop/drone/ros2_ws/,		ROS 2 工作空間 (放置你的 Python 控制程式碼，不需要 venv)
~/Desktop/drone/ros2_ws/src/test_flight.py, 自動飛行程式 (修正版：包含強制解鎖 param2=21196)
https://github.com/v901203/ros2-px4-drone,	GitHub 遠端倉庫網址

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟一：啟動模擬環境 (Simulation)
cd ~/Desktop/drone
./start_env.sh

# 【重要設定】關閉安全檢查 (只需做一次，除非重啟模擬器)
# 當你看見 pxh> 提示符號後，請依序輸入以下兩行指令並按 Enter：
# pxh> param set NAV_RCL_ACT 0   (說明：沒有遙控器也不會觸發降落保護)
# pxh> param set NAV_DLL_ACT 0   (說明：沒有地面站也不會觸發降落保護)

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟二：監聽「上帝視角」真值 (Ground Truth)
# 開啟 第二個 終端機，用來比對 EKF 估計值與真實位置
gz topic -e -t /model/x500_0/odometry_with_covariance

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟三：執行控制程式 (Control Node)
# 開啟 第三個 終端機，執行自動飛行腳本
source /opt/ros/humble/setup.bash
source ~/Desktop/drone/ros2_ws/install/local_setup.bash
python3 ~/Desktop/drone/ros2_ws/src/test_flight.py

------------------------------------------------------------------------------------------------------------------------------------------

# 步驟四：版本控制與備份 (Git Workflow)
# 每天開發結束，或功能修改成功後執行

# 1. 檢查狀態 (確認修改了哪些檔案)
git status

# 2. 加入所有變更
git add .

# 3. 提交存檔 (寫下清楚的修改紀錄)
git commit -m "Update: 描述你改了什麼功能"

# 4. 上傳到 GitHub (密碼請輸入 Personal Access Token)
git push
# 帳號: v901203
# 密碼: ghp_xxxxxxxxxxxx (你的 Classic Token)

------------------------------------------------------------------------------------------------------------------------------------------

# 附錄：除錯指令 (Troubleshooting)
# 如果 Gazebo 卡住或開不起來，請執行以下指令強制清理：
pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent
pkill -9 -f ruby

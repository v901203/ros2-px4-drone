#!/bin/bash
# 自動化安裝 Mission Planner (LInux)
MP_DIR="$HOME/MissionPlanner"
MP_ZIP="MissionPlanner-latest.zip"
MP_URL="https://firmware.ardupilot.org/Tools/MissionPlanner/$MP_ZIP"

echo "📦 正在檢查 Mission Planner..."

if [ ! -d "$MP_DIR" ]; then
    echo "⬇️  正在下載 Mission Planner..."
    mkdir -p "$MP_DIR"
    wget -O "$MP_DIR/$MP_ZIP" "$MP_URL"
    
    echo "📂 正在解壓縮..."
    unzip -o "$MP_DIR/$MP_ZIP" -d "$MP_DIR"
    rm "$MP_DIR/$MP_ZIP"
    
    echo "✅ Mission Planner 下載完成！路徑: $MP_DIR"
else
    echo "✅ Mission Planner 已存在 ($MP_DIR)"
fi

echo "🔍 檢查執行環境 (Mono)..."
if command -v mono &> /dev/null; then
    echo "✅ Mono 環境已安裝！"
else
    echo "⚠️  未偵測到 Mono 環境！請執行以下指令安裝："
    echo "   sudo apt update && sudo apt install -y mono-complete"
    echo "installing dependencies..."
    # Attempt install if user has sudo sans password (unlikely but try)
    sudo apt update && sudo apt install -y mono-complete
fi


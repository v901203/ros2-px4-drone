#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import subprocess
import threading
import re
import math
import time

class CoordDiagnosticNode(Node):
    def __init__(self):
        super().__init__('coord_diagnostic_node')

        self.spawn_points = [
            [0.0, 0.0],    # Leader (Instance 0)
            [-2.0, 2.0],   # Left Wing (Instance 1)
            [-2.0, -2.0]   # Right Wing (Instance 2)
        ]
        
        # Namespaces mapping
        # Leader (Inst 0) -> /fmu
        # Left (Inst 1) -> /px4_1/fmu
        # Right (Inst 2) -> /px4_2/fmu
        self.namespaces = ["/fmu", "/px4_1/fmu", "/px4_2/fmu"]

        self.vehicle_positions = [None, None, None]
        self.gz_pos = [None, None, None]
        self.running = True

        # === PX4 Subscribers ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for i in range(3):
            def callback(msg, sys_id=i):
                # NED 座標系: Use .x, .y, .z instead of .position[]
                self.vehicle_positions[sys_id] = [msg.x, msg.y, msg.z]
            
            # Using correct namespace and suffix
            topic = f'{self.namespaces[i]}/out/vehicle_local_position_v1'
            self.create_subscription(VehicleLocalPosition, topic, callback, qos_profile)

        # === Gazebo CLI Reader Thread ===
        self.gz_lock = threading.Lock()
        self.gz_thread = threading.Thread(target=self.read_gz_stream)
        self.gz_thread.daemon = True
        self.gz_thread.start()

        self.timer = self.create_timer(1.0, self.log_comparison)
        print("🔍 診斷節點已啟動 (Corrected Property Access)...")

    def read_gz_stream(self):
        """獨立線程讀取 Gazebo CLI 輸出，避免緩衝區阻塞"""
        cmd = ['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info']
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        re_name = re.compile(r'name:\s*"([^"]+)"')
        re_x = re.compile(r'x:\s*([\d\.-]+)')
        re_y = re.compile(r'y:\s*([\d\.-]+)')
        re_z = re.compile(r'z:\s*([\d\.-]+)')
        
        current_model = None
        temp_pos = {}
        
        # 使用 iter 不斷讀取
        for line in iter(process.stdout.readline, ''):
            if not self.running: break
            line = line.strip()
            
            # 檢測模型名稱
            m_name = re_name.search(line)
            if m_name:
                current_model = m_name.group(1)
                temp_pos = {} 
                continue
            
            if current_model and "x500_lidar_2d" in current_model:
                mx = re_x.search(line)
                my = re_y.search(line)
                mz = re_z.search(line)
                
                if mx: temp_pos['x'] = float(mx.group(1))
                if my: temp_pos['y'] = float(my.group(1))
                if mz: temp_pos['z'] = float(mz.group(1))
                
                # 當讀到 "}" 且數據完整時更新
                if '}' in line and 'x' in temp_pos and 'y' in temp_pos:
                    idx = -1
                    # Gazebo models logic:
                    # x500_lidar_2d_0 (Instance 0) -> Leader (i=0)
                    # x500_lidar_2d_1 (Instance 1) -> Left Wing (i=1)
                    # x500_lidar_2d_2 (Instance 2) -> Right Wing (i=2)
                    if 'x500_lidar_2d_0' in current_model: idx = 0
                    elif 'x500_lidar_2d_1' in current_model: idx = 1
                    elif 'x500_lidar_2d_2' in current_model: idx = 2
                    
                    if idx != -1:
                        with self.gz_lock:
                            # 儲存最新的 Gazebo 真實座標
                            self.gz_pos[idx] = [temp_pos['x'], temp_pos['y'], temp_pos.get('z', 0)]
                    
                    current_model = None
                    temp_pos = {}

    def log_comparison(self):
        print("\n" + "="*80)
        print(f"⏰ 時間: {self.get_clock().now().to_msg().sec}s")
        print("="*80)
        
        for i in range(3):
            p = self.vehicle_positions[i]
            
            with self.gz_lock:
                g = self.gz_pos[i]
            
            role = ['Leader (Inst 0, /fmu)', 'Left Wing (Inst 1, /px4_1)', 'Right Wing (Inst 2, /px4_2)'][i]
            
            if p is None:
                print(f"[{role}]: ⏳ 等待 PX4 數據 (Topic: {self.namespaces[i]}/out/vehicle_local_position_v1)...")
                continue
            
            # === 座標系轉換 ===
            # PX4 (Local NED) -> World (ENU)
            # NED: X=North, Y=East
            # ENU: X=East, Y=North
            # 
            # 所以 PX4 East (p[1]) 對應 World X
            # 所以 PX4 North (p[0]) 對應 World Y
            # p[0] is x(North), p[1] is y(East), p[2] is z(Down)
            
            px4_local_n = p[0]
            px4_local_e = p[1]
            px4_local_d = p[2]
            
            spawn_world_x = self.spawn_points[i][0] # East
            spawn_world_y = self.spawn_points[i][1] # North
            
            # 推算世界座標 (ENU)
            # World_Est_X = PX4_East + Spawn_East
            # World_Est_Y = PX4_North + Spawn_North
            world_x_est = px4_local_e + spawn_world_x
            world_y_est = px4_local_n + spawn_world_y
            world_z_est = -px4_local_d
            
            print(f"\n--- [{role}] ---")
            print(f"  📍 物理出生點 (ENU): x={spawn_world_x:.2f}, y={spawn_world_y:.2f}")
            print(f"  🎯 PX4 局部 (NED):   N={px4_local_n:.2f}, E={px4_local_e:.2f}, D={px4_local_d:.2f}")
            print(f"  🌍 推算世界 (ENU):   x={world_x_est:.2f}, y={world_y_est:.2f}, z={world_z_est:.2f}")
            
            if g is not None:
                error_x = world_x_est - g[0]
                error_y = world_y_est - g[1]
                error_z = world_z_est - g[2]
                error_dist = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                print(f"  ✅ Gazebo 真實座標: x={g[0]:.2f}, y={g[1]:.2f}, z={g[2]:.2f}")
                print(f"  ⚖️  座標誤差:       dx={error_x:.2f}, dy={error_y:.2f}, dz={error_z:.2f}, 距離={error_dist:.2f}m")
                
                if abs(error_dist) > 1.0:
                    print(f"  ⚠️  誤差 {error_dist:.2f}m > 1.0m！")
                else:
                    print(f"  ✅ 誤差在容許範圍內")
            else:
                print(f"  ⏳ Gazebo 數據:     等待中... (檢查 gz topic 指令)")
        
        print("="*80)

def main(args=None):
    rclpy.init(args=args)
    node = CoordDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

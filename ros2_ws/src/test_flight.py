import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from sensor_msgs.msg import LaserScan
import time
import math

class ForceArmCommander(Node):
    def __init__(self):
        super().__init__('force_arm_commander')

        self.drone_configs = [
            {'ns': '/fmu', 'sys_id': 1, 'role': 'Leader'},
            {'ns': '/px4_1/fmu', 'sys_id': 2, 'role': 'Left Wing'},
            {'ns': '/px4_2/fmu', 'sys_id': 3, 'role': 'Right Wing'}
        ]
        
        # PX4 NED 座標系: x=北(North), y=東(East), z=下(Down)
        # Body frame: x=前方, y=右方 (NED 右手系)
        # yaw=0 朝北, yaw=π/2 朝東, yaw=π 朝南, yaw=-π/2 朝西
        #
        # V 型隊形 (從上方俯瞰, Leader 在尖端):
        #   off_x=-2: 在 Leader 後方 2m
        #   off_y=+2: 在 Leader 右方 2m (NED body y+ = 右)
        #   off_y=-2: 在 Leader 左方 2m
        
        self.offsets = [
            {'x': 0.0, 'y': 0.0, 'z': 0.0},      # Leader (隊形尖端)
            {'x': -2.0, 'y': 2.0, 'z': 0.0},     # 右後方 (body y+)
            {'x': -2.0, 'y': -2.0, 'z': 0.0}     # 左後方 (body y-)
        ]

        self.takeoff_altitude = -5.0
        self.takeoff_time = 5.0

        self.vehicle_status = [{'nav_state': 0, 'arming_state': 0} for _ in range(3)]
        self.initial_positions = [None] * 3
        self.current_positions = [None] * 3  # 新增：儲存即時位置
        self.lidar_data = [None] * 3
        self.last_lidar_warn_time = [0.0] * 3
        self.start_time = time.time()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pubs_offboard = []
        self.pubs_trajectory = []
        self.pubs_command = []
        self.subs_status = []
        self.subs_local_pos = []
        self.subs_lidar = []

        for i, config in enumerate(self.drone_configs):
            ns = config['ns']
            self.pubs_offboard.append(self.create_publisher(OffboardControlMode, f'{ns}/in/offboard_control_mode', qos_profile))
            self.pubs_trajectory.append(self.create_publisher(TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', qos_profile))
            self.pubs_command.append(self.create_publisher(VehicleCommand, f'{ns}/in/vehicle_command', qos_profile))
            self.subs_status.append(self.create_subscription(
                VehicleStatus, f'{ns}/out/vehicle_status_v1', 
                lambda msg, idx=i: self.status_callback(msg, idx), qos_profile))
            self.subs_local_pos.append(self.create_subscription(
                VehicleLocalPosition, f'{ns}/out/vehicle_local_position_v1',
                lambda msg, idx=i: self.local_pos_callback(msg, idx), qos_profile))
            self.subs_lidar.append(self.create_subscription(
                LaserScan, f'/drone_{i}/scan',
                lambda msg, idx=i: self.lidar_callback(msg, idx), 10))

        self.timer = self.create_timer(0.05, self.timer_callback) 
        self.get_logger().info("🔥 暴力解鎖指揮官已啟動！準備強制起飛...")

    def status_callback(self, msg, idx):
        self.vehicle_status[idx]['nav_state'] = msg.nav_state
        self.vehicle_status[idx]['arming_state'] = msg.arming_state

    def local_pos_callback(self, msg, idx):
        # 更新即時位置與航向
        # VehicleLocalPosition msg 包含 heading (yaw)
        self.current_positions[idx] = [msg.x, msg.y, msg.z, msg.heading]

        if self.initial_positions[idx] is None:
            self.initial_positions[idx] = [msg.x, msg.y, msg.z]
            self.get_logger().info(f"📍 [{self.drone_configs[idx]['role']}] 初始位置鎖定: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def lidar_callback(self, msg, idx):
        self.lidar_data[idx] = msg

        if not msg.ranges:
            return

        front_index = len(msg.ranges) // 2
        front_dist = msg.ranges[front_index]
        if math.isfinite(front_dist) and front_dist < 1.0:
            now = time.time()
            # 限制警告頻率，避免日誌刷屏
            if now - self.last_lidar_warn_time[idx] > 1.0:
                self.last_lidar_warn_time[idx] = now
                self.get_logger().warn(
                    f"⚠️ 無人機 {idx} 警告：前方 {front_dist:.2f}m 有障礙物！"
                )

    def arm_and_set_offboard(self, i):
        self.publish_offboard_heartbeat(i)  # 持續發送 Offboard 心跳

        if self.vehicle_status[i]['nav_state'] != 14:
            self.send_command(i, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        
        if self.vehicle_status[i]['arming_state'] != 2:
            self.send_command(i, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.start_time
        
        takeoff_phase = elapsed < self.takeoff_time
        cruise_time = elapsed - self.takeoff_time

        # === 新增：飛行狀態顯示邏輯 ===
        if not takeoff_phase:
            # 週期參數需與 calculate_drone_target 保持一致
            fly_time, turn_time = 8.0, 4.0
            phase_time = fly_time + turn_time
            t = cruise_time % (phase_time * 4)
            phase = int(t // phase_time)
            
            # 每 2 秒印一次狀態，方便監控
            if int(elapsed) % 2 == 0 and getattr(self, 'last_printed_sec', -1) != int(elapsed):
                self.last_printed_sec = int(elapsed)
                
                # PX4 NED: A邊=x+ 朝北, B邊=y+ 朝東, C邊=x- 朝南, D邊=y- 朝西
                sides = ['A (西邊)', 'B (北邊)', 'C (東邊)', 'D (南邊)']
                dirs  = ['北 (North) ↑', '東 (East) →', '南 (South) ↓', '西 (West) ←']
                shapes = ['/\\', '>', '\\/', '<']
                
                t_phase = t % phase_time
                if t_phase < fly_time:
                    state_str = f"直線巡航"
                else:
                    target_phase = (phase + 1) % 4
                    state_str = f"轉彎中 (-> {dirs[target_phase]})"

                self.get_logger().info(
                    f"\n[飛行監控] --------------------------\n"
                    f"  當前路徑邊條 : {sides[phase]} (邊 {['A','B','C','D'][phase]})\n"
                    f"  V型隊伍朝向  : {dirs[phase]}\n"
                    f"  預期隊形形狀 : {shapes[phase]}\n"
                    f"  目前動作     : {state_str}\n"
                    f"------------------------------------"
                )

        if takeoff_phase:
            progress = min(elapsed / self.takeoff_time, 1.0)
            
        # 移除了等待 Leader 定位數據的邏輯，因為現在是獨立計算
        for i in range(len(self.drone_configs)):
             self.publish_offboard_heartbeat(i)

        for i, config in enumerate(self.drone_configs):
            self.arm_and_set_offboard(i)

            if self.initial_positions[i] is None:
                continue
            
            init_x, init_y, init_z = self.initial_positions[i]

            if takeoff_phase:
                current_target_z = self.takeoff_altitude * progress 
                self.publish_trajectory(i, [init_x, init_y, current_target_z], self.initial_positions[i][3] if len(self.initial_positions[i]) > 3 else 0.0)

            else:
                cruise_time = elapsed - self.takeoff_time
                
                # 計算該無人機在全域座標系下的絕對目標位置
                # 這裡不再依賴 Leader 的實時位置，而是根據虛擬 Leader 的路徑加上各自的偏移量計算
                target_pos, target_yaw = self.calculate_drone_target(cruise_time, i)
                
                # 將全域目標轉換為本地指令 (Local Command = Global Target - Initial Offset)
                off_x = self.initial_positions[i][0]
                off_y = self.initial_positions[i][1]
                
                # 注意：這裡假設 initial_positions 是無人機啟動時的本地座標原點在全域下的位置
                # 但實際上 PX4 的 Local Position 是相對於開機點 (0,0,0) 的
                # 如果我們要讓它們飛絕對路徑，我们需要假設它們起飛的位置就是它們該在的位置（或者修正這個假設）
                # 在這個腳本中，initial_positions 只是用來記錄 "起飛點"，並沒有真正校準到統一座標系
                # 但為了簡化，我們假設三台無人機的 Local Frame 都是對齊的，只是原點不同
                # 為了讓它們保持隊形，我們直接發送 "相對於起飛點" 的位移
                
                # REVISION: 
                # Leader 的路徑是 (0,0) -> (10,0) ... 這是相對於 Leader 起飛點的
                # 僚機的路徑應該也是相對於僚機自己的起飛點嗎？
                # 如果是這樣，那僚機也會飛一個 10x10 的正方形，只是起點不同。
                # 但我們希望僚機保持隊形。
                # 如果 Leader 在 (0,0)，僚機在 Leader 的 (-2, 2) 處，即 (-2, 2)。
                # 此時僚機的 Local Pos 應該是 (0,0) （假設它就在起飛點）。
                # 當 Leader 飛到 (10,0)，僚機應該在 (8,2)。
                # 此時僚機相對於自己起飛點 (-2,2) 的位移是 (8-(-2), 2-2) = (10, 0)。
                # 這意味著如果我們只讓它們飛一樣的軌跡，它們就會保持隊形（除了轉彎時）。
                
                # 所以，我們只需要計算 "相對於自己起飛點" 的目標位置。
                # 但是，轉彎邏輯變得不同了。
                # Leader 轉彎時，位置不變。
                # 僚機轉彎時，位置要從 "轉彎前的隊形點" 飛到 "轉彎後的隊形點"。
                
                # 讓我們用 calculate_drone_target 返回 "相對於 Leader 起得點 (0,0) 的絕對座標"
                # 然後我們要扣掉這台無人機的 "初始偏移量" 嗎？
                # 不，offsets 定義的是 "相對於 Leader 的位置"。
                # 初始狀態：Leader at (0,0). Left Wing at (-2,2) relative to Leader.
                # Left Wing's Local Pos is (0,0). Its Absolute Pos is implicitly (-2,2) in the format frame.
                # So if Target Absolute Pos is (8,2), and Initial Absolute Pos was (-2,2).
                # The Delta is (10, 0).
                # New Local Pos = Initial Local (0,0) + Delta = (10,0).
                
                # Wait, calculating absolute target:
                # 1. Virtual Leader Pos (VLP)
                # 2. Target Global = VLP + R(yaw) * Offset
                # 3. Initial Global = (0,0) + R(0) * Offset = Offset
                # 4. Command Local = Target Global - Initial Global
                # This assumes everyone starts at Yaw 0 and satisfies the offset.
                
                # Let's verify start condition.
                # t=0. VLP=(0,0,alt). Yaw=0.
                # Target Global = (0,0) + (-2,2) = (-2,2).
                # Initial Global = (-2,2).
                # Command Local = (-2,2) - (-2,2) = (0,0). Correct. Spawns at 0.
                
                # t=Fly. VLP=(5,0). Yaw=0.
                # Target Global = (5,0) + (-2,2) = (3,2).
                # Command Local = (3,2) - (-2,2) = (5,0). Correct. Moves forward 5m.
                
                # t=Turn (90 deg). VLP=(10,0). Yaw=0->90.
                # Start of turn: Target = (10,0) + (-2,2) = (8,2).
                # End of turn: Target = (10,0) + Rot(90)*(-2,2) = (10,0) + (-2,-2) = (8,-2).
                # So Global Target goes (8,2) -> (8,-2).
                # Command Local = Global Target - Initial Global (-2,2).
                # Start Cmd: (8,2) - (-2,2) = (10,0).
                # End Cmd: (8,-2) - (-2,2) = (10, -4).
                # So it flies from (10,0) to (10,-4) locally.
                # This is a straight line.
                
                cmd_pos = [
                    target_pos[0] - self.offsets[i]['x'],
                    target_pos[1] - self.offsets[i]['y'],
                    target_pos[2] 
                ]
                
                self.publish_trajectory(i, cmd_pos, target_yaw)

    def calculate_drone_target(self, cruise_time, i):
        # 1. 計算虛擬 Leader 的狀態
        fly_time = 8.0   
        turn_time = 4.0  
        phase_time = fly_time + turn_time 
        cycle_time = phase_time * 4      
        
        t = cruise_time % cycle_time
        phase = int(t // phase_time)      
        t_phase = t % phase_time          
        
        # PX4 NED 座標系: x=北, y=東
        # 正方形路徑 (逆時針繞行): 北→東→南→西
        #   (10,0)------(10,10)
        #     |   B→        |
        #     ↑A          C↓
        #     |              |
        #   (0,0)-------(0,10)
        #          ←D
        corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        yaws = [0.0, math.pi/2, math.pi, -math.pi/2]

        start_corner = corners[phase]
        end_corner = corners[(phase + 1) % 4]
        current_yaw = yaws[phase]
        next_yaw = (yaws[(phase + 1) % 4])
        # 如果是從 -pi/2 轉到 0，需要避免旋轉一整圈
        if phase == 3: next_yaw = 0.0 # -pi/2 到 0 是逆時針，沒問題。或者從 3*pi/2 改為 -pi/2
        
        # 2. 獲取該無人機的 Offset
        off_x = self.offsets[i]['x']
        off_y = self.offsets[i]['y']
        off_z = self.offsets[i]['z']

        if t_phase < fly_time:
            # 直線飛行階段：大家都跟著 Leader 走直線，保持相對位置不變 (Yaw 固定)
            progress = t_phase / fly_time
            vl_x = start_corner[0] + (end_corner[0] - start_corner[0]) * progress
            vl_y = start_corner[1] + (end_corner[1] - start_corner[1]) * progress
            
            # 標準 2D 旋轉: Global = Leader + R(yaw) * BodyOffset
            gx = vl_x + (off_x * math.cos(current_yaw) - off_y * math.sin(current_yaw))
            gy = vl_y + (off_x * math.sin(current_yaw) + off_y * math.cos(current_yaw))

            return [gx, gy, self.takeoff_altitude + off_z], current_yaw

        else:
            # 轉彎階段：直線插值 (Shortcut/Cut corner)
            # 起點：Leader 在 end_corner, Yaw = current_yaw
            # 終點：Leader 在 end_corner, Yaw = next_yaw
            
            turn_progress = (t_phase - fly_time) / turn_time
            # 使用 Sigmoid 讓轉向平滑，但路徑是直線插值
            smooth_progress = 1 / (1 + math.exp(-6 * (turn_progress - 0.5)))
            
            # Start Point Global
            vl_x = end_corner[0]
            vl_y = end_corner[1]
            
            start_gx = vl_x + (off_x * math.cos(current_yaw) - off_y * math.sin(current_yaw))
            start_gy = vl_y + (off_x * math.sin(current_yaw) + off_y * math.cos(current_yaw))
            
            # End Point Global
            end_gx = vl_x + (off_x * math.cos(next_yaw) - off_y * math.sin(next_yaw))
            end_gy = vl_y + (off_x * math.sin(next_yaw) + off_y * math.cos(next_yaw))
            
            # Linear Interpolation for Position
            curr_gx = start_gx + (end_gx - start_gx) * smooth_progress
            curr_gy = start_gy + (end_gy - start_gy) * smooth_progress
            
            # Linear Interpolation for Yaw
            curr_yaw = current_yaw + (next_yaw - current_yaw) * smooth_progress
            
            # Normalize yaw
            while curr_yaw > math.pi: curr_yaw -= 2.0 * math.pi
            while curr_yaw < -math.pi: curr_yaw += 2.0 * math.pi
            
            return [curr_gx, curr_gy, self.takeoff_altitude + off_z], curr_yaw

    # 移除不再需要的 calculate_leader_path 和 calculate_formation_pos
    # 這是為了保持代碼乾淨，但為了 replace_string_in_file 方便，我們可以用新函數替換舊函數
    
    def unused_function(self):
        pass

    def publish_offboard_heartbeat(self, i):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pubs_offboard[i].publish(msg)

    def publish_trajectory(self, i, pos, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(x) for x in pos]
        msg.yaw = float(yaw)
        
        # 將未使用的目標速度與加速度設為 NaN，釋放飛控的軌跡產生器
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yawspeed = float('nan')

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pubs_trajectory[i].publish(msg)

    def send_command(self, i, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = self.drone_configs[i]['sys_id']
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pubs_command[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForceArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
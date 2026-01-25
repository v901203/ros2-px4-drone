import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import time
import math

class ForceArmCommander(Node):
    def __init__(self):
        super().__init__('force_arm_commander')

        # === 1. 修正後的 ID 與 名稱配置 (針對你的 start_env.sh) ===
        # Leader: 由 make 啟動 -> 預設是 /fmu, ID=1
        # Left Wing: 由 -i 1 啟動 -> /px4_1/fmu, ID=2
        # Right Wing: 由 -i 2 啟動 -> /px4_2/fmu, ID=3
        self.drone_configs = [
            {'ns': '/fmu', 'sys_id': 1, 'role': 'Leader'},
            {'ns': '/px4_1/fmu', 'sys_id': 2, 'role': 'Left Wing'},
            {'ns': '/px4_2/fmu', 'sys_id': 3, 'role': 'Right Wing'}
        ]
        
        # 編隊位置偏移 (NED 座標)
        self.offsets = [
            {'x': 0.0, 'y': 0.0},    # Leader
            {'x': -2.0, 'y': -2.0},  # Left Wing
            {'x': -2.0, 'y': 2.0}    # Right Wing
        ]

        # Takeoff configuration
        self.takeoff_altitude = -5.0
        self.takeoff_time = 5.0

        # 狀態追蹤
        self.vehicle_status = [{'nav_state': 0, 'arming_state': 0} for _ in range(3)]
        self.start_time = time.time()

        # === 2. 通訊設定 ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pubs_offboard = []
        self.pubs_trajectory = []
        self.pubs_command = []
        self.subs_status = []

        for i, config in enumerate(self.drone_configs):
            ns = config['ns']
            self.pubs_offboard.append(self.create_publisher(OffboardControlMode, f'{ns}/in/offboard_control_mode', qos_profile))
            self.pubs_trajectory.append(self.create_publisher(TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', qos_profile))
            self.pubs_command.append(self.create_publisher(VehicleCommand, f'{ns}/in/vehicle_command', qos_profile))
            self.subs_status.append(self.create_subscription(
                VehicleStatus, f'{ns}/out/vehicle_status_v1', 
                lambda msg, idx=i: self.status_callback(msg, idx), qos_profile))

        # === 3. 執行迴圈 ===
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.get_logger().info("🔥 暴力解鎖指揮官已啟動！準備強制起飛...")

    def status_callback(self, msg, idx):
        self.vehicle_status[idx]['nav_state'] = msg.nav_state
        self.vehicle_status[idx]['arming_state'] = msg.arming_state

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.start_time

        takeoff_phase = elapsed < self.takeoff_time
        if takeoff_phase:
            progress = min(elapsed / self.takeoff_time, 1.0)
            target_alt = self.takeoff_altitude * progress
            target_pos = [0.0, 0.0, target_alt]
            target_yaw = 0.0
        else:
            cruise_time = elapsed - self.takeoff_time
            leader_pos, leader_yaw = self.calculate_leader_path(cruise_time)

        for i, config in enumerate(self.drone_configs):
            self.publish_offboard_heartbeat(i)

            if takeoff_phase:
                self.publish_trajectory(i, target_pos, target_yaw)
            else:
                target_pos, target_yaw = self.calculate_formation_pos(i, leader_pos, leader_yaw)
                self.publish_trajectory(i, target_pos, target_yaw)

            if elapsed > 1.0 and elapsed < 15.0:
                self.try_force_arm(i)

    def try_force_arm(self, i):
        current_nav = self.vehicle_status[i]['nav_state']
        current_arm = self.vehicle_status[i]['arming_state']
        role = self.drone_configs[i]['role']
        sys_id = self.drone_configs[i]['sys_id']

        # 狀態 1: 如果還不是 Offboard 模式 (14)，切換它！
        if current_nav != 14:
            self.get_logger().info(f"⏳ [{role}] 請求切換 Offboard...")
            self.send_command(i, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        
        # 狀態 2: 已經是 Offboard，但還沒解鎖 (2)，強制解鎖！
        elif current_nav == 14 and current_arm != 2:
            self.get_logger().info(f"🔓 [{role}] 發送暴力解鎖指令！")
            
            # === 關鍵代碼 ===
            # param1 = 1.0 (Arm)
            # param2 = 211965.0 (這是 PX4 的強制金鑰，無視檢查)
            self.send_command(i, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                              param1=1.0, 
                              param2=211965.0) 

    def calculate_leader_path(self, cruise_time):
        # 簡單的正方形飛行 (在完成垂直起飛後開始)
        cycle = cruise_time % 40.0
        speed = 1.0
        if cycle < 10.0: return [speed*cycle, 0.0, self.takeoff_altitude], 0.0
        elif cycle < 20.0: return [10.0, speed*(cycle-10), self.takeoff_altitude], 1.57
        elif cycle < 30.0: return [10.0-speed*(cycle-20), 10.0, self.takeoff_altitude], 3.14
        else: return [0.0, 10.0-speed*(cycle-30), self.takeoff_altitude], -1.57

    def calculate_formation_pos(self, i, leader_pos, leader_yaw):
        if i == 0: return leader_pos, leader_yaw
        
        off_x = self.offsets[i]['x']
        off_y = self.offsets[i]['y']
        
        # 旋轉矩陣
        wx = leader_pos[0] + (off_x * math.cos(leader_yaw) - off_y * math.sin(leader_yaw))
        wy = leader_pos[1] + (off_x * math.sin(leader_yaw) + off_y * math.cos(leader_yaw))
        return [wx, wy, self.takeoff_altitude], leader_yaw

    def publish_offboard_heartbeat(self, i):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pubs_offboard[i].publish(msg)

    def publish_trajectory(self, i, pos, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(x) for x in pos]
        msg.yaw = float(yaw)
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
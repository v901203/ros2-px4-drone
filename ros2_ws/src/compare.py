#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math

class CoordDiagnosticNode(Node):
    def __init__(self):
        super().__init__('coord_diagnostic_node')

        self.spawn_points = [
            [0.0, 0.0],    # Leader (Instance 0)
            [2.0, -2.0],   # Left Wing (Instance 1)
            [-2.0, -2.0]   # Right Wing (Instance 2)
        ]
        
        # Namespaces mapping
        # Leader (Inst 0) -> /fmu
        # Left (Inst 1) -> /px4_1/fmu
        # Right (Inst 2) -> /px4_2/fmu
        self.namespaces = ["/fmu", "/px4_1/fmu", "/px4_2/fmu"]

        self.vehicle_positions = [None, None, None]
        self.gz_pos = [None, None, None]
        self.lidar_status = [None, None, None]
        self.last_lidar_stamp = [None, None, None]

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

        self.subs_lidar = []
        for i in range(3):
            self.subs_lidar.append(self.create_subscription(
                LaserScan,
                f'/drone_{i}/scan',
                lambda msg, idx=i: self.lidar_callback(msg, idx),
                10
            ))

        # === Gazebo Dynamic Pose Subscriber (from ros_gz_bridge) ===
        # /world/default/dynamic_pose/info (Pose_V) -> /gz_dynamic_pose (PoseArray)
        self.sub_gz_dynamic_pose = self.create_subscription(
            PoseArray,
            '/gz_dynamic_pose',
            self.gz_pose_array_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.log_comparison)
        print("🔍 診斷節點已啟動 (ROS2 訂閱模式: PX4 + /gz_dynamic_pose)")

    def gz_pose_array_callback(self, msg):
        # PoseArray 前三筆對應三台模型本體姿態
        if len(msg.poses) < 3:
            return

        for i in range(3):
            pose = msg.poses[i]
            self.gz_pos[i] = [pose.position.x, pose.position.y, pose.position.z]

    def lidar_callback(self, msg, idx):
        finite_ranges = [r for r in msg.ranges if math.isfinite(r)]
        if finite_ranges:
            min_dist = min(finite_ranges)
            front_idx = len(msg.ranges) // 2
            front_dist = msg.ranges[front_idx] if math.isfinite(msg.ranges[front_idx]) else float('inf')
            self.lidar_status[idx] = {
                'count': len(msg.ranges),
                'min_dist': min_dist,
                'front_dist': front_dist,
                'stamp': self.get_clock().now().nanoseconds,
            }
        else:
            self.lidar_status[idx] = {
                'count': len(msg.ranges),
                'min_dist': float('inf'),
                'front_dist': float('inf'),
                'stamp': self.get_clock().now().nanoseconds,
            }

    def log_comparison(self):
        print("\n" + "="*80)
        print(f"⏰ 時間: {self.get_clock().now().to_msg().sec}s")
        print("="*80)
        
        for i in range(3):
            p = self.vehicle_positions[i]
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
                print("  ⏳ Gazebo 數據:     等待中... (檢查 /gz_dynamic_pose bridge)")

            lidar = self.lidar_status[i]
            if lidar is None:
                print(f"  ⏳ 雷達 bridge:     等待中... (檢查 /drone_{i}/scan)")
            else:
                age_sec = (self.get_clock().now().nanoseconds - lidar['stamp']) / 1e9
                if age_sec > 2.0:
                    print(f"  ⚠️  雷達 bridge:     最後更新 {age_sec:.1f}s 前 (可能中斷)")
                else:
                    front_text = "inf" if not math.isfinite(lidar['front_dist']) else f"{lidar['front_dist']:.2f}m"
                    min_text = "inf" if not math.isfinite(lidar['min_dist']) else f"{lidar['min_dist']:.2f}m"
                    print(
                        f"  📡 雷達 bridge:     正常 | beams={lidar['count']} | front={front_text} | min={min_text}"
                    )
        
        print("="*80)

def main(args=None):
    rclpy.init(args=args)
    node = CoordDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

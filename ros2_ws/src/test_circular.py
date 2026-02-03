#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import math
import time

class FormationControlNode(Node):
    def __init__(self):
        super().__init__('formation_control_node')
        
        # 參數設置
        self.RADIUS = 3.0        # 編隊半徑 (米)
        self.ALTITUDE = 5.0      # 飛行高度 (米)
        self.SPEED = 0.2         # 角速度 (rad/s)
        self.formation_angles = [0.0, 0.0, math.pi] # Leader(0), Left(0), Right(180) 相對角度
        
        # 定義物理出生點偏移量 [East, North] (ENU)
        self.spawn_offsets = [
            [0.0, 0.0],    # Leader (Instance 0)
            [-2.0, 2.0],   # Left Wing (Instance 1)
            [-2.0, -2.0]   # Right Wing (Instance 2)
        ]

        # Namespaces for the 3 drones:
        # Leader (Inst 0) -> /fmu
        # Left (Inst 1) -> /px4_1/fmu
        # Right (Inst 2) -> /px4_2/fmu
        self.namespaces = ["/fmu", "/px4_1/fmu", "/px4_2/fmu"]

        self.vehicle_status = [VehicleStatus() for _ in range(3)]
        self.vehicle_local_position = [VehicleLocalPosition() for _ in range(3)]
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pubs = [] # [offboard_0, traj_0, cmd_0, offboard_1, ...]

        for i in range(3):
            ns = self.namespaces[i]
            
            # Publishers (Input topics usually don't have _v1)
            p_offboard = self.create_publisher(OffboardControlMode, f'{ns}/in/offboard_control_mode', qos_profile)
            p_trajectory = self.create_publisher(TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', qos_profile)
            p_command = self.create_publisher(VehicleCommand, f'{ns}/in/vehicle_command', qos_profile)
            
            self.pubs.extend([p_offboard, p_trajectory, p_command])
            
            # Subscribers (Output topics usually don't have _v1 in standard ROS2/PX4 setup)
            def status_cb(msg, sys_id=i): self.vehicle_status[sys_id] = msg
            def pos_cb(msg, sys_id=i): self.vehicle_local_position[sys_id] = msg
            
            # Using v1 as confirmed by other working scripts in the workspace
            self.create_subscription(VehicleStatus, f'{ns}/out/vehicle_status_v1', status_cb, qos_profile)
            self.create_subscription(VehicleLocalPosition, f'{ns}/out/vehicle_local_position_v1', pos_cb, qos_profile)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time_start = time.time()
        
        # Arming state
        self.is_armed = [False]*3
        self.offboard_set = [False]*3

    def arm_and_set_offboard(self, i):
        # Send Offboard Heartbeat
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publish_offboard_control_mode(i, msg)

        # Set Offboard Mode
        if self.vehicle_status[i].nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            cmd = VehicleCommand()
            cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            cmd.param1 = 1.0
            cmd.param2 = 6.0
            cmd.target_system = i + 1
            cmd.target_component = 1
            cmd.source_system = 1
            cmd.source_component = 1
            cmd.from_external = True
            self.publish_vehicle_command(i, cmd)

        # Arm
        if self.vehicle_status[i].arming_state != VehicleStatus.ARMING_STATE_ARMED:
            cmd = VehicleCommand()
            cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            cmd.param1 = 1.0
            cmd.target_system = i + 1
            cmd.target_component = 1
            cmd.source_system = 1
            cmd.source_component = 1
            cmd.from_external = True
            self.publish_vehicle_command(i, cmd)

    def publish_offboard_control_mode(self, i, msg):
        pub = self.pubs[i*3] 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)

    def publish_trajectory_setpoint(self, i, msg):
        pub = self.pubs[i*3 + 1]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)

    def publish_vehicle_command(self, i, msg):
        pub = self.pubs[i*3 + 2]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)

    def timer_callback(self):
        current_time = time.time() - self.time_start
        
        # Calculate rotation angle
        rotation_angle = self.SPEED * current_time
        
        for i in range(3):
            self.arm_and_set_offboard(i)
            
            # Leader (0) Hover at (0,0)
            if i == 0:
                msg = TrajectorySetpoint()
                msg.position = [0.0, 0.0, -self.ALTITUDE]
                msg.yaw = 0.0
                self.publish_trajectory_setpoint(i, msg)
                continue
            
            # Wings (1, 2)
            # 1. Calculate Phase
            # Leader=0 deg, Left=0 deg (offset), Right=180 (offset)? 
            # Original code: [0.0, 0.0, math.pi]. 
            # Reverting logic to ensure formation matches original intent
            phase_angle = self.formation_angles[i] + rotation_angle
            
            # 2. Target World (ENU: X=East, Y=North)
            target_world_east = self.RADIUS * math.cos(phase_angle)
            target_world_north = self.RADIUS * math.sin(phase_angle)
            
            # 3. Local Offset (ENU)
            spawn_east = self.spawn_offsets[i][0]
            spawn_north = self.spawn_offsets[i][1]
            
            target_local_east = target_world_east - spawn_east
            target_local_north = target_world_north - spawn_north
            
            # 4. Map to NED (X=North, Y=East, Z=Down)
            msg = TrajectorySetpoint()
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.yaw = float('nan')
            
            msg.position[0] = target_local_north # North
            msg.position[1] = target_local_east  # East
            msg.position[2] = -self.ALTITUDE     # Down
            
            self.publish_trajectory_setpoint(i, msg)

def main(args=None):
    rclpy.init(args=args)
    node = FormationControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

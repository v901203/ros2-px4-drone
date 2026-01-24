import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition

class TestFlightNode(Node):
    def __init__(self):
        super().__init__('test_flight_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        
        # [關鍵修正] 這裡加上了 _v1
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile)

        self.vehicle_status = VehicleStatus()
        self.current_estimate_z = 0.0
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(">>> 程式已啟動：準備飛行 <<<")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def local_position_callback(self, msg):
        # PX4 的 Z 軸向下為正，轉為正數方便閱讀
        self.current_estimate_z = -msg.z 

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        # 每 2 秒印出一次 PX4 估計的高度
        if self.offboard_setpoint_counter % 20 == 0:
            self.get_logger().info(f"PX4 高度估計 (EKF): {self.current_estimate_z:.3f} m")

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0]
        msg.yaw = 1.57
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(">>> 切換 Offboard 模式 <<<")

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0, 
            param2=21196.0
        )
        self.get_logger().info(">>> 強制解鎖 <<<")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestFlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus

class OffboardTest(Node):
    def __init__(self):
        super().__init__('offboard_test')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v3', self.status_cb, qos)
        self.current_z = 0.0
        self.arming_state = 0
        self.counter = 0
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Baslatildi. ARM denemesi yapilacak...')

    def odom_cb(self, msg):
        self.current_z = msg.position[2]

    def status_cb(self, msg):
        self.arming_state = msg.arming_state

    def timer_callback(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)

        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = ts
        self.control_pub.publish(msg)

        sp = TrajectorySetpoint()
        sp.position = [0.0, 2.0, -5.0]
        sp.yaw = 0.0
        sp.timestamp = ts
        self.setpoint_pub.publish(sp)

        # Her 2 saniyede bir offboard + arm gonder (ta ki arm olana kadar)
        if self.counter > 50 and self.arming_state != 2:
            if self.counter % 40 == 0:
                self.get_logger().info(f'ARM denemesi... (arming_state={self.arming_state})')
                self.send_command(176, 1.0, 6.0)  # Offboard mode
                self.send_command(400, 1.0, 21196.0)  # Arm
                # Force arm
                self.send_command(400, 1.0, 0.0)  # Arm without checks

        if self.arming_state == 2 and self.counter % 200 == 0:
            self.get_logger().info(f'ARMED! Z: {self.current_z:.2f}')

        if self.counter % 100 == 0:
            self.get_logger().info(f'[{self.counter}] Z={self.current_z:.2f} arm={self.arming_state}')

        self.counter += 1

    def send_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = OffboardTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

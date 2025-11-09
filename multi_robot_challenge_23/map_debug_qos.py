#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

class QoSMapChecker(Node):
    def __init__(self):
        super().__init__('map_debug_qos')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.get_logger().info('🟡 Venter på /map med TRANSIENT_LOCAL QoS...')

    def cb(self, msg):
        self.get_logger().info(f"✅ Mottatt map: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QoSMapChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

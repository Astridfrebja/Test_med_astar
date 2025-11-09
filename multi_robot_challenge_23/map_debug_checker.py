#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapDebugChecker(Node):
    def __init__(self):
        super().__init__('map_debug_checker')
        self.msg_received = False
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.callback,
            10
        )
        self.get_logger().info("🟡 Venter på /map-data...")
        self.timer = self.create_timer(5.0, self.check_timeout)

    def callback(self, msg):
        if not self.msg_received:
            self.msg_received = True
            self.get_logger().info(
                f"✅ Map mottatt: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
            )
            self.destroy_subscription(self.sub)
            self.destroy_timer(self.timer)

    def check_timeout(self):
        if not self.msg_received:
            self.get_logger().error("❌ Ingen /map-data mottatt etter 5 sekunder!")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MapDebugChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

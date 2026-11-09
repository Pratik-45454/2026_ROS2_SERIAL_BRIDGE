import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import math

WHEEL_DIAMETER_M = 0.0574
COUNTS_PER_REV   = 4000
DIST_PER_COUNT   = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV


class ProcessOdom(Node):
    def __init__(self):
        super().__init__('process_odom')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT


        self._pub = self.create_publisher(Float32MultiArray, '/odom_dist_m', qos_profile)
        self._sub = self.create_subscription(
            Int32MultiArray,
            '/odom_data_received',
            self.odom_callback,
            qos_profile,
        )

        self.get_logger().info(
            f'Ready. dist_per_count={DIST_PER_COUNT * 1000:.6f} mm/count'
        )

    def odom_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f'Expected 2 values, got {len(msg.data)}')
            return

        
        dist_x = float(-msg.data[0]) * DIST_PER_COUNT   # dw2 → X meters
        dist_y = float(-msg.data[1]) * DIST_PER_COUNT   # dw1 → Y meters

        print("dist x and y:" , dist_x*1000 , dist_y*1000)
        out = Float32MultiArray()
        out.data = [dist_x, dist_y]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
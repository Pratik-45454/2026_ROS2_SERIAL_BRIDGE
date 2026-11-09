#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, UInt8
import Jetson.GPIO as GPIO

BUTTON_PIN = 33  # change to your physical pin

class GPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')


        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability=QoSDurabilityPolicy.VOLATILE

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BUTTON_PIN, GPIO.IN)

        self.pub = self.create_publisher(UInt8, '/set_reset', qos_profile)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("GPIO node ready")

    def timer_callback(self):
        try:
            msg = UInt8()
            msg.data = 1 if GPIO.input(BUTTON_PIN) == GPIO.HIGH else 0
            
            self.pub.publish(msg)
            print(f"Button data: {msg.data}")
        except Exception as e:
            print(f"ERROR: {e}")

    def destroy_node(self):
            GPIO.cleanup()
            super().destroy_node()

def main():
    rclpy.init()
    node = GPIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
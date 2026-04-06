#!/usr/bin/env python3

"""
Serial bridge between Jetson and STM32.

- Subscribes to /cmd_vel (geometry_msgs/Twist)
- Sends velocity commands over serial
- Reads IMU yaw data from STM32 and publishes sensor_msgs/Imu
"""

import math
import struct
import time
from math import cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from dev_pi_communicate.crc8 import crc8
from dev_pi_communicate.serial_comms import serial_comms



START_BYTE = 0xA5

RECEIVE_SIZE = 1 + 24 + 1
TRANSMIT_SIZE = 1 + 4 + 1
SERIAL_BAUDRATE = 115200

SERIAL_PORT = "/dev/serial/by-id/usb-Silicon_Labs_ESP_TTL_2_ESP_TTL_2-if00-port0"

M_PI = math.pi



class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.serial = serial_comms(
            SERIAL_PORT,
            SERIAL_BAUDRATE,
            RECEIVE_SIZE,
            TRANSMIT_SIZE,
            "CRC",
        )

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.imu_publisher = self.create_publisher(Imu, "imu/data", 10)

        self.cmd_vel = Twist()
        self.last_tx_time = time.time()

        self.create_timer(0.001, self.serial_loop)

        self.get_logger().info("Serial bridge started")


    def send_cmd_vel(self):
        if(self.cmd_vel.angular.x == 1):
            payload = b"".join([
                struct.pack("B", START_BYTE),
                struct.pack("f", self.cmd_vel.linear.x),
                struct.pack("f", self.cmd_vel.linear.y),
                struct.pack("f", self.cmd_vel.angular.z),
                struct.pack("B", 0x67),
            ])

        else:
            payload = b"".join([
                struct.pack("B", START_BYTE),
                struct.pack("f", self.cmd_vel.linear.x),
                struct.pack("f", self.cmd_vel.linear.y),
                struct.pack("f", self.cmd_vel.angular.z),
                struct.pack("B", 0x00),
            ])
        

        crc = self.calculate_crc(payload[1:])
        packet = payload + struct.pack("B", crc)

        self.serial.write_data(packet)


    def serial_loop(self):
        self.send_cmd_vel()

        data = self.serial.read_data()
        if data is None:
            return

        imu_raw = struct.unpack("<ffffff", data[:-1])
        self.publish_imu(imu_raw[0])


    def publish_imu(self, yaw_deg):
        yaw = yaw_deg * M_PI / 180.0
        qw, qx, qy, qz = self.yaw_to_quaternion(yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"

        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz

        self.imu_publisher.publish(imu)


    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg


    def calculate_crc(self, data: bytes) -> int:
        c = crc8()
        c.update(data)
        return c.digest()[0]

    def yaw_to_quaternion(self, yaw):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        return cy, 0.0, 0.0, sy



def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

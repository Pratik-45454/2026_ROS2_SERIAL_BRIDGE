#!/usr/bin/env python3

# ---------------- SERIAL BRIDGE BETWEEN RASPBERRY PI AND STM32 ----------------
# Publishes:
#   - IMU yaw as sensor_msgs/Imu          -> "imu_data"
#   - SICK sensor data (4 channels)       -> "sick_data"
#   - Proximity data (raw uint8)          -> "proximity_data"
#
# Reads a fixed, well-defined serial packet from STM32

import math
import struct
import time
from math import cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

from dev_pi_communicate.crc8 import crc8
from dev_pi_communicate.serial_comms import serial_comms


START_BYTE = 0xA5
RECIEVE_SIZE = 1 + 25 + 1
TRANSMIT_SIZE = 1 + 1 + 1

SERIAL_BAUDRATE = 115200
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54A7002607-if00"

M_PI = 3.14159265358979323846



class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.serial = serial_comms(
            SERIAL_PORT,
            SERIAL_BAUDRATE,
            RECIEVE_SIZE,
            TRANSMIT_SIZE,
            "CRC",
        )

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.imu_publisher = self.create_publisher(Imu, "imu_data", qos)
        self.sick_publisher = self.create_publisher(Float32MultiArray, "sick_data", qos)
        self.prox_publisher = self.create_publisher(UInt8MultiArray, "proximity_data", qos)

        self.last_transmit_time = time.time()

        self.timer = self.create_timer(0.001, self.serial_read_callback)

        self.get_logger().info("Serial bridge ready.")


    def send_data(self):
        if time.time() - self.last_transmit_time < 0.01:
            return

        payload = bytes(struct.pack("B", START_BYTE))
        crc = self.calculate_crc(payload[1:])
        packet = payload + bytes(struct.pack("B", crc))

        self.serial.write_data(packet)
        self.last_transmit_time = time.time()


    def serial_read_callback(self):
        self.send_data()

        raw = self.serial.read_data()
        if raw is None:
            return

        # Packet format:
        # <iiHHHHfBI
        # [0] encoder_left
        # [1] encoder_right
        # [2-5] sick sensors
        # [6] yaw (deg)
        # [7] proximity (uint8)
        # [8] timestamp

        data = struct.unpack("<iiHHHHfBI", raw[:-1])

        self.process_sick(data[2:6])
        self.process_imu(data[6])
        self.process_proximity(data[7])


    def process_sick(self, data):
        msg = Float32MultiArray()

        msg.data = [
            0.31709677 * data[0] - 15.75917103,
            0.29745752 * data[1] + 10.84057088,
            0.28342641 * data[2] - 5.54270299,
            0.30279794 * data[3] - 3.49987992,
        ]

        self.sick_publisher.publish(msg)

    def process_imu(self, yaw_deg):
        imu = Imu()

        yaw = yaw_deg * M_PI / 180.0
        qw, qx, qy, qz = self.rpy_to_quaternion(0.0, 0.0, yaw)

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"

        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0

        self.imu_publisher.publish(imu)

    def process_proximity(self, prox):
        msg = UInt8MultiArray()
        msg.data = [prox]
        self.prox_publisher.publish(msg)


    def calculate_crc(self, data):
        c = crc8()
        c.update(data)
        return c.digest()[0]

    def rpy_to_quaternion(self, roll, pitch, yaw):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return qw, qx, qy, qz



def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.serial.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

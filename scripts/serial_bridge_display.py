import rclpy
import struct
import csv
import os
import time
from rclpy.node import Node
from myserialbridge.serial_comms import serial_comms
from std_msgs.msg import UInt8MultiArray, Byte, UInt8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

serial_port = '/dev/serial/by-id/usb-Silicon_Labs_ESP_TTL_2_ESP_TTL_2-if00-port0'

START_BYTE = 0xA5
data_rx_size = 1 + 18 + 1  # start + payload + crc

class SerialBridgeDisplay(Node):

    def __init__(self):
        super().__init__('serial_bridge_display')
        self.get_logger().info("SerialBridgeDisplay initialized")

        self.serial_display = serial_comms(
            serial_port=serial_port,
            serial_baudrate=115200,
            tx_data_size=data_rx_size,
            rx_data_size=data_rx_size,
            hash_func_="CRC",
        )

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        init_robot_qos_profile = QoSProfile(depth=10)
        init_robot_qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        init_robot_qos_profile.durability = QoSDurabilityPolicy.VOLATILE



        self.forest_data_pub         = self.create_publisher(UInt8MultiArray, '/forest_data', qos_profile)
        self.color_data_pub          = self.create_publisher(UInt8, '/color_data', qos_profile)
        self.zone_data_pub          = self.create_publisher(UInt8, '/zone_data', qos_profile)
        self.trust_data_pub          = self.create_publisher(UInt8, '/trust_data', qos_profile)
        self.activate_motors_data_pub = self.create_publisher(UInt8, '/activate_motors_data', qos_profile)
        self.init_robot_data_pub = self.create_publisher(UInt8, '/init_robot', init_robot_qos_profile)
        self.pick_dropped_data_pub = self.create_publisher(UInt8, '/pick_dropped_data', qos_profile)
        self.second_row_data_pub = self.create_publisher(UInt8, '/second_row_data', qos_profile)
        self.third_row_data_pub = self.create_publisher(UInt8, '/third_row_data', qos_profile)

        log_dir = os.path.expanduser('~/tmp1')
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, f'serial_bridge_display_{int(time.time() * 1000)}.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp_ms', 'color', 'trust', 'activate_motors'])
        self._csv_file.flush()
        self.get_logger().info(f'[SerialBridge] CSV logging to: {csv_path}')

        self.timer = self.create_timer(0.001, self.read_write_serial_data)

    def write_display(self):
        # nothing to send for now
        pass

    def read_display(self):
        _data = self.serial_display.read_data()
        if _data is None:
            return

        payload = _data[:-1]  # strip CRC byte

        if len(payload) != 18:
            print(f"Display bad packet: got {len(payload)} bytes, expected 15")
            return

        color, trust, bl_1, bl_2, bl_3, bl_4, bl_5, bl_6, bl_7, bl_8, bl_9, bl_10, bl_11, bl_12, control, pick_dropped, second_row, third_row = \
            struct.unpack("<BBBBBBBBBBBBBBBBBB", payload)

        # unpack control byte (same as sniffer)
        motors = [(control >> i) & 1 for i in range(6)]
        init_all   = (control >> 6) & 1
        start_tree = (control >> 7) & 1

        print(f"init = {init_all}",
              f"color={color}, trust={trust},"
              f"grids={[bl_1,bl_2,bl_3,bl_4,bl_5,bl_6,bl_7,bl_8,bl_9,bl_10,bl_11,bl_12]}, "
              f"motors={motors}, init={init_all}, start={start_tree}, pick_dropped={pick_dropped}, second_row={second_row}, third_row={third_row}")

        color_msg = UInt8()
        color_msg.data = color
        
        zone_msg = UInt8()
        zone_msg.data = 0 # (0,1, 2): (Start, retry1, retry2)

        trust_msg = UInt8()
        trust_msg.data = trust

        activate_msg = UInt8()
        activate_msg.data = control  # publish raw control byte, let subscribers unpack

        forest_msg = UInt8MultiArray()
        forest_msg.data = [bl_1, bl_2, bl_3, bl_4, bl_5, bl_6,
                           bl_7, bl_8, bl_9, bl_10, bl_11, bl_12]
        
        pick_dropped_msg = UInt8()
        pick_dropped_msg.data = pick_dropped

        second_row_msg = UInt8()
        second_row_msg.data = second_row

        third_row_msg = UInt8()
        third_row_msg.data = third_row

        self.color_data_pub.publish(color_msg)
        self.zone_data_pub.publish(zone_msg)
        self.trust_data_pub.publish(trust_msg)
        self.activate_motors_data_pub.publish(activate_msg)
        self.init_robot_data_pub.publish(activate_msg)
        self.forest_data_pub.publish(forest_msg)
        self.pick_dropped_data_pub.publish(pick_dropped_msg)
        self.second_row_data_pub.publish(second_row_msg)
        self.third_row_data_pub.publish(third_row_msg)

        now_ms = int(time.time() * 1000)
        self._csv_writer.writerow([now_ms, color, trust, control])
        self._csv_file.flush()

    def read_write_serial_data(self):
        self.write_display()
        self.read_display()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.serial_display.close()
        node._csv_file.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
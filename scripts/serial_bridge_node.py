import rclpy
import struct
import csv
import os
import time
from rclpy.node import Node
from serial_bridge_2026.serial_comms import serial_comms
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, Int32MultiArray, Byte, Float32
from geometry_msgs.msg import Twist
from serial_bridge_2026.crc8 import crc8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

serial_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

START_BYTE = 0xA5

bluepill_tx_size = 1 + 21 + 1
bluepill_rx_size = 1 + 77 + 1

HARDCODED_MODE  = False
HARDCODED_VX    =  0.1
HARDCODED_VY    =  0.1
HARDCODED_OMEGA =  0.0
FLIP_DELAY_S    =  10.0

SICK_FR_M = 0.383595
SICK_FR_B = -2.44403
SICK_FL_M = 0.240183
SICK_FL_B = 3.44336

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.get_logger().info("SerialBridgeNode initialized")

        self.serial_bluepill = serial_comms(
            serial_port=serial_port,
            serial_baudrate=115200,
            tx_data_size=bluepill_tx_size,
            rx_data_size=bluepill_rx_size,
            hash_func_="CRC",
        )

        qos_profile_cmd = QoSProfile(depth=1)
        qos_profile_cmd.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_cmd.durability  = QoSDurabilityPolicy.VOLATILE

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.yaw_pub        = self.create_publisher(Float32,            '/imu_data_received',  qos_profile)
        self.dead_pub       = self.create_publisher(Int32MultiArray,    '/odom_data_received', qos_profile)
        self.sick_pub       = self.create_publisher(Float32MultiArray,  '/sick_data',          qos_profile)
        self.prox_front_pub = self.create_publisher(Byte,               '/prox_front_received',qos_profile)
        self.prox_rear_pub  = self.create_publisher(Byte,               '/prox_rear_received', qos_profile)

        self.yaw_setpoint_sub = self.create_subscription(
            Float32MultiArray, '/setpoint_yaw',
            self.yaw_setpoint_callback, qos_profile_cmd)

        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.twist_callback, qos_profile_cmd)

        self.twist_msg = None
        self.setpoint_msg = Float32MultiArray()
        self.setpoint_msg.data = [0.0, 0.0]
        self.setpoint_msg.data[1] = 0x42

        
        self._sick_filtered = None   # holds last filtered [f1, f2, f3, f4]
        self._sick_alpha = 0.2       # smoothing factor

        # --- CSV logger setup ---
        log_dir = os.path.expanduser('~/tmp1')
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, f'serial_bridge_odom_{int(time.time() * 1000)}.csv')
        self._csv_file   = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp_ms', 'dw1', 'dw2'])
        self._csv_file.flush()
        self.get_logger().info(f'[SerialBridge] Odom CSV logging to: {csv_path}')

        self._start_time = time.time()
        self._flipped    = False

        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(0.001, self.read_write_serial_data)



    def twist_callback(self, msg):
        self.twist_msg = msg

    def yaw_setpoint_callback(self, msg: Float32MultiArray):
        self.setpoint_msg = msg


    def write_bluepill(self):
        # if HARDCODED_MODE:
        #     elapsed = time.time() - self._start_time
        #     vx      = HARDCODED_VX if elapsed < FLIP_DELAY_S else -HARDCODED_VX
        #     vy      = HARDCODED_VY
        #     omega   = HARDCODED_OMEGA

        #     if elapsed >= FLIP_DELAY_S and not self._flipped:
        #         self.get_logger().info(f'[SerialBridge] Flipping vx sign after {elapsed:.1f}s')
        #         self._flipped = True
        # else:
        if self.twist_msg is None:
            return
        vx    = self.twist_msg.linear.x
        vy    = self.twist_msg.linear.y
        omega = self.twist_msg.angular.z


        print(f"{vx, vy, omega}")
        payload = b"".join([
            struct.pack("f", vx),
            struct.pack("f", vy),
            struct.pack("f", omega),
            struct.pack("f", self.setpoint_msg.data[0]),
            struct.pack("B", int(self.setpoint_msg.data[1]) & 0xFF)
        ])

        crc   = self.serial_bluepill.calc_crc(payload)
        frame = b"".join([
            struct.pack("B", START_BYTE),
            payload,
            struct.pack("B", crc),
        ])
        self.serial_bluepill.write_data(frame)



    def read_bluepill(self):
        _data = self.serial_bluepill.read_data()

        if _data is None:
            return


        present_time = self.get_clock().now()

        payload = _data[:-1]

        if len(payload) != 77:
            print(f"Bluepill bad packet: expected 29 bytes, got {len(payload)}")
            return

        (dw1, dw2,
         s1, s2, s3, s4,
         prox,
         yaw, pitch, roll,
         pid_sp_0, pid_sp_1, pid_sp_2, pid_sp_3,
         pid_ip_0, pid_ip_1, pid_ip_2, pid_ip_3,
         pid_op_0, pid_op_1, pid_op_2, pid_op_3
        ) = struct.unpack("<iiHHHHBfffffffffffffff", payload)

        front_prox1 = prox & 0b00000001
        front_prox2 = prox & 0b00000010
        rear_prox1  = prox & 0b00000100
        rear_prox2  = prox & 0b00001000
        front_prox  = 1 if (front_prox1 or front_prox2) else 0
        rear_prox   = 1 if (rear_prox1  or rear_prox2)  else 0

        # time diff in ms
        diff_ms = (present_time - self.prev_time).nanoseconds / 1e6
        self.prev_time = present_time

        print(
            f"received: dw1={dw1}, dw2={dw2}, "
            f"s1={s1}, s2={s2}, s3={s3}, s4={s4}, "
            f"front={front_prox}, rear={rear_prox}, "
            f"yaw={yaw:.3f}"
            f"dt={diff_ms:.2f} ms"
        )
        # print(f"raw_prox binary: {bin(prox)}, front1={front_prox1}, front2={front_prox2}")


        received_dead = Int32MultiArray()
        received_dead.data = [dw1, dw2]

        received_sick = UInt16MultiArray()
        received_sick.data = [s1, s2, s3, s4]
        self.process_sick(received_sick.data)

        received_front_prox      = Byte()
        received_front_prox.data = bytes([front_prox & 0xFF])
        received_rear_prox       = Byte()
        received_rear_prox.data  = bytes([rear_prox  & 0xFF])

        yaw_msg      = Float32()
        yaw_msg.data = yaw

        self.dead_pub.publish(received_dead)
        self.prox_front_pub.publish(received_front_prox)
        self.prox_rear_pub.publish(received_rear_prox)
        self.yaw_pub.publish(yaw_msg)

        # CSV log
        now_ms = int(time.time() * 1000)
        self._csv_writer.writerow([now_ms, dw1, dw2])
        self._csv_file.flush()



    # def process_sick(self, data):
    #     raw = [
    #         SICK_FR_M * data[0] + SICK_FR_B,
    #         SICK_FR_M * data[1] + SICK_FR_B,
    #         SICK_FR_M * data[2] + SICK_FR_B,
    #         SICK_FL_M * data[3] + SICK_FL_B,
    #     ]

    #     if self._sick_filtered is None:
    #         self._sick_filtered = raw 
    #     else:
    #         a = self._sick_alpha
    #         self._sick_filtered = [
    #             a * r + (1 - a) * f
    #             for r, f in zip(raw, self._sick_filtered)
    #         ]

    #     sick_msg = Float32MultiArray()
    #     sick_msg.data = self._sick_filtered
    #     print(sick_msg.data[0] - sick_msg.data[3])
    #     self.sick_pub.publish(sick_msg)
    
    def process_sick(self, data):
        sick_msg = Float32MultiArray()
        sick_msg.data = [0.99 * d for d in data]
        self.sick_pub.publish(sick_msg)


    def read_write_serial_data(self):
        self.write_bluepill()
        self.read_bluepill()



def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.serial_bluepill.close()
        node._csv_file.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
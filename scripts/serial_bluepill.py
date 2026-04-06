#! /usr/bin/env python3

# -------------------------SERIAL BRIDGE BETWEEN RASP PI AND STM32-------------------------------
# This node subscribes to
#   I 'cmd_robot'_vel topic to recieve velocity commands
#   II 'act_vel' from BT to recieve actuator control signals, and  write the data to stm discovery

# This node reads odometry value from bluepill and publish the data on
#   I 'freewheel/odom' topic to ekf filter package through
#   II'Ball_status' topic to BT

import math
import struct
import time
from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64MultiArray, UInt8MultiArray, UInt32MultiArray
#from ros2_bno_msg.msg import BnoData
from dev_pi_communicate.crc8 import crc8
from dev_pi_communicate.serial_comms import serial_comms

START_BYTE = 0xA5
RECIEVE_SIZE = 1 + 25 + 1
TRANSMIT_SIZE = 1 + 1 + 1
serial_baudrate = 115200

M_PI = 3.14159265358979323846
serial_port_ros = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54A7002607-if00"
#serial_port_pico = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E661AC88633E5127-if00"
#serial_port_address_FTDI = (
#    "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
#)
#serial_port_address_black = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
#serial_port_address_blue = (
#    "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
#)
#serial_port_address_bluepill = (
#    "/dev/serial/by-id/usb-freepill_STM32_Virtual_ComPort_6D72358E5756-if00"
#)


# serial_port_stm = (
#     "/dev/serial/by-id/usb-STMicroelectronics_STM32_Vi       # imu_msg.linear_acceleration.x = data[3]
        # imu_msg.linear_acceleration.y = data[4]
        # imu_msg.linear_acceleration.z = data[5]
        # imu_msg.orientation_covariance[0] = 0.00121847
        # imu_msg.orientation_covariance[1] = 0.0
        # imu_msg.orientation_covariance[2] = 0.0
        # imu_msg.orientation_covariance[3] = 0.0
        # imu_msg.orientation_covariance[4] = 0.00121847
        # imu_msg.orientation_covariance[5] = 0.0
        # imu_msg.orientation_covariance[6] = 0.0
        # imu_msg.orientation_covariance[7] = 0.0
        # imu_msg.orientation_covariance[8] = 0.00121847

        # imu_msg.angular_velocity_covariance[0] = 4.4944
        # imu_msg.angular_velocity_covariance[1] = 0.0
        # imu_msg.angular_velocity_covariance[2] = 0.0
        # imu_msg.angular_velocity_covariance[3] = 0.0
        # imu_msg.angular_velocity_covariance[4] = 4.4944
        # imu_msg.angular_velocity_covariance[5] = 0.0
        # imu_msg.angular_velocity_covariance[6] = 0.0
        # imu_msg.angular_velocity_covariance[7] = 0.0
        # imu_msg.angular_velocity_covariance[8] = 4.4944

        # imu_msg.linear_acceleration_covariance[0] = 0.25
        # imu_msg.linear_acceleration_covariance[1] = 0.0
        # imu_msg.linear_acceleration_covariance[2] = 0.0
        # imu_msg.linear_acceleration_covariance[3] = 0.0
        # imu_msg.linear_acceleration_covariance[4] = 0.25
        # imu_msg.linear_acceleration_covariance[5] = 0.0
        # imu_msg.linear_acceleration_covariance[6] = 0.0
        # imu_msg.linear_acceleration_covariance[7] = 0.0
        # imu_msg.linear_acceleration_covariance[8] = 0.25rtual_ComPort_324B30633331-if00"
# )
"""
Junction euivalent Int ----------------------------
"""
# JUNCTION_NOT_DETECTED = 0
# LEFT_TURN = 1
# RIGHT_TURN = 2
# CROSS_JUNCTION = 3
# T_JUNCTION = 4
# RIGHT_T_JUNCTION = 5
# LEFT_T_JUNCTION = 6
# X_HORIZONTAL_LINE = 7
# Y_VERTICAL_LINE = 8
# L_JUNCTION = 9
# MIRROR_L_JUNCTIO = 10
"""----------------------------------------------"""


class Serial_comms_TX_node(Node): #defines new node class, "Node" is the parent class, the class is inheriting form Node class
    def __init__(self):#constructor
        super().__init__("serial_bridge")#initializes parent class with node name, (super is used to access parent class)
        self.write_serial_port = serial_comms( 
            serial_port_ros,
            serial_baudrate,
            RECIEVE_SIZE,
            TRANSMIT_SIZE,
            "CRC",
        )#write_serial_port is an object of serial_comms class

        # self.read_serial_port = serial_comms(
        #     serial_port_stm,
        #     serial_baudrate,
        #     RECIEVE_SIZE,
        #     TRANSMIT_SIZE,
        #     "CRC",
        # )

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        # qos_profile.lifespan = Duration(seconds=0.05)

        # self.bno_publisher = self.create_publisher(BnoData, "bno_data", qos_profile) #Ros taking bno data thru disc
        #self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "cmd_robot_vel", self.send_cmd_vel_data, 10)#Float32MultiArray = message type, cmd_robot_vel = topic name

        self.imu_publisher = self.create_publisher(Imu, "imu_data", qos_profile) #Ros taking bno data thru disc

        # self.odom_publisher = self.create_publisher(Odometry, "odom_data", qos_profile)
        # self.sick_publisher = self.create_publisher(Float32MultiArray, "sick_data", qos_profile)
        self.sick_publisher = self.create_publisher(Float32MultiArray, "sick_data", 10)

        self.cmd_vel_msg = Float32MultiArray() #Float32MultiArray is a message class, whose instance is cmd_vel_msg
        self.cmd_vel_rx_flag = False
        self.cmd_vel_msg.data = [0.0, 0.0, 0.0]
        self.act_vel_msg = UInt8MultiArray()
        self.act_vel_rx_flag = False
        self.act_vel_msg.data = [0, 0, 0]

        self.timer_tick_count = 0

        # self.timer2 = self.create_timer(0.015, self.Send_Data_CallBack)
        self.timer1 = self.create_timer(0.001, self.serial_read_callback)

        self.rx_data = [0.0] * 24
        self.x_offset = 0.0
        self.y_offset = 0.0

        self.cmd_vel_last_rx_time = time.time()
        self.act_vel_last_rx_time = time.time()
        self.last_transmit_time = time.time()
        self.last_published_time = time.time()

        self.get_logger().info("Serial bridge readyH...")

    # Joystick read callback function
    def Send_Data_CallBack(self):
        now = time.time()
        if now - self.last_transmit_time < 0.01:
            return

        if now - self.cmd_vel_last_rx_time >= 0.3:
            self.cmd_vel_msg.data = [0.0, 0.0, 0.0]

        # self.cmd_vel_msg.data = [0.0, 0.0, 1.0]
        # self.act_vel_msg.data = [70, 0, 1]


        # if now - self.act_vel_last_rx_time >= 0.05:

        DataToSend = [ #Here the order of the data being sent is defined as well as their sizes and  types
            bytes(struct.pack("B", START_BYTE)),
            bytes(struct.pack("f", self.cmd_vel_msg.data[0])),
            # bytes(struct.pack("f", self.cmd_vel_msg.data[1])),
            # bytes(struct.pack("f", self.cmd_vel_msg.data[2])),
            # bytes(struct.pack("B", self.act_vel_msg.data[0])),
            # bytes(struct.pack("B", self.act_vel_msg.data[1])),
            # bytes(struct.pack("B", self.act_vel_msg.data[2])),
        ]
        DataToSend = b"".join(DataToSend)#converts the list of bytes into a single bytes object
        data_hash = self.calculate_crc(DataToSend[1:])#CRC calculation for error checking in the MCU side after it recieves the data
        DataToSend = [DataToSend, bytes(struct.pack("B", data_hash))]#appends the CRC byte to the data being sent
        DataToSend = b"".join(DataToSend)#convert the whole thing into a single bytes object
        # print(f"{data_hash =}")
        # print(f"{self.cmd_vel_msg.data[2]}")
        self.last_transmit_time = time.time()

        self.write_serial_port.write_data(DataToSend)
        self.cmd_vel_rx_flag = False
        self.act_vel_rx_flag = False

    def send_act_vel_data(self, act_vel_msg_: UInt8MultiArray):
        self.act_vel_msg.data[0] = act_vel_msg_.data[0]
        self.act_vel_msg.data[1] = act_vel_msg_.data[1]
        self.act_vel_msg.data[2] = act_vel_msg_.data[2]
        # print(
        #     f"act_vel: {act_vel_msg_.data[0]}, {act_vel_msg_.data[1]},{act_vel_msg_.data[2]}"
        # )
        self.act_vel_last_rx_time = time.time()

        self.act_vel_rx_flag = True

    def serial_read_callback(self):
        self.Send_Data_CallBack()
        _data = self.write_serial_port.read_data()
        if _data is not None:
            """data = [encoder_count_agg[0], encoder_count_agg[1], sick_1, sick_2, sick_3, sick_4,bno_yaw, prox, timestamp]""" #This format is correct

            #print(len(_data[0:-1]), _data)
            data = struct.unpack("<iiHHHHfBI", _data[0:-1])

            self.timer_tick_count += 1

            # if self.timer_tick_count >= 4:
                
            #     self.timer_tick_count = 0

            #     sick_x = data[2]  
            #     sick_y = data[3]

            #     # overwrite offsets so odom snaps to SICK reference
            #     self.x_offset = data[0] - sick_x
            #     self.y_offset = data[1] - sick_y

            #     self.get_logger().debug("Odom resynced using SICK")

            #print(f"Sensor Data: {data}")


            self.rx_data = data
            # self.process_odom(self.rx_data[0:2])
            # self.rx_data[1] = self.rx_data[1]

            self.process_sick(self.rx_data[2:6])
            self.process_imu(self.rx_data[6])
            now = time.time()

            
            delay = now - self.last_published_time
            if delay >= 0.02:
                print(f"{delay = }")
            # print(f"{data = }")

            self.last_published_time = time.time()



        # else:
        #     print("data none")

    """
    data:[pos_x, pose_y, theta, vel_x, vel_y, vel_z]
    """

    def process_odom(self, data):
        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = data[0] - self.x_offset
        odom_msg.pose.pose.position.y = data[1] - self.y_offset
        odom_msg.pose.pose.position.z = 0.0
        #qw, qx, qy, qz = self.rollpitchyaw_to_quaternion(0.0, 0.0, data[2])
        # odom_msg.pose.pose.orientation.w = qw
        # odom_msg.pose.pose.orientation.x = qx
        # odom_msg.pose.pose.orientation.y = qy
        # odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.030461,
        ]
        odom_msg.twist.twist.linear.x = data[0]
        odom_msg.twist.twist.linear.y = data[1]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        #odom_msg.twist.twist.angular.z = data[5]
        odom_msg.twist.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
        ]

        self.raw_odom_publisher.publish(odom_msg)
        odom_msg.pose.pose.position.x = data[0] - self.x_offset
        odom_msg.pose.pose.position.y = data[1] - self.y_offset
        self.odom_publisher.publish(odom_msg)

    """
    data: [yaw, pitch, roll, accel_x, accel_y,accel-z]
    """


    def process_imu(self, data): #process_imu function to publish imu data
        imu_msg = Imu()
        yaw_rad = data * M_PI / 180.0
        qw, qx, qy, qz = self.rollpitchyaw_to_quaternion(0.0, 0.0, yaw_rad)
        #  Assign header values to the imu_msg
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # Assign actuall values to the imu_msg
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # imu_msg.linear_acceleration.x = data[3]
        # imu_msg.linear_acceleration.y = data[4]
        # imu_msg.linear_acceleration.z = data[5]
        # imu_msg.orientation_covariance[0] = 0.00121847
        # imu_msg.orientation_covariance[1] = 0.0
        # imu_msg.orientation_covariance[2] = 0.0
        # imu_msg.orientation_covariance[3] = 0.0
        # imu_msg.orientation_covariance[4] = 0.00121847
        # imu_msg.orientation_covariance[5] = 0.0
        # imu_msg.orientation_covariance[6] = 0.0
        # imu_msg.orientation_covariance[7] = 0.0
        # imu_msg.orientation_covariance[8] = 0.00121847

        # imu_msg.angular_velocity_covariance[0] = 4.4944
        # imu_msg.angular_velocity_covariance[1] = 0.0
        # imu_msg.angular_velocity_covariance[2] = 0.0
        # imu_msg.angular_velocity_covariance[3] = 0.0
        # imu_msg.angular_velocity_covariance[4] = 4.4944
        # imu_msg.angular_velocity_covariance[5] = 0.0
        # imu_msg.angular_velocity_covariance[6] = 0.0
        # imu_msg.angular_velocity_covariance[7] = 0.0
        # imu_msg.angular_velocity_covariance[8] = 4.4944

        # imu_msg.linear_acceleration_covariance[0] = 0.25
        # imu_msg.linear_acceleration_covariance[1] = 0.0
        # imu_msg.linear_acceleration_covariance[2] = 0.0
        # imu_msg.linear_acceleration_covariance[3] = 0.0
        # imu_msg.linear_acceleration_covariance[4] = 0.25
        # imu_msg.linear_acceleration_covariance[5] = 0.0
        # imu_msg.linear_acceleration_covariance[6] = 0.0
        # imu_msg.linear_acceleration_covariance[7] = 0.0
        # imu_msg.linear_acceleration_covariance[8] = 0.25

        self.imu_publisher.publish(imu_msg)
        # print(f"{data[0]=},{data[1]=}, {data[5]=}")

    def landmark_update_callback(self, msg: Float64MultiArray):
        if msg.data[0] == X_HORIZONTAL_LINE:
            self.y_offset = msg.data[2]

        elif msg.data[0] == Y_VERTICAL_LINE:
            self.x_offset == msg.data[1]

        else:
            self.x_offset = msg.data[1]
            self.y_offset = msg.data[2]

        # self.get_logger().info("landmark_update_callback")
        print(f"offset_x:{self.x_offset}, offset_y:{self.y_offset}")
        # breakpoint()

    def calculate_checksum(self, data=[]):
        digest = int()
        for i in range(1, len(data)):
            digest += data[i]
        return int(digest)

    def calculate_crc(self, data=[]):
        hash_func = crc8()
        hash_func.update(data)
        return hash_func.digest()[0]

    def rollpitchyaw_to_quaternion(self, roll, pitch, yaw):
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

    def quaternion_to_rollpitchyaw(self, q):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
        cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q[3] * q[1] - q[0] * q[2]))
        cosp = math.sqrt(1 - 2 * (q[3] * q[1] - q[0] * q[2]))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        #  yaw (z-axis rotation)
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def process_sick(self, data):
        sick_msg = Float32MultiArray()

        
        data_tuned_0 =  0.31709677 * data[0] - 15.75917103
        data_tuned_1 =  0.29745752 * data[1] + 10.84057088
        data_tuned_2 =  0.28342641 * data[2] - 5.54270299
        data_tuned_3 =  0.30279794 * data[3] - 3.49987992
        sick_msg.data = [data_tuned_0, data_tuned_1, data_tuned_2, data_tuned_3]

        self.sick_publisher.publish(sick_msg)
        print(f"SICK data published: {sick_msg.data}")


def main(args=None):
    rclpy.init()
    node = Serial_comms_TX_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if hasattr(node, "write_serial_port"):
            node.write_serial_port.close()
        node.destroy_node()
        rclpy.try_shutdown()
        exit()


if __name__ == "__main":
    main()

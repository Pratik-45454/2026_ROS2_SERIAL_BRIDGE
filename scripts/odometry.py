import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import math

WHEEL_DIAMETER_M = 0.0574
COUNTS_PER_REV   = 4000
DIST_PER_COUNT   = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV
forward_offset_x = 0.185 #robo length = 54 so, offset x = (54/2) - 8.5 {deadwheel from edge} in cm
forward_offset_y = 0.187  #robo length = 60 so, offset x = (60/2) - 11.3 {deadwheel from edge} in cm


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.current_yaw = 0.0
        self.previous_yaw = 0.0
        self.x_prev_dead = 0.0
        self.y_prev_dead = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading_phi = 0.0

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT


        self._pub = self.create_publisher(Float32MultiArray, '/odom', qos_profile)
        self._sub = self.create_subscription(
            Int32MultiArray,
            '/odom_data_received',
            self.odom_callback,
            qos_profile,
        )
        self.imu_sub = self.create_subscription(
            Float32,
            '/imu_data_received',
            self.imu_callback,
            qos_profile,
        )

        # self.get_logger().info(
            # f'Ready. dist_per_count={DIST_PER_COUNT * 1000:.6f} mm/count'
        # )
    def imu_callback(self, msg: Float32):
        # self.get_logger().info(f"Received IMU data: {msg.data}")
        self.current_yaw = msg.data
        self.heading_phi = self.current_yaw 

    def normalizeAngle(self, angle):
        while (angle >  math.pi): angle -= 2.0 * math.pi
        while (angle < -math.pi): angle += 2.0 * math.pi
        return angle
    
    def odom_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f'Expected 2 values, got {len(msg.data)}')
            return
        
       
        x_curr_dead = float(-msg.data[0]) * DIST_PER_COUNT   
        y_curr_dead = float(-msg.data[1]) * DIST_PER_COUNT   


        
     
        delta_x_dead = x_curr_dead - self.x_prev_dead
        delta_y_dead = y_curr_dead - self.y_prev_dead

        # print("delta_x_dead: mm" ,delta_x_dead*1000)
        # print("delta_y_dead: mm" ,delta_y_dead*1000)

        

        delta_phi = 0.0
        if ((self.heading_phi < 0.0) != (self.current_yaw < 0.0)):
        
            e = self.current_yaw - self.heading_phi
            if (abs(e) > math.pi):  
                delta_phi = (2.0 * math.pi) - abs(e)
                if e > 0:
                    delta_phi = -delta_phi
            else:
                delta_phi = e
        else:
            delta_phi = self.current_yaw - self.heading_phi
        
       
        delta_x_ = delta_x_dead - forward_offset_x * delta_phi
        delta_y_ = delta_y_dead - forward_offset_y * delta_phi

        # print("delta_x_: mm" ,delta_x_*1000)
        # print("delta_y_: mm",delta_y_*1000)

        
        delta_x = delta_x_ * math.cos(self.heading_phi) - delta_y_ * math.sin(self.heading_phi)
        delta_y = delta_x_ * math.sin(self.heading_phi) + delta_y_ * math.cos(self.heading_phi)

        # print("delta_x: mm" ,delta_x*1000)
        # print("delta_y: mm",delta_y*1000)

        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading_phi += delta_phi
        self.heading_phi = self.normalizeAngle(self.heading_phi)

        print("x position, y position, heading: ", self.x_pos*1000, self.y_pos*1000, self.heading_phi)

        self.x_prev_dead = x_curr_dead
        self.y_prev_dead = y_curr_dead

        out = Float32MultiArray()
        out.data = [self.x_pos, self.y_pos, self.heading_phi]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
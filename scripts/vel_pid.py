import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from myserialbridge.pid import PID

kp_x = 1.0
ki_x = 0.1
kd_x = 0.05     

kp_y = 1.0
ki_y = 0.1
kd_y = 0.05

kp_w = 1.0
ki_w = 0.1
kd_w = 0.05

max_linear_speed = 1.0
max_angular_speed = 1.0
class VelPID(Node):
    def __init__(self):
        super().__init__('vel_pid')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pid_vx = PID(kp=kp_x, ki=ki_x, kd=kd_x, outMin=-1.0, outMax=1.0, sampleTime=100, _tol=0.01)
        self.pid_vy = PID(kp=kp_y, ki=ki_y, kd=kd_y, outMin=-1.0, outMax=1.0, sampleTime=100, _tol=0.01)
        self.pid_w = PID(kp=kp_w, ki=ki_w, kd=kd_w, outMin=-1.0, outMax=1.0, sampleTime=100, _tol=0.01)

        self.pid_vx.setTunings(kp_x, ki_x, kd_x)
        self.pid_vy.setTunings(kp_y, ki_y, kd_y)
        self.pid_w.setTunings(kp_w, ki_w, kd_w)

        self.pid_vx.setMode(PID.Mode.AUTOMATIC)
        self.pid_vy.setMode(PID.Mode.AUTOMATIC)
        self.pid_w.setMode(PID.Mode.AUTOMATIC)
        
        self.pid_vx.setOutputLimits(-max_linear_speed, max_linear_speed)
        self.pid_vy.setOutputLimits(-max_linear_speed, max_linear_speed)
        self.pid_w.setOutputLimits(-max_angular_speed, max_angular_speed)

        self._pub = self.create_publisher(Twist, '/adjusted_vel', qos_profile)
        self._sub_ref = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.receive_callback,
            qos_profile,
        )

        self._sub_input = self.create_subscription(
            Twist,
            '/current_vel',
            self.current_callback,
            qos_profile,
        )
        self.cmd_vel_rx : Twist = None
        self.cmd_vel_curr : Twist = None
        self.timer = self.create_timer(0.01, self.pid_callback)
        self.get_logger().info('VelPID node started')

    def current_callback(self, msg: Twist):
        self.cmd_vel_curr = msg
        self.get_logger().info(f'Received current_vel: linear_x={msg.linear.x}, linear_y={msg.linear.y}, angular_z={msg.angular.z}')
        return
    
    def receive_callback(self, msg: Twist):
        self.cmd_vel_rx = msg
        self.get_logger().info(f'Received cmd_vel: linear_x={msg.linear.x}, linear_y={msg.linear.y}, angular_z={msg.angular.z}')
        return

    def pid_callback(self, msg: Twist):
        if self.cmd_vel_rx is None or self.cmd_vel_curr is None:
            return
        self.pid_vx.setpoint = self.cmd_vel_rx.linear.x
        self.pid_vy.setpoint = self.cmd_vel_rx.linear.y
        self.pid_w.setpoint = self.cmd_vel_rx.angular.z

        self.pid_vx.input = self.cmd_vel_curr.linear.x
        self.pid_vy.input = self.cmd_vel_curr.linear.y 
        self.pid_w.input = self.cmd_vel_curr.angular.z
        
        self.pid_vx.compute()
        self.pid_vy.compute()  
        self.pid_w.compute()
        out_msg = Twist()
        out_msg.linear.x = self.pid_vx.output       
        out_msg.linear.y = self.pid_vy.output
        out_msg.angular.z = self.pid_w.output
        self._pub.publish(out_msg)
        self.get_logger().info(f'Received cmd_vel: linear_x={out_msg.linear.x}, linear_y={out_msg.linear.y}, angular_z={out_msg.angular.z}')



def main(args=None):
    rclpy.init(args=args)
    node = VelPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
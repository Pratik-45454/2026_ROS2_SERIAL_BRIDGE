import rclpy
from rclpy.node import Node
from gripper_msgs.msg import GripperCommand, GripperStatus
import threading
import time

MOVE_TIME = 2.0


class GripperSim(Node):
    def __init__(self):
        super().__init__('gripper_sim')

        self.status = GripperStatus()
        self.status_e = GripperStatus()

        self.status_a = GripperStatus()
        self.status_p = GripperStatus()


        self.status_pub_1 = self.create_publisher(GripperStatus, '/gripper_status_height', 10)
        self.status_pub_2 = self.create_publisher(GripperStatus, '/gripper_status_extension', 10)
        self.status_pub_3 = self.create_publisher(GripperStatus, '/gripper_status_act', 10)
        self.status_pub_4 = self.create_publisher(GripperStatus, '/gripper_status_pose', 10)

        self.create_timer(0.1, self.publish_status)  

        self.create_subscription(GripperCommand, '/gripper_height', self.got_height_command,    10)
        self.create_subscription(GripperCommand, '/gripper_length', self.got_extension_command, 10)
        self.create_subscription(GripperCommand, '/gripper_act',    self.got_act_command,       10)
        self.create_subscription(GripperCommand, '/gripper_yaw',    self.got_yaw_command,       10)

        self.get_logger().info('Gripper sim ready, i am waiting for commands...')

    def got_height_command(self, msg):
        self.get_logger().info(f'SetHeight received the target: {msg.height}')
        self.reset_flags()
        threading.Thread(target=self.finish_height).start()  

    def got_extension_command(self, msg):
        self.get_logger().info(f'SetExtension received the target: {msg.extension}')
        self.reset_flags()
        threading.Thread(target=self.finish_extension).start()

    def got_act_command(self, msg):
        self.get_logger().info(f'GripperAct received the target: {msg.grip_closed}')
        self.reset_flags()
        threading.Thread(target=self.finish_act).start()

    def got_yaw_command(self, msg):
        self.get_logger().info(f'GripperPose received the target: {msg.yaw}')
        self.reset_flags()
        threading.Thread(target=self.finish_yaw).start()

 

    def finish_height(self):
        time.sleep(MOVE_TIME)
        self.status.height_reached = True
        self.status.busy = False
        self.get_logger().info('SetHeight done!')

    def finish_extension(self):
        time.sleep(MOVE_TIME)
        self.status.length_reached = True
        self.status.busy = False
        self.get_logger().info('SetExtension done!')

    def finish_act(self):
        time.sleep(MOVE_TIME)
        self.status.object_grasped = True
        self.status.busy = False
        self.get_logger().info('GripperAct done!')

    def finish_yaw(self):
        time.sleep(MOVE_TIME)
        self.status.yaw_done = True
        self.status.busy = False
        self.get_logger().info('GripperPose done!')



    def reset_flags(self):
        self.status.height_reached = False
        self.status.length_reached = False
        self.status.object_grasped = False
        self.status.yaw_done       = False
        self.status.busy           = True

    def publish_status(self):
        self.status_pub_1.publish(self.status)
        self.status_pub_2.publish(self.status)
        self.status_pub_3.publish(self.status)
        self.status_pub_4.publish(self.status)



def main():
    rclpy.init()
    node = GripperSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
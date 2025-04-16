#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

class orien_to_yaw(Node):
    def __init__(self):
        super().__init__('yaw')
        self.create_subscription(Imu,'/imu',callback=self.calculate_yaw,qos_profile=10)
        self.orient=self.create_publisher(Float32,'/orient',10)
        self.yaw=Float32()

    def calculate_yaw(self,msg:Imu):
        _,_,yaw=euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        # self.get_logger().info(f"yaw: {yaw}")
        
        self.yaw.data=yaw-1.57
        self.get_logger().info(f"yaw: {yaw}")
        # if self.yaw.data<0.0:
        #     self.yaw.data+=6.24

        self.orient.publish(self.yaw)


def main(args=None):
    rclpy.init(args=args)
    node =orien_to_yaw()
    thread=MultiThreadedExecutor()
    thread.add_node(node)
    thread.spin()
    node.destroy_node()

if __name__=='__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# import ser_text.srv
from  custom_service.srv import Text
# from ser_text.srv import   
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
import math


class Texttomove(Node):
    def __init__(self):
        super().__init__('move')
        self.callback_group=ReentrantCallbackGroup()
        self.serv=self.create_service(Text,'/text_move',self.text_mv,callback_group=self.callback_group)
        self.scan=self.create_subscription(LaserScan,'/scan',self.get_data,10)
        self.pub_vel=self.create_publisher(Twist,'/cmd_vel',10)
        self.cmd_rev=False
        self.movement=False
 
    def give_range(self,degree,msg:LaserScan):
        rad=math.radians(degree)
        
        index=int(rad/msg.angle_increment)

        if 0<=index<=len(msg.ranges):
            # print(index)
            return msg.ranges[index]
        else:
            self.get_logger().info(f'Could not find index ')
            return None

    def get_data(self,msg:LaserScan):
        self.inc_dec=15
        # print(f'{len(msg.ranges)}.....')
        self.deg_30=self.give_range(0+ self.inc_dec,msg=msg)
        # print(self.deg_30)
        self.deg_330=self.give_range(360- self.inc_dec,msg=msg)
        self.deg_60=self.give_range(90- self.inc_dec,msg=msg)
        self.deg_120=self.give_range(90+ self.inc_dec,msg=msg)

        self.deg_210=self.give_range(180+ self.inc_dec,msg=msg)
        self.deg_150=self.give_range(180- self.inc_dec,msg=msg)
        self.deg_240=self.give_range(270- self.inc_dec,msg=msg)
        self.deg_300=self.give_range(270+ self.inc_dec,msg=msg)

        self.move_bot()




 
    


    def move_bot(self):
        if not self.movement:
            return
        
        phrase=self.word
        self.min_dist_left=0.5
        msg=Twist()
        if phrase=='left':
            
            print('left called ')
            if self.deg_60>=self.min_dist_left and self.deg_120>=self.min_dist_left:
                
                msg.angular.z=-0.5
                msg.linear.x=0.0
                
                self.movement=True
            else:
                msg.angular.z=0.0
                msg.linear.x=0.0
                self.get_logger().info(f'Close to object .. ')


        elif phrase=='forward' or phrase=='front':
            
            print('forward called ')
            if self.deg_30>=self.min_dist_left and self.deg_330>=self.min_dist_left:
                msg.angular.z=0.0
                msg.linear.x=0.5
                self.movement=True
            else:
                msg.angular.z=0.0
                msg.linear.x=0.0
                self.get_logger().info(f'Close to object .. Collision can occured ')

        elif phrase=='backward' or phrase=='back':
            print('backward called ')
            if self.deg_150>=self.min_dist_left and self.deg_210>=self.min_dist_left:
                msg.angular.z=0.0
                msg.linear.x=-0.5             
                self.movement=True
            else:
                msg.angular.z=0.0
                msg.linear.x=0.0

            self.get_logger().info(f'Close to object .. Collision can occured ')
    

        elif phrase=='right' :
           
            print('rightward called ')
            if self.deg_240>=self.min_dist_left and self.deg_300>=self.min_dist_left:     
                msg.angular.z=0.5
                msg.linear.x=0.0
                self.movement=True
            else:
                msg.angular.z=0.0
                msg.linear.x=0.0
                self.get_logger().info(f'Close to object .. Collision can occured ') 
        elif phrase=='stop':
            print('stop')
            
            msg.angular.z=0.0
            msg.linear.x=0.0
            self.movement=True 
        else:
            
            msg.angular.z=0.0
            msg.linear.x=0.0
            
            print('No Prompt')
        self.pub_vel.publish(msg) 



    def text_mv(self,Request,Response):
        self.word=Request.input
        self.movement=True
        
        rate=self.create_rate(10,self.get_clock())
        while not self.movement:
            # self.get_logger().info(f'Still moving to target ')
            rate.sleep()

        Response.output=True

        return Response
    

def main(args=None):
    rclpy.init(args=args)
    node =Texttomove()
    executor=MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
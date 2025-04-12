import rclpy 
from  rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math    
from rcl_interfaces.msg import Parameter, SetParametersResult

class map_bot(Node):
    def __init__(self):
        super().__init__('map')
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.reading=self.create_subscription(LaserScan,'/scan',self.get_read,10)
        self.min_dis=0.3
        self.get_logger().info(f'Node is Intialized')

        self.declare_parameter("control_kp",0.0)
        self.control_kp=self.get_parameter("control_kp").get_parameter_value().double_value

        self.declare_parameter("control_ki",0.0)
        self.control_ki=self.get_parameter("control_ki").get_parameter_value().double_value

        self.declare_parameter("control_kd",0.0)
        self.control_kd=self.get_parameter("control_kd").get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.param_call)
        self.integral=0.0
        self.preverror=0.0
        self.get_logger().info(f'PID control is intialized')
    
    def error_control(self,error):
        proportional=self.control_kp*error

        self.integral +=error
        integral=self.control_ki*self.integral

        derivative=self.control_kd*(error-self.preverror)
        self.preverror=error
        
        self.control=proportional+integral+derivative
        self.get_logger().info('Control Value is:' + str(self.control))
        return self.control
    def give_control(self):
        return self.control
    def param_call(self,params:list[Parameter]):
        for param in params:
            param_name=param.name
            param_value=param.value
            setattr(self,param_name,param_value)
        return SetParametersResult(successful=True)
    

    def get_read(self,msg:LaserScan):
        
        right_front=msg.ranges[int(13 * 3.14/(9 * msg.angle_increment))]
        
        right_back=msg.ranges[int(14 * 3.14/(9 *msg.angle_increment))]
        self.get_logger().info(f'Distance of angle 260: { right_front} and Distance of angle 280: {right_back}')
        error_dis= right_front-right_back
        
        self.get_logger().info(f'The error is as {error_dis}')
        control=self.error_control(error_dis)
        # control_state=self.give_control()

        self.adjust_bot(control)
        # if control_state == float("nan"):
        #     return

        # self.adjust_bot(control)
        
    def adjust_bot(self,control):
        twst_msg=Twist()


        twst_msg.angular.z = control

        self.pub.publish(twst_msg)

def main(args=None):
    rclpy.init(args=args)
    bot=map_bot()
    rclpy.spin(bot)
    rclpy.shutdown()

if __name__=='__main__':
    main()
import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        self.laser_sub = self.create_subscription(LaserScan,'scan',callback = self.switch_mode, qos_profile=10)
        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.timer = self.create_timer(0.1,callback=self.run_loop)
    
        self.mode = None

    def switch_mode(self,msg):

        # if person infront:
        #     mode = person follow
        # else:
        #     mode = wall_follow

        pass


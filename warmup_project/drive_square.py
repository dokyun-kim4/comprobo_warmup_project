import rclpy
from math import acos
from numpy import rad2deg 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom



class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')

        # subcribe to odometry node, calculate when to move & turn
        self.odom_sub = self.create_subscription(Odom,'odom',callback=self.square_draw,qos_overriding_options=10)

        # publish the lin_vel and ang_vel calculated from `square_draw`
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, callback = self.run_loop)

        self.lin_vel = 0
        self.ang_vel = 0

        self.start = {'x':0,'y':0}


    def get_angle(self,w):
        '''
        Given a quaterion orientation `w`, convert it to degrees
        '''
        return rad2deg(acos(w)*2)

    def square_draw(self,msg):
        '''
        Calculate lin_vel & ang_vel according to current position
        '''
        pass

        



        


    def run_loop(self):
        '''
        Publish the message
        '''
        pass
        
    


def main():
    rclpy.init()
    square_publisher = DriveSquare()
    rclpy.spin(square_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    square_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
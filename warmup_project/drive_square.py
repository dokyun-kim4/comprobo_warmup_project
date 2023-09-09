import rclpy
from math import acos,dist
from numpy import rad2deg 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom



class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')

        # subcribe to odometry node, calculate when to move & turn
        self.odom_sub = self.create_subscription(Odom,'odom',callback=self.square_draw,qos_profile=10)

        # publish the lin_vel and ang_vel calculated from `square_draw`
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, callback = self.run_loop)

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.prev_coord = [0,0] # (x,y)
        self.goal_angle = 90

        self.lin_goal_reached = False
        self.done = False


    def get_angle(self,w):
        '''
        Given a quaterion orientation `w`, convert it to degrees
        '''
        return rad2deg(acos(w)*2)

    def square_draw(self,msg):
        '''
        Calculate lin_vel & ang_vel according to current position
        '''
        crnt_coord = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        crnt_ang = self.get_angle(msg.pose.pose.orientation.w)
        
        if self.done:
            self.lin_vel = 0.0
            self.ang_vel = 0.0

        elif dist(self.prev_coord,crnt_coord) <= 1 and not self.lin_goal_reached:
            self.lin_vel = 0.2
        
        else: # If 1 meter reached
            self.lin_goal_reached = True
            self.lin_vel = 0.0 # Stop
            self.prev_coord = crnt_coord # Update prev_coord to crnt_coord
            # Turn until goal angle
            if crnt_ang <= self.goal_angle:
                self.lin_vel = 0.0
                self.ang_vel = 0.15
            else: # If goal angle reached
                print("I turned")
                self.ang_vel = 0.0 # Stop turning
                self.goal_angle += 90
                print(f'Next angle goal: {self.goal_angle}')
                self.lin_goal_reached = False

    def run_loop(self):
        '''
        Publish the message
        '''
        msg = Twist()
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel

        self.vel_pub.publish(msg)
        
    


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
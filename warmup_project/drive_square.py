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
        self.goal_angle = 88 # Using 89 instead of 90 for tolerance during rotation
        self.line_count = 0

        self.lin_goal_reached = False
        self.done = False

    def get_angle(self,w):
        '''
        Given a quaterion orientation `w`, convert it to degrees
        '''
        return rad2deg(acos(w)*2)

    def square_draw(self,msg):
        '''
        Determine when to move forward, turn, or stop.
        '''
        crnt_coord = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        crnt_ang = self.get_angle(msg.pose.pose.orientation.w)
        
        if self.line_count == 4: # Stop when 4 lines have been drawn
            self.lin_vel = 0.0
            self.ang_vel = 0.0

        elif dist(self.prev_coord,crnt_coord) <= 1 and not self.lin_goal_reached: # Move forward until 1 meter reached
            self.lin_vel = 0.2
        
        else: # Start turning when 1 meter reached
            self.lin_goal_reached = True
            self.lin_vel = 0.0 # Stop moving forward
            self.prev_coord = crnt_coord # Update prev_coord to crnt_coord
            # Turn until goal angle
            if crnt_ang <= self.goal_angle:
                print(f"goal: {self.goal_angle}, current: {crnt_ang}")
                self.lin_vel = 0.0
                self.ang_vel = 0.5
            else: # If goal angle reached
                self.ang_vel = 0.0 # Stop turning
                self.goal_angle += 90 # Update goal_angle
                self.lin_goal_reached = False
                self.line_count += 1

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
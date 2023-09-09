import rclpy
from math import dist,sqrt
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # subcribe to odometry node, calculate when to move & turn
        self.laser_sub = self.create_subscription(LaserScan,'scan',callback=self.get_data,qos_profile=10)

        # publish the lin_vel and ang_vel calculated from `square_draw`
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, callback = self.run_loop)
        
        self.dist_tolerance = 0.1 # meters
        self.angle_tolerance = 0.05 # percent
        self.dist_goal = 0.6
        self.dists = None

        self.fixing_angle = False

    def get_data(self,msg):
        '''
        Store neato's distance from wall @ 45,90,135 degrees and an approximation error value
        '''
        
        self.dists = {'deg45':msg.ranges[45],'deg90':msg.ranges[90],'deg135':msg.ranges[135]}
        print(self.dists)
        self.dists['pcnt_error'] = (self.dists['deg45']/sqrt(2) - self.dists['deg90'])/self.dists['deg90']
        
    

    def angle_check(self):
        '''
        Returns True if current angle from the wall is within the given percentage tolerance
        '''
        return abs(self.dists['pcnt_error']) <= self.angle_tolerance
    
    # def dist_check(self):
    #     '''
    #     pluh
    #     '''
    #     return self.dists['deg90'] >= self.dist_goal-self.dist_tolerance and self.dists['deg90'] <= self.dist_goal+self.dist_tolerance
        

    def run_loop(self):
        '''
        Publish the message
        '''
        msg = Twist()
        k = 2
        
        #print(self.angle_check)
        if self.dists:
            if self.angle_check():
                msg.angular.z = 0.0
                msg.linear.x = 0.3

            else:
                print("Turning")
                msg.linear.x = 0.0

                if self.dists['pcnt_error'] < 0:
                    msg.angular.z = -0.35 #+ abs(self.dists['pcnt_error']) * k
                else:
                    msg.angular.z = 0.35 #+ abs(self.dists['pcnt_error']) * k

        
        self.vel_pub.publish(msg)
        


def main():
    rclpy.init()
    wall_publisher = WallFollow()
    rclpy.spin(wall_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
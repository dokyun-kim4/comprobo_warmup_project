import rclpy
from math import dist,sqrt
from numpy import cos,sin, deg2rad
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker,MarkerArray


class WallFollow(Node):
    """
    A ROS2 node that directs the Neato to stay parallel to a wall while moving forward

    Attributes:
        laser_sub: ROS2 node that is subscribed to Neato's LaserScan data
        vel_pub: ROS2 node that publishes linear/angular velocity to Neato
        marker_pub1: ROS2 node that publishes a marker at the 45 degree point of the wall
        marker_pub2: ROS2 node that publishes a marker at the 90 degree point of the wall
        marker_pub3: ROS2 node that publishes a marker at the 135 degree point of the wall
        timer: ROS2 timer that governs loop timer
        angle_tolerance: Angle tolerance for determining if Neato is parallel (%)
        dists: Python dictionary containing distance from 45,90,135 degree points of laserscan
    
    Methods:
        pol2cart(rho,phi):
            Given a polar coordinate (rho,phi), convert to cartesian (x,y)
            Returns (x,y)
        get_data(msg):
            Given Neato's laserscan, isolate 45,90,135 degree points and store it in `dists`. Then, calculate how off the Neato is
            from being parallel with the wall
        check_angle():
            Check if Neato is currently parallel with the wall. Return True if parallel, False if not
    """
    def __init__(self):
        super().__init__('wall_follower')

        # subcribe to odometry node, calculate when to move & turn
        self.laser_sub = self.create_subscription(LaserScan,'scan',callback=self.get_data,qos_profile=10)
    

        # publish the lin_vel and ang_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub1 = self.create_publisher(Marker,'visualization_marker1',10)
        self.marker_pub2 = self.create_publisher(Marker,'visualization_marker2',10)
        self.marker_pub3 = self.create_publisher(Marker,'visualization_marker3',10)


        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, callback = self.run_loop)
        
        self.angle_tolerance = 0.045 # percent
        self.dists = None


    def pol2cart(self,rho, phi):
        x = rho * cos(deg2rad(phi))
        y = rho * sin(deg2rad(phi))
        return(x, y)       


    def get_data(self,msg):
        '''
        Store neato's distance from wall @ 45,135 degrees and an approximation error value
        '''        
        self.dists = {'deg45':msg.ranges[45],'deg90':msg.ranges[90],'deg135':msg.ranges[135]}
        self.dists['pcnt_error'] = (self.dists['deg45']-sqrt(2)*self.dists['deg90'])/(sqrt(2)*self.dists['deg90'])
        print(self.dists)

    def check_angle(self):
        '''
        Returns True if current angle from the wall is within the given percentage tolerance
        '''
        return abs(self.dists['pcnt_error']) <= self.angle_tolerance        

    def run_loop(self):
        '''
        Publish the message
        '''

        msg = Twist()
        if self.dists:
            keys = [45,90,135]
            markerlist = []
            for key in keys:
                marker = Marker()
                dist = self.dists["deg"+str(key)]
                x,y = self.pol2cart(dist,key)
                marker.header.frame_id = "base_link"
                marker.ns = "my_namespace"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.MODIFY
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0; # Don't forget to set the alpha!
                marker.color.r = 255.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                markerlist.append(marker)

        
            # markers = MarkerArray(markers=markerlist)
            self.marker_pub1.publish(markerlist[0])
            self.marker_pub2.publish(markerlist[1])
            self.marker_pub3.publish(markerlist[2])
           
                
        k = 0.4
        
        if self.dists:
            if self.check_angle():
                msg.angular.z = 0.0
                msg.linear.x = 0.2

            else:
                msg.linear.x = 0.0

                if self.dists['pcnt_error'] < 0:
                    msg.angular.z = -0.1 + (self.dists['pcnt_error']*k)
                else:
                    msg.angular.z = 0.1 + (self.dists['pcnt_error']*k)

        
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
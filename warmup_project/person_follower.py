import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg, deg2rad, cos,sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom
from visualization_msgs.msg import Marker


class PersonFollow(Node):
    """
    A ROS2 node that directs the Neato to follow a person if detected within its scan range

    Attributes:
        laser_sub: ROS2 node that is subscribed to Neato's LaserScan data
        odom_sub: ROS2 node that is subscribed to Neato's odometry data
        vel_pub: ROS2 node that publishes linear/angular velocity to Neato
        marker_pub: ROS2 node that publishes a marker at the center of detected person
        timer: ROS2 node that governs loop timer
        crnt_angle: Neato's current odometry angle (degrees)
        angle_to_turn: Amount the Neato needs to turn to face the person (degrees)
        following_distance: Distance from person the Neato tries to maintain (meters)
        reach_goal: Boolean indicating if Neato is within `following_distance`
        person: A list of points that make up the person

    Methods:
        get_angle(msg):
            Using the odometry data (msg), converts quaterion angle to degrees to get Neato's angular orientation
            Store to `crnt_angle'
        pol2cart(rho,phi):
            Given a polar coordinate (rho,phi), convert to cartesian (x,y)
            Returns (x,y)
        group_clusters(dists,max_gap):
            Given a list of tuples in form (angle,distance), identify clusters where each point is less than `max_gap` away from each other.
            Returns a list of lists where each inner list is a cluster
        closest_clusters(clusters,window):
            Given a list of clusters identified by `group_clusters()`, filter out clusters that are outside `window (meters)`.
            Then detect the largest cluster within the remaining clusters. The largest cluster is stored in `person`
        follow_person(msg):
            Using the Neato's laser scan, call `group_cluster` and `closest_clusters` to detect person. Then, calculate the appropritate 
            linear & angular velocity to follow the person.
    """
    def __init__(self):
        super().__init__('person_follower')

        self.laser_sub = self.create_subscription(LaserScan,'scan',callback = self.follow_person,qos_profile=10)
        self.odom_sub = self.create_subscription(Odom,'odom',callback=self.get_angle,qos_profile=10)

        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.marker_pub = self.create_publisher(Marker,'person_marker',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, callback = self.run_loop)
        
        self.crnt_angle = None
        self.angle_to_turn = None
        self.following_distance = 0.4
        self.reach_goal = None
        self.person = None

    def get_angle(self,msg):
        w = msg.pose.pose.orientation.w
        self.crnt_angle = rad2deg(acos(w)*2)

    def pol2cart(self,rho, phi):
        x = rho * cos(deg2rad(phi))
        y = rho * sin(deg2rad(phi))
        return(x, y)      

    def group_clusters(self,dists,max_gap):
        """
        Given a lidar scan of points, return a list of lists where each inner list is a cluster of points
        Each point in the cluster is represented as a tuple in the form: (angle,distance)
        """
        clusters = []
        crnt_cluster = [dists[1]] # Start with first tuple in the list
        prev_dist = dists[0][1] # The first distance in the list

        for dist in dists[1::]:
            if dist != 0.0 and abs(dist[1] - prev_dist) <= max_gap: # If crnt point is within tolerable gap distance, add it to crnt cluster
                print(dist)
                crnt_cluster.append(dist)
            else: # If 
                clusters.append(crnt_cluster)
                crnt_cluster = [dist]
            prev_dist = dist[1]
        
        if crnt_cluster != []:
             clusters.append(crnt_cluster)  
        return clusters 
    
    def closest_cluster(self,clusters,window):#change to max_dist
        """
        Return largest cluster in the specified window
        """
        min_size = 4
        crnt_largest = []
        for cluster in clusters:
            if cluster[0][1] <= window: # Is the cluster in scan window?
                # If it is:
                if len(cluster) >= min_size: # Is it larger than specified minimum size?
                    # If it is:
                    if len(cluster) > len(crnt_largest): # Is it larger than the currently largest cluster?
                        # If it is:
                        crnt_largest = cluster # Currently largest cluster gets updated

        return crnt_largest


    def follow_person(self,msg):
        """
        Given a lidar scan of Neato, identify which angle to turn to
        """
        ranges = [i for i in range(361)] # 0 to 360

        neato_angles = ranges[45:0:-1]+[0]+ranges[359:314:-1] # 45~0, then 0~315
        dists = list(msg.ranges[45:0:-1])+[0]+list(msg.ranges[359:314:-1]) # Extract angle-appropriate lidar values

        # Combine it to a list of tuples where each tuple is (angle,distance)
        angle_and_dists = list(map(lambda x, y:(x,y), neato_angles, dists)) 

        # Group lidar readings into clusters
        clusters = self.group_clusters(angle_and_dists,max_gap=0.5)

        # Identify the person (largest closest cluster)
        self.person = self.closest_cluster(clusters,window=1.5)

        # You can print `person` to see the range of angles & distance that the person is in
        # print(person)

        # The robot should turn towards the middle of the object, so `angle_to_turn` is the median angle from the list `person`
        if len(self.person) > 0:
            self.angle_to_turn = self.person[len(self.person) // 2][0]

            if self.person[len(self.person) // 2][1] <= self.following_distance:
                self.reach_goal = True
            else:
                self.reach_goal = False
        else:
            self.angle_to_turn = 0

    
    def run_loop(self):
        msg = Twist() 
        if self.person:
            marker = Marker()
            com = self.person[len(self.person) // 2]
            x,y = self.pol2cart(com[1],com[0])
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
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0; # Don't forget to set the alpha!
            marker.color.r = 255.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)

        if self.angle_to_turn: # If not None
            if self.angle_to_turn >= 315: # clockwise
                msg.angular.z = -0.4 #* (1 / (self.angle_to_turn - 270) / 45)
            elif self.angle_to_turn <=45: # counterclockwise
                msg.angular.z = 0.4 #* (self.angle_to_turn / 45)
            if self.reach_goal is False:
                msg.linear.x = 0.2
        self.vel_pub.publish(msg)

def main():
    rclpy.init()
    person_publisher = PersonFollow()
    rclpy.spin(person_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    person_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
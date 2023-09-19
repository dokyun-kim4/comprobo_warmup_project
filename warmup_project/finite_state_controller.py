import rclpy
from rclpy.node import Node
from math import acos, sqrt
from numpy import rad2deg, deg2rad, cos,sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom
from visualization_msgs.msg import Marker

class FiniteStateController(Node):
    """
    A ROS2 node that directs the Neato to follow a person if detected within its scan range, follow a wall if no person was detected

    Attributes:
        laser_sub_person: ROS2 node that is subscribed to Neato's LaserScan data, used to detect person
        laser_sub_wall: ROS2 node that is subscribed to Neato's LaserScan data, used to detect wall
        odom_sub_person: ROS2 node that is subscribed to Neato's odometry data
        vel_pub: ROS2 node that publishes linear/angular velocity to Neato
        timer: ROS2 timer that governs loop timer
        mode: String representing current mode ('wall-follow' or 'person-follow')
        lin_vel: Neato's linear velocity
        ang_vel: Neato's angular velocity
        crnt_angle: Neato's current odometry angle (degrees)
        angle_to_turn: Amount the Neato needs to turn to face the person (degrees)
        following_distance: Distance from person the Neato tries to maintain (meters)
        reach_goal: Boolean indicating if Neato is within `following_distance`
        window: Scan radius (m)
        person: A list of points that make up the person
        angle_tolerance: Angle tolerance for determining if Neato is parallel (%)
        dists: Python dictionary containing distance from 45,90,135 degree points of laserscan

    Methods:
        get_wall(msg):
            Given Neato's laserscan, isolate 45,90,135 degree points and store it in `dists`. Then, calculate how off the Neato is
            from being parallel with the wall
        check_angle():
            Check if Neato is currently parallel with the wall. Return True if parallel, False if not
        follow_wall():
            Based on how unaligned the Neato is, determine the appropriate linear & angular velocity
        get_angle(msg):    def pol2cart(self,rho, phi):
    x = rho * cos(deg2rad(phi))
    y = rho * sin(deg2rad(phi))
    return(x, y)     n form (angle,distance), identify clusters where each point is less than `max_gap` away from each other.
            Returns a list of lists where each inner list is a cluster
        closest_clusters(clusters,window):
            Given a list of clusters identified by `group_clusters()`, filter out clusters that are outside `window (meters)`.
            Then detect the largest cluster within the remaining clusters. The largest cluster is stored in `person`
        follow_person(msg):
            Using the Neato's laser scan, call `group_cluster` and `closest_clusters` to detect person. Then, calculate the appropritate 
            linear & angular velocity to follow the person.
    """
    def __init__(self):
        super().__init__('finite_state_controller')

        self.laser_sub_person = self.create_subscription(LaserScan,'scan',callback = self.person_infront, qos_profile=10)
        self.laser_sub_wall = self.create_subscription(LaserScan,'scan', callback= self.get_wall, qos_profile=10)
        self.odom_sub_person = self.create_subscription(Odom, 'odom', callback= self.get_angle, qos_profile=10)

        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.marker_pub1 = self.create_publisher(Marker,'visualization_marker1',10)
        self.marker_pub2 = self.create_publisher(Marker,'visualization_marker2',10)
        self.marker_pub3 = self.create_publisher(Marker,'visualization_marker3',10)
        self.marker_pub4 = self.create_publisher(Marker,'visualization_marker4',10)
        self.timer = self.create_timer(0.1,callback=self.run_loop)

        # ---------- FSM variables -------------------------------
        self.mode = None
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        #---------------------------------------------------------

        # ---------- person follower variables -------------------
        self.crnt_angle = None
        self.angle_to_turn = None
        self.following_distance = 1
        self.reach_goal = None
        self.window = 4 # scan area (m)
        self.person = None
        # --------------------------------------------------------

        # ---------- wall follower variables ---------------------
        self.angle_tolerance = 0.045 # percent
        self.dist_goal = 0.6
        self.dists = None
        self.fixing_angle = False
        # --------------------------------------------------------

    def pol2cart(self,rho, phi):
        x = rho * cos(deg2rad(phi))
        y = rho * sin(deg2rad(phi))
        return(x, y)      
    # ---------------------------------------- Wall follower functions -------------------------------------------------#
    def get_wall(self,msg):
        '''
        Store neato's distance from wall @ 45,135 degrees and an approximation error value
        '''
        self.dists = {'deg45':msg.ranges[45],'deg90':msg.ranges[90],'deg135':msg.ranges[135]}
        self.dists['pcnt_error'] = (self.dists['deg45']-sqrt(2)*self.dists['deg90'])/(sqrt(2)*self.dists['deg90'])

    def check_angle(self):
        '''
        Returns True if current angle from the wall is within the given percentage tolerance
        '''
        return abs(self.dists['pcnt_error']) <= self.angle_tolerance
    
    def follow_wall(self):
        k = 0.4
        
        if self.dists:
            if self.check_angle():
                self.ang_vel = 0.0
                self.lin_vel = 0.1

            else:
                self.lin_vel = 0.0

                if self.dists['pcnt_error'] < 0:
                    self.ang_vel = -0.1 + (self.dists['pcnt_error']*k)
                else:
                    self.ang_vel = 0.1 + (self.dists['pcnt_error']*k)
    #--------------------------------------------------------------------------------------------------------------------#

    # ---------------------------------------- Person follower functions ----------------------------------------------- #
    def get_angle(self,msg):
        w = msg.pose.pose.orientation.w
        self.crnt_angle = rad2deg(acos(w)*2)

    def person_infront(self,msg):
        ranges = [i for i in range(361)] # 0 to 360

        neato_angles = ranges[45:0:-1]+[0]+ranges[359:314:-1] # 45~0, then 0~315
        dists = list(msg.ranges[45:0:-1])+[0]+list(msg.ranges[359:314:-1]) # Extract angle-appropriate lidar values

        # Combine it to a list of tuples where each tuple is (angle,distance)
        angle_and_dists = list(map(lambda x, y:(x,y), neato_angles, dists)) 

        # Group lidar readings into clusters
        clusters = self.group_clusters(angle_and_dists,max_gap=0.5)

        # Identify the person (largest closest cluster)
        self.person = self.closest_cluster(clusters,max_window=4,min_window=1)
        if self.person:
            self.mode = "person-follow"
            
        else:
            self.mode = "wall-follow"
        print(self.person,self.mode)

    def group_clusters(self,dists,max_gap):
        """
        Given a lidar scan of points, return a list of lists where each inner list is a cluster of points
        Each point in the cluster is represented as a tuple in the form: (angle,distance)
        """
        clusters = []
        crnt_cluster = [dists[1]] # Start with first tuple in the list
        prev_dist = dists[0][1] # The first distance in the list

        for dist in dists[1::]:
            if abs(dist[1] - prev_dist) <= max_gap: # If crnt point is within tolerable gap distance, add it to crnt cluster
                crnt_cluster.append(dist)
            else: # If 
                clusters.append(crnt_cluster)
                crnt_cluster = [dist]
            prev_dist = dist[1]
        
        if crnt_cluster != []:
             clusters.append(crnt_cluster)  
        return clusters 
    
    def closest_cluster(self,clusters,min_window, max_window):#change to max_dist
        """
        Return largest cluster in the specified window
        """
        min_size = 4
        max_size = 44
        crnt_largest = []
        for cluster in clusters:
            if cluster[-1][1] <= max_window and cluster[-1][1] >= min_window: # Is the cluster in scan window?
                # If it is:
                if len(cluster) >= min_size and len(cluster) <= max_size: # Is it larger than specified minimum size?
                    # If it is:
                    if len(cluster) > len(crnt_largest): # Is it larger than the currently largest cluster?
                        # If it is:
                        crnt_largest = cluster # Currently largest cluster gets updated

        return crnt_largest
    
    def follow_person(self):
        if len(self.person) > 0:
            self.angle_to_turn = self.person[len(self.person) // 2][0]

            if self.person[len(self.person) // 2][1] <= self.following_distance:
                self.reach_goal = True
            else:
                self.reach_goal = False
        else:
            self.angle_to_turn = 0
        
        if self.angle_to_turn: # If not None
            if self.angle_to_turn >= 315: # clockwise
                self.ang_vel = -0.4 #* (1 / (self.angle_to_turn - 270) / 45)
            elif self.angle_to_turn <=45: # counterclockwise
                self.ang_vel = 0.4 #* (self.angle_to_turn / 45)
            if self.reach_goal is False:
                self.lin_vel = 0.1

    #----------------------------------------------------------------------------------------------------------------------#




    def run_loop(self):
        msg = Twist()
        if self.mode == "wall-follow":
            self.follow_wall()
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

        elif self.mode == "person-follow":
            self.follow_person()
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
                marker.color.r = 0.0
                marker.color.g = 255.0
                marker.color.b = 0.0
                self.marker_pub4.publish(marker)
        else:
            self.lin_vel = 0.0
            self.ang_vel = 0.0

        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.vel_pub.publish(msg)

def main():
    rclpy.init()
    fsm_publisher = FiniteStateController()
    rclpy.spin(fsm_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fsm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
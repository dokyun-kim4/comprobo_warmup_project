import rclpy
from rclpy.node import Node
from math import acos, sqrt
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        self.laser_sub_person = self.create_subscription(LaserScan,'scan',callback = self.person_infront, qos_profile=10)
        self.laser_sub_wall = self.create_subscription(LaserScan,'scan', callback= self.get_wall, qos_profile=10)

        self.odom_sub_person = self.create_subscription(Odom, 'odom', callback= self.get_angle, qos_profile=10)
        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.timer = self.create_timer(0.1,callback=self.run_loop)

        # ---------- FSM variables -------------------------------
        self.mode = None

        #---------------------------------------------------------

        # ---------- person follower variables -------------------
        self.crnt_angle = None
        self.angle_to_turn = None
        self.following_distance = 1
        self.reach_goal = None
        self.window = 4 # scan area (m)
        # --------------------------------------------------------

        # ---------- wall follower variables ---------------------
        self.angle_tolerance = 0.045 # percent
        self.dist_goal = 0.6
        self.dists = None
        self.fixing_angle = False
        # --------------------------------------------------------

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
        person = self.closest_cluster(clusters,window=2)
        if person:
            self.mode = "person-follow"
        else:
            self.mode = "wall-follow"
        
        print(person,self.mode)

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
    
    def closest_cluster(self,clusters,window):#change to max_dist
        """
        Return largest cluster in the specified window
        """
        min_size = 4
        crnt_largest = []
        for cluster in clusters:
            if cluster[-1][1] <= window: # Is the cluster in scan window?
                # If it is:
                if len(cluster) >= min_size: # Is it larger than specified minimum size?
                    # If it is:
                    if len(cluster) > len(crnt_largest): # Is it larger than the currently largest cluster?
                        # If it is:
                        crnt_largest = cluster # Currently largest cluster gets updated

        return crnt_largest
    #----------------------------------------------------------------------------------------------------------------------#




    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
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
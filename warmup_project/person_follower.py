import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.laser_sub = self.create_subscription(LaserScan,'scan',callback = self.follow_person,qos_profile=10)
        self.odom_sub = self.create_subscription(Odom,'odom',callback=self.get_angle,qos_profile=10)

        self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, callback = self.run_loop)
        
        self.crnt_angle = None
        self.angle_to_turn = None
        self.following_distance = 1
        self.reach_goal = None

    
    def get_angle(self,msg):
        w = msg.pose.pose.orientation.w
        self.crnt_angle = rad2deg(acos(w)*2)


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
        person = self.closest_cluster(clusters,window=4)

        # You can print `person` to see the range of angles & distance that the person is in
        # print(person)

        # The robot should turn towards the middle of the object, so `angle_to_turn` is the median angle from the list `person`
        if len(person) > 0:
            self.angle_to_turn = person[len(person) // 2][0]

            if person[len(person) // 2][1] <= self.following_distance:
                self.reach_goal = True
            else:
                self.reach_goal = False
        else:
            self.angle_to_turn = 0

    
    def run_loop(self):
        msg = Twist() 
        if self.angle_to_turn: # If not None
            if self.angle_to_turn >= 315: # clockwise
                msg.angular.z = -0.6 * (1 / (self.angle_to_turn - 270) / 45)
            elif self.angle_to_turn <=45: # counterclockwise
                msg.angular.z = 0.6 * (self.angle_to_turn / 45)
            if self.reach_goal is False:
                msg.linear.x = 0.2
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
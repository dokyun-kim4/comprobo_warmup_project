import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom
from time import sleep


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.laser_sub = self.create_subscription(
            LaserScan, "scan", callback=self.get_laser, qos_profile=10
        )

        self.odom_sub = self.create_subscription(
            Odom, "odom", callback=self.get_angle, qos_profile=10
        )

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, callback=self.run_loop)

        self.current_angle = None
        self.dists = {
            "deg0": 1.5,
            "deg90": 2.0,
            "deg270": 2.0,
        }

        self.obstacle_detect = False
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.obstacle_threshold = 1.0
        self.ang_direction = None
        self.task_finish = True
        self.goal_angle = 85
        self.avoid_obstacle = False
        self.rotated = False
        self.turn_reset = False

    def get_angle(self, msg):
        """
        Get current neato angle
        """
        self.current_angle = rad2deg(acos(msg.pose.pose.orientation.w) * 2)
        print(self.current_angle)

    def get_laser(self, msg):
        """
        Get current neato laser data
        """
        self.dists = {
            "deg0": msg.ranges[0],
            "deg90": msg.ranges[90],
            "deg270": msg.ranges[270],
        }

    def check_front_obstacle(self):
        """
        Check for obstacle in front of neato
        """
        if self.dists["deg0"] > self.obstacle_threshold or self.dists["deg0"] == 0.0:
            self.obstacle_detect = False

        else:
            self.obstacle_detect = True
            print("obstacle")

    def compare_side_dist(self):
        """
        Check for obstacle to the left and right of the neato
        """
        if self.dists["deg90"] >= self.dists["deg270"]:
            self.ang_direction = 1
        else:
            self.ang_direction = -1

    def rotate(self):
        if self.current_angle >= self.goal_angle:
            self.ang_vel = 0.0
            print("turned")
            self.rotated = True
        else:
            self.ang_vel = 0.3 * self.ang_direction

    def rotate_reset(self):
        if self.current_angle <= 1 + self.goal_angle:
            self.ang_vel = 0.0

    def obstacle_parallel(self):
        if self.ang_direction == 1:
            parallel_angle = self.dists["deg270"]
        else:
            parallel_angle = self.dists["deg90"]
        if parallel_angle > self.obstacle_threshold:
            self.ang_vel = -0.6
            self.lin_vel = 0.05
            self.goal_angle = 0.0
            self.avoid_obstacle = True
        else:
            self.lin_vel = 0.3
            self.ang_vel = 0.0
            self.avoid_obstacle = False

    def run_loop(self):
        """
        Executes the Node runtime loop and publishes the "cmd_vel" topic
        """
        msg = Twist()
        # first check in front of neato
        if self.obstacle_detect is False:
            self.check_front_obstacle()
            self.lin_vel = 0.3
        if self.obstacle_detect is True:
            self.lin_vel = 0.0
            self.rotated = False
            if self.task_finish is True:
                self.compare_side_dist()
                print(self.ang_direction)
                self.task_finish = False

            if self.task_finish is False:
                if self.avoid_obstacle is False:

                    if self.rotated is False:
                        self.rotate()
                    if self.current_angle >= self.goal_angle:
                        self.lin_vel = 0.3
                        print("i have turned")
                        self.obstacle_parallel()
                else:
                    if self.turn_reset is False:
                        self.rotate_reset()
                        self.lin_vel = 0.3
                        self.task_finish = True

        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.vel_pub.publish(msg)


def main():
    rclpy.init()
    obstacle_publisher = ObstacleAvoider()
    rclpy.spin(obstacle_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
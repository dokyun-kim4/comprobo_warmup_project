import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom


class ObstacleAvoider(Node):
    """
    A ros2 Node which directs a Neato to avoid obstacles in its path.

    Attributes:
        laser_sub - subscription node
            A node which subscribes to the 'laser' topic.
        odom_sub - subscription node
            A node which subscribes to the 'odom' topic.
        vel_sub - publisher node
            A node which publishes the 'cmd_vel' topic.
        timer - timer node
            A node which governs ObstacleAvoider's loop time.
        current_angle - float
            A variable which stores the Neato's current angle while it runs.
        dists - dictionary
            A dictionary representing the Neato's current distance from objects
            directly in front of it as well as 90 degrees to the right and left.
        obstacle_detect - boolean
            A boolean indicating whether an obstacle has been detected.
        lin_vel - float
            A float representing the Neato's linear velocity.
        ang_vel - float
            A float representing the Neato's angular velocity.
        obstacle_threshold - float
            A float representing the maximum distance an obstacle can
            be from a Neato before it detects it as an obstacle.
        ang_direction - integer
            An integer indicating the direction a Neato should turn when
            avoiding an obstacle.
        turn_determine - boolean
            A boolean indicating if the Neato has yet to determine which direction to turn.
        goal_angle = integer
            An integer representing the ideal angle the Neato should turn whenever it
            rotates.
        avoid_obstacle - boolean
            A boolean indicating if the Neato has finished avoiding the obstacle.
        rotated - boolean
            A boolean to check if the Neato has completed its first rotation.
        turn_reset - boolean
            A boolean to check if the Neato has completed rotating to its original
            orientation.

    Methods:
        get_angle(self, msg)
                Get the current angle of the Neato in degrees.
            Args:
                msg - subscriber node
                    Calls the odom_sub node to sample the Neato's
                    current angle.

        get_laser(self, msg)
                Get the measured distances from Lidar scan angles
                0, 90, and 270.
            Args:
                msg - subscriber node
                    Calls the laser_sub node to sample the Neato's
                    current laser scan data.

        check_front_obstacle(self)
                Checks for obstacles directly in front of the Neato.
                If an obstacle is detected, self.obstacle_detect equals
                True, otherwise it equals False.

        compare_side_dist(self)
                Compares the Neato's distance to obstacles on its direct
                left and right. Based on the result, self.ang_direction
                is either set to 1 or -1.

        rotate(self)
                Checks if the Neato has reached goal angle while rotating.
                If the goal is not reached, the Neato continues to rotate.
                Otherwise, the Neato's rotation is stopped.

        rotate_reset(self)
                Checks if the Neato has reset its rotation to its original
                orientation. If it has, ang_vel is set to 0 and turn_reset
                is set to True.

        obstacle_parallel(self)
                Drives the Neato parallel to the obstacle it is avoiding
                until the Neato has cleared the obstacle. Upon clearing the
                obstacle, the Neato slows down and rotates around the obstacle.
                Once the obstacle has been avoided, avoid_obstacle is set to True.

        run_loop(self)
                Executes the Node runtime loop and publishes the 'cmd_vel' topic.
                While running the loop, run_loop also sets several conditionals to
                ensure that only one portion of the loop is running at a given time.
                This allows the various behaviors of the Neato to occur sequentially.
    """

    def __init__(self):
        """
        Constructs all required attributes for the ObstacleAvoider Node

        Parameters:
            laser_sub - subscription node
            A node which subscribes to the 'laser' topic.
        odom_sub - subscription node
            A node which subscribes to the 'odom' topic.
        vel_sub - publisher node
            A node which publishes the 'cmd_vel' topic.
        timer - timer node
            A node which governs ObstacleAvoider's loop time.
        current_angle - float
            A variable which stores the Neato's current angle while it runs.
        dists - dictionary
            A dictionary representing the Neato's current distance from objects
            directly in front of it as well as 90 degrees to the right and left.
        obstacle_detect - boolean
            A boolean indicating whether an obstacle has been detected.
        lin_vel - float
            A float representing the Neato's linear velocity.
        ang_vel - float
            A float representing the Neato's angular velocity.
        obstacle_threshold - float
            A float representing the maximum distance an obstacle can
            be from a Neato before it detects it as an obstacle.
        ang_direction - integer
            An integer indicating the direction a Neato should turn when
            avoiding an obstacle.
        turn_determine - boolean
            A boolean indicating if the Neato has yet to determine which direction to turn.
        goal_angle = integer
            An integer representing the ideal angle the Neato should turn whenever it
            rotates.
        avoid_obstacle - boolean
            A boolean indicating if the Neato has finished avoiding the obstacle.
        rotated - boolean
            A boolean to check if the Neato has completed its first rotation.
        turn_reset - boolean
            A boolean to check if the Neato has completed rotating to its original
            orientation.
        """
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
        self.turn_determine = True
        self.goal_angle = 85
        self.avoid_obstacle = False
        self.rotated = False
        self.turn_reset = False

    def get_angle(self, msg):
        """
            Get the current angle of the Neato in degrees.
        Args:
            msg - subscriber node
                Calls the odom_sub node to sample the Neato's
                current angle.
        """
        self.current_angle = rad2deg(acos(msg.pose.pose.orientation.w) * 2)
        print(self.current_angle)

    def get_laser(self, msg):
        """
        Get the measured distances from Lidar scan angles
            0, 90, and 270.
        Args:
            msg - subscriber node
                Calls the laser_sub node to sample the Neato's
                current laser scan data.
        """
        self.dists = {
            "deg0": msg.ranges[0],
            "deg90": msg.ranges[90],
            "deg270": msg.ranges[270],
        }

    def check_front_obstacle(self):
        """
        Checks for obstacles directly in front of the Neato.
        If an obstacle is detected, self.obstacle_detect equals
        True, otherwise it equals False.
        """
        if self.dists["deg0"] > self.obstacle_threshold or self.dists["deg0"] == 0.0:
            self.obstacle_detect = False

        else:
            self.obstacle_detect = True
            print("obstacle")

    def compare_side_dist(self):
        """
        Compares the Neato's distance to obstacles on its direct
        left and right. Based on the result, self.ang_direction
        is either set to 1 or -1.
        """
        if self.dists["deg90"] >= self.dists["deg270"]:
            self.ang_direction = 1
        else:
            self.ang_direction = -1

    def rotate(self):
        """
        Checks if the Neato has reached goal angle while rotating.
        If the goal is not reached, the Neato continues to rotate.
        Otherwise, the Neato's rotation is stopped.
        """
        if self.current_angle >= self.goal_angle:
            self.ang_vel = 0.0
            print("turned")
            self.rotated = True
        else:
            self.ang_vel = 0.3 * self.ang_direction

    def rotate_reset(self):
        """
        Checks if the Neato has reset its rotation to its original
        orientation. If it has, ang_vel is set to 0 and turn_reset
        is set to True.
        """
        if self.current_angle <= 1 + self.goal_angle:
            self.ang_vel = 0.0
            self.turn_reset = True

    def obstacle_parallel(self):
        """
        Drives the Neato parallel to the obstacle it is avoiding
        until the Neato has cleared the obstacle. Upon clearing the
        obstacle, the Neato slows down and rotates around the obstacle.
        Once the obstacle has been avoided, avoid_obstacle is set to True.
        """
        if self.ang_direction == 1:
            parallel_angle = self.dists["deg270"]
        else:
            parallel_angle = self.dists["deg90"]
        if parallel_angle > self.obstacle_threshold:
            self.ang_vel = -0.6 * self.ang_direction
            self.lin_vel = 0.05
            self.goal_angle = 5.0
            self.avoid_obstacle = True
        else:
            self.lin_vel = 0.3
            self.ang_vel = 0.0
            self.avoid_obstacle = False

    def run_loop(self):
        """
        Executes the Node runtime loop and publishes the 'cmd_vel' topic.
        While running the loop, run_loop also sets several conditionals to
        ensure that only one portion of the loop is running at a given time.
        This allows the various behaviors of the Neato to occur sequentially.
        """
        msg = Twist()
        # first check in front of neato
        if self.obstacle_detect is False:  # self.obstacle_detect is false
            self.check_front_obstacle()
            self.lin_vel = 0.3
        if self.obstacle_detect is True:  # self.obstacle_detect is true
            self.lin_vel = 0.0
            self.rotated = False  # self.rotated is false
            if self.turn_determine is True:  # self.task_finished is initially true
                self.compare_side_dist()
                print(self.ang_direction)
                self.turn_determine = False  # self.task_finish is set to false. Not needed to initialize above lines.

            if self.turn_determine is False:
                if self.avoid_obstacle is False:  # self.avoid_obstacle is false

                    if self.rotated is False:  # self.rotated is initially False
                        # self.rotate()  # self.rotated is set to True -> needs to be reset at bottom
                        self.ang_vel = 0.3 * self.ang_direction
                    if self.current_angle >= self.goal_angle:
                        self.lin_vel = 0.3
                        print("i have turned")
                        self.obstacle_parallel()  # once done, self.avoid_obstacle is set to True, needs to get reset at bottom
                elif self.avoid_obstacle is True and self.turn_reset is False:
                    self.rotate_reset()
                    self.lin_vel = 0.3
                if self.turn_reset is True:
                    self.__init__()

        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.vel_pub.publish(msg)


def main():
    """
    Initializes and publishes the ObstacleAvoider Node
    """
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
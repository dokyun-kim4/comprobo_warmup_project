import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import tty
import select
import sys
import termios

DIRECTION = {
        'w':0.3,
        'a':1.0,
        's':-0.3,
        'd':-1.0,
        'x':0.0}


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, callback = self.run_loop)
    

    def run_loop(self):
        msg = Twist()
        def getKey():
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

        settings = termios.tcgetattr(sys.stdin)
        key = None
    
        if key in DIRECTION or not key:
            key = getKey()
            if key == '\x03':
                rclpy.shutdown()
            
            if key == 'w' or key == 's': # Forward/backwards
                msg.linear.x = DIRECTION[key]
            elif key == 'a' or key == 'd':
                msg.angular.z = DIRECTION[key]
           

        self.vel_pub.publish(msg)
        


def main():
    rclpy.init()
    teleop_publisher = Teleop()
    rclpy.spin(teleop_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
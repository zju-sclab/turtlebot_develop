import roslib
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from math import radians


class alongWall:
    def __init__(self, dis, M, m):
        self.sub = rospy.Subscriber('laser', Float32MultiArray, self.call)
        self.cmdPub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move_cmd = Twist()
        self.dis = dis
        self.rate = 0
        self.M = M
        self.m = m

    def call(self, data):
        while True:
            d = data.data[0]
            self.rate = abs(d-self.dis)/self.dis
            if self.rate > self.M:  # update dis
                self.dis = d
            else:
                if d < self.dis-self.m:
                    # set angle
                    self.move_cmd.angular.z = radians(5)
                    self.move_cmd.linear.x = 5
                    self.cmdPub.publish(self.move_cmd)
                if d > self.dis+self.m:
                    # set angle
                    self.move_cmd.angular.z = radians(-5)
                    self.move_cmd.linear.x = 5
                    self.cmdPub.publish(self.move_cmd)


def main(args):
    rospy.init_node('AlongWallMove', anonymous=False)
    _ = alongWall(0.5, 0.1, 0.05)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "keyboard stop..."


if __name__ == '__main__':
    main(sys.argv)

import roslib
import rospy
import sys
from math import radians
from geometry_msgs.msg import Twist

def main(args):
    rospy.init_node('turnAngle', anonymous=False)
    cmdPub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    cmd=Twist()
    cmd.angular.z=radians(-45)
    r = rospy.Rate(5)
    for x in range(0,15):
        cmdPub.publish(cmd)
        r.sleep()
    
if __name__ == '__main__':
    main(sys.argv)

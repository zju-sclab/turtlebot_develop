import roslib
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self):
        # initiliaze
        rospy.init_node('robotMove', anonymous=False)
        self.correctData = Float32MultiArray()
        self.dx_camA = 0.0
        self.correctSub = rospy.Subscriber(
            'QRcode_Correct', Float32MultiArray, self.callbackCamA)
        self.cmdPub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move_cmd = Twist()

    def callbackCamA(self, data):
        # print(data.data)
        self.correctData = data
        print(self.correctData.data)
        self.dx_camA = self.correctData.data[0]-self.correctData.data[2]/2
        print(self.dx_camA,self.dx_camA*46/166)
        if self.dx_camA > 4 or self.dx_camA < -4:
            print("correcting...")
            self.correctLR(data)
        else:
            self.cmdPub.publish(Twist())

    def correctLR(self, data):
        #print(self.dx_camA)
        if self.dx_camA < 4 and self.dx_camA > -4:
            self.cmdPub.publish(Twist())
        else:
            if self.dx_camA < 0:
                self.move_cmd.linear.x = 0.03
                self.move_cmd.angular.z = 0
                self.cmdPub.publish(self.move_cmd)
            else:
                self.move_cmd.linear.x = -0.03
                self.move_cmd.angular.z = 0
                self.cmdPub.publish(self.move_cmd)

    def shutdown(self):
            # stop turtlebot
            rospy.loginfo("Stop")
            self.cmdPub.publish(Twist())
            rospy.sleep(1)
            
def main(args):
    robot = Robot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        robot.shutdown()


if __name__ == '__main__':
    main(sys.argv)

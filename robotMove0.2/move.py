import roslib
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from math import radians


class Robot:
    def __init__(self):
        # initiliaze
        rospy.init_node('robotMove', anonymous=False)
        self.correctData = Float32MultiArray()
        self.closeData = Float32MultiArray()
        self.dx_camA = 0  # self.correctData.data[0]-self.correctData.data[2]/2
        self.dx_camB = 0  # self.closeData.data[0]-self.closeData.data[2]/2
        self.correctSub = rospy.Subscriber(
            'QRcode_Correct', Float32MultiArray, self.callbackCamA)
        self.correctSub = rospy.Subscriber(
            'QRcode_setp', Float32MultiArray, self.callbackCamB)
        self.cmdPub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move_cmd = Twist()
        self.clockwise = -1
        self.anticlockwise = 1
        self.cameraFlag = True

    def callbackCamA(self, data):
        # print(data.data)
        if self.cameraFlag:
            self.correctData = data
            self.dx_camA = self.correctData.data[0]-self.correctData.data[2]/2
            print(self.dx_camA)
            if self.dx_camA > 4 or self.dx_camA < -4:
                print("correcting...")
                self.correctLR(data)
            else:
                print("turning 90 degree... to move close")
                self.turnAngle90(self.clockwise)
                self.cameraFlag = False
                
    def callbackCamB(self, data):
        if not self.cameraFlag:
            self.closeData = data
            self.dx_camB = self.closeData.data[0]-self.closeData.data[2]/2
            print(self.dx_camB)
            if self.dx_camB < 30 and self.dx_camB > -30:
                print("move closing...")
                self.moveClose()
            else:
                print("turn 90 degree... to correct")
                self.turnAngle90(self.anticlockwise)
                self.cameraFlag = True

    def correctLR(self, data):
        #print(self.dx_camA)
        if self.dx_camA < 4 and self.dx_camA > -4:
            self.cmdPub.publish(Twist())
        else:
            if self.dx_camA < 0:
                self.move_cmd.linear.x = 0.04
                self.move_cmd.angular.z = 0
                self.cmdPub.publish(self.move_cmd)
            else:
                self.move_cmd.linear.x = -0.04
                self.move_cmd.angular.z = 0
                self.cmdPub.publish(self.move_cmd)

    def turnAngle90(self, direction):
        self.move_cmd.linear.x = 0
        if direction > 0:
            self.move_cmd.angular.z = radians(90)  # 30degree/s
        else:
            self.move_cmd.angular.z = radians(-90)
        r = rospy.Rate(4)
        for x in range(0, 4):
            self.cmdPub.publish(self.move_cmd)
            r.sleep()

    def moveClose(self):
        self.move_cmd.linear.x = 0.08
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

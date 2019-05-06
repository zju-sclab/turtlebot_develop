import roslib
import rospy
import sys
import serial

from LaserWriter import LaserWriter
from multiprocessing import Process, Queue
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
        self.laser = (Queue(),Queue())
        self.correctSub = rospy.Subscriber(
            'QRcode_Correct', Float32MultiArray, self.callbackCamA)
        # self.laserSub = rospy.Subscriber(
        #     'MultiLaserDistance', Float32MultiArray, self.callbackLaser)
        self.cmdPub = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move_cmd = Twist()
        self.clockwise = -1
        self.anticlockwise = 1
        self.cameraFlag = True
        # self.freshLaser = 0

    def callbackCamA(self, data):
        # print(data.data)
        self.correctData = data
        self.dx_camA = self.correctData.data[0]-self.correctData.data[2]/2
        # print(self.dx_camA)
        if self.cameraFlag:
            if self.dx_camA > 4 or self.dx_camA < -4:
                print("correcting...")
                self.correctLR(data)
            else:
                print("turning 90 degree... prepare to move forward")
                self.turnAngle90(self.clockwise)
                self.angleCorrect()
                # self.cameraFlag = False
                # self.freshLaser = 0

    def angleCorrect(self):
        d0=self.laser[0].get(True)
        d1=self.laser[1].get(True)
        print "hello"
        while abs(d0-d1) > 0.01:# definition
            if abs(d0-d1) < 0.5:  # is valid data
                print 'angle correcting...', d0, d1
                self.turn(d0-d1)
            d0=self.laser[0].get(True)
            d1=self.laser[1].get(True)
        self.stop()
        print 'move to close'
        self.moveClose()

    def correctLR(self, data):
        # print(self.dx_camA)
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
        self.cmdPub.publish(Twist())

    def turn(self, direction):
        self.move_cmd.linear.x = 0
        if direction > 0:
            self.move_cmd.angular.z = radians(10)  # 10degree/s
        else:
            self.move_cmd.angular.z = radians(-10)
        self.cmdPub.publish(self.move_cmd)
        # r = rospy.Rate(4)
        # for x in range(0, 2):
        #     self.cmdPub.publish(self.move_cmd)
        #     r.sleep()
        # self.cmdPub.publish(Twist())

    def isStop(self):
        if self.move_cmd.linear.x is not 0 or self.move_cmd.angular.z is not 0:
            return False
        else:
            return True

    def stop(self):
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmdPub.publish(self.move_cmd)

    def moveClose(self):
        self.move_cmd.linear.x = 0.1
        self.move_cmd.angular.z = 0
        self.cmdPub.publish(self.move_cmd)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop")
        self.cmdPub.publish(Twist())
        rospy.sleep(1)


def main(args):
    robot = Robot()
    writer = LaserWriter()
    pw = Process(target=writer.pollingRead, args=(robot.laser,))
    pw.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        robot.shutdown()
        pw.terminate()


if __name__ == '__main__':
    main(sys.argv)

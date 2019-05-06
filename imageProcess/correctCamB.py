import roslib
import rospy
import cv2
import sys
import numpy
#from Float32MultiArray.msg import Float32MultiArray
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_process:
    def __init__(self):
        #self.pub
        self.bridge=CvBridge()
        self.image_sub=rospy.Subscriber("/usb_cam0/image_raw0",Image,self.callback)
        self.correctPub=rospy.Publisher("QRcode_setp",Float32MultiArray,queue_size=10)

    def callback(self,data):
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow("1", cv_image)
        shape=cv_image.shape
        src_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        src_gray=cv2.blur(src_gray,(3,3))
        
        ret,threshold_output=cv2.threshold(src_gray,112,255,cv2.THRESH_BINARY)
   
        #findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
        _, contours, hierarchy=cv2.findContours(threshold_output,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
        contours2=[]
        parentIdx=-1
        #print(hierarchy[0,0,2])
        for i in range(0,len(contours)):
            if hierarchy[0,i,2] !=-1:
                if hierarchy[0,hierarchy[0,i,2],2] != -1:
                    parentIdx=i
                    contours2.append(contours[parentIdx])
                    #drawContours(drawing, contours, parentIdx, CV_RGB(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 1, 8);
                    parentIdx = -1

        if len(contours2)!=3:
            print "error! number of checked position angle is not 3!-:"+str(len(contours2))
        else:
            vec=[]
            vec.extend(contours2[0])
            vec.extend(contours2[1])
            vec.extend(contours2[2])
            vec=numpy.array(vec)
            rect = cv2.minAreaRect(vec)
            print("width:",rect[1],"height:",rect[2],"\n")
            x0,y0=rect[0]
            dx=shape[1]/2-x0
            dy=shape[0]/2-y0
            print("QRcodeCentre:",x0,y0,"rawImageCentre:",shape[1],shape[0],"Correct:",dx,dy)
            array_correct=[x0,y0,shape[1],shape[0],dx,dy]
            array_pub=Float32MultiArray(data=array_correct)
            self.correctPub.publish(array_pub)

            # box=cv2.boxPoints(rect)
            # print(box)


def main(args):
    rospy.init_node('image_process2',anonymous=True)
    _ = image_process()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)

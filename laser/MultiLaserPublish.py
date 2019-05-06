#!/usr/bin/python
import serial
import MultiLaser
import rospy
import roslib
import sys
from std_msgs.msg import String, Float32MultiArray

class MultiLaserPub:
    def __init__(self):
        self.ser=[]
        self.cmd_alwaysMeature = [0x80, 0x06, 0x03, 0x77]
        self.port = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        self.pub=rospy.Publisher("MultiLaserDistance",Float32MultiArray,queue_size=20)
        for i in range(len(self.port)):
            self.ser.append(serial.Serial(self.port[i], baudrate=9600, timeout=0.001))
        for i in self.ser:
            i.write(self.cmd_alwaysMeature)
    
    def toHex(self,argv):
        """
        raw data to hex
        """
        result = []
        hLen = len(argv)
        for i in xrange(hLen):
            hvol = ord(argv[i])
            hhex = '%02x' % hvol
            result.append(hhex)
        return result

    def getDistance(self,distanceHEX):
        """
        compute distance from hex data which is 12 bytes long
        """
        distance_int = 100*(int(distanceHEX[3], 16)-48)+10 * \
            (int(distanceHEX[4], 16)-48)+(int(distanceHEX[5], 16)-48)
        distance_decimal = 0.1*float(int(distanceHEX[7], 16)-48)+0.01*float(
            int(distanceHEX[8], 16)-48)+0.001*float(int(distanceHEX[9], 16)-48)
        distance = float(distance_int)+distance_decimal
        return round(distance, 3)

    def pollingRead(self):
        '''
        polling read all the laser's data and get the distance
        '''
        data=[]
        dis=[]
        for i in range(len(self.ser)):
            data.append([])
            dis.append(0)
        
        while True:
            for i in range(len(self.ser)):
                d = self.ser[i].read(12)
                d = self.toHex(d)
                if len(data[i])+len(d) < 12:
                    data[i] += d
                    d = []
                else:
                    data[i] += d[0:(12-len(data[i]))]
                    d = d[(12-len(data[i])):]
                    if data[3]=='E' and data[4]=='R' and data[5]=='R':
                        print 'error'
                    else:
                        dis[i] = self.getDistance(data[i])
                    array=[i,dis[i]]
                    arrayToPub=Float32MultiArray(data=array)
                    self.pub.publish(arrayToPub)
                    print i, data[i], dis[i]
                    data[i] = d
    def run(self):
        self.pollingRead()

def main(args):
    rospy.init_node('MultiLaserPub',anonymous=True)
    mlp=MultiLaserPub()
    print 'hello'
    try:
        mlp.run()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)

#!/usr/bin/python
import serial
from multiprocessing import Process, Queue

cmd_alwaysMeature = [0x80, 0x06, 0x03, 0x77]
cmd_shutdown = [0x80, 0x04, 0x02, 0x7A]
cmd_fre = [[0xFA, 0x04, 0x0A, 0x05, 0x0E], [0xFA, 0x04,
                                            0x0A, 0x10, 0x19], [0xFA, 0x04, 0x0A, 0x20, 0x29]]  # frequency:05,10,20
cmd_interval = [0xFA, 0x04, 0x05, ]
cmd_resolution = [0xFA, 0x04, 0x0C, 0x02, 0xF4]
cmd_closeLaser = [0x80, 0x06, 0x05, 0x00, 0x75]  # close laser
cmd_openLaser = [0x80, 0x06, 0x05, 0x01, 0x74]  # open laser
cmd_range = [0xFA, 0x04, 0x09, 0x05, 0xF4]  # range is 5 metres

class LaserWriter:
    def __init__(self):
        self.ser=[]
        self.port = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        for i in range(len(self.port)):
            self.ser.append(serial.Serial(self.port[i], baudrate=9600, timeout=0.001))
        for i in self.ser:
            i.write(cmd_alwaysMeature)
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


    def pollingRead(self,queue):
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
                    if data[i][3]=='E' and data[i][4]=='R' and data[i][5]=='R':
                        print 'error'
                    else:
                        dis[i] = self.getDistance(data[i])
                        queue.put((i,dis[i]))
                    # print i, data[i], dis[i]
                    data[i] = d

    # def getDis():
    #     ser=[]
    #     for i in range(len(port)):
    #         ser.append(serial.Serial(port[i], baudrate=9600, timeout=0.001))
    #     for i in ser:
    #         i.write(cmd_alwaysMeature)
    #     try:
    #         pollingRead(ser)
    #     except KeyboardInterrupt:
    #         for i in ser:
    #             i.close()


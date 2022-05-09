#!/usr/bin/env python3
import depthai as dai
from scipy.spatial.transform import Rotation as R

'''
    This file utilizes the OAK-D's IMU to keep track of the current angle of the rover relative to its starting position. If the rover doesn't turn, 
    the output will print 0. If the rover turns to the right 10 degrees, the output will print 10. If the rover then turnes left 15 degrees, the
    output will print -5.
'''

def setup():
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)
    xlinkOut.setStreamName("imu")
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    imu.out.link(xlinkOut.input)
    return pipeline

def imu():
    start = 0
    flag = 0
    iterations = 100

    with dai.Device(setup()) as device:

        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        while True:
            imuData = imuQueue.get()
            imuPackets = imuData.packets
            
            for imuPacket in imuPackets:
                rVvalues = imuPacket.rotationVector        
                r = R.from_quat([rVvalues.real, rVvalues.i, rVvalues.j, rVvalues.k]) 
                rotation = r.as_euler('zyz', degrees=True)
                if flag < iterations:
                    start += rotation[2]
                    flag += 1
                    continue
                if flag == iterations:
                    start = start / iterations + 180
                    flag += 1
                raw = rotation[2] + 180
                if raw > start:
                    out = raw - start
                elif raw < start:
                    out = raw + (360 - start)
                else:
                    out = 0
                out = int(((out + 180) % 360) - 180)
                print(out)

if __name__ == '__main__':
    imu()

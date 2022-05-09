
# Copyright (c) 2020 Luxonis
# Permission granted to copy, modify, merge, publish, distribute, sublicense. See LICENSE for further information  

from pathlib import Path
import numpy as np
import cv2
import depthai as dai
from requests import head
from calc import HostSpatialsCalc
from utility import *
import time
import math
import time
import json

'''
            This file performs all OAK-D object detection work such as drawing bounding boxes over detected objects in the image,
            clacluating each objects width, distance, angular size/position, and writes lidar-replicated data to a json file to
            be read by replaceLidar.py if the actual lidar component ever goes offline.
'''

CONF_THRESHOLD = 0.15
# Size of screen
SHAPE = 320
# Horizontal FOV of OAK-D color camera
HFOV = 69
START_TIME = None
coco_90 = [ "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",         \
            "fire hydrant", "12", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",     \
            "elephant", "bear", "zebra", "giraffe", "26", "backpack", "umbrella", "29", "30", "handbag", "tie",             \
            "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",            \
            "skateboard", "surfboard", "tennis racket", "bottle", "45", "wine glass", "cup", "fork", "knife", "spoon",      \
            "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",     \
            "chair", "couch", "potted plant", "bed", "66", "dining table", "68", "69", "toilet", "71", "tv", "laptop",      \
            "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "83",      \
            "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
    
def depthai_cam_and_link_setup():
    # This function is just creating links within the camera using the p pipeline.
    # Code is mostly from DepthAI and is modifyable under their license.
    p = dai.Pipeline()
    p.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_3)
    camRgb = p.create(dai.node.ColorCamera)
    camRgb.setPreviewSize(SHAPE, SHAPE)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFp16(True)
    nn = p.create(dai.node.NeuralNetwork)
    nn.setBlobPath(str(Path("models/efficientdet_lite0_2021.3_6shaves.blob").resolve().absolute()))
    nn.setNumInferenceThreads(2)
    camRgb.preview.link(nn.input)
    nn_xout = p.create(dai.node.XLinkOut)
    nn_xout.setStreamName("nn")
    nn.out.link(nn_xout.input)
    rgb_xout = p.create(dai.node.XLinkOut)
    rgb_xout.setStreamName("rgb")
    nn.passthrough.link(rgb_xout.input)
    monoLeft = p.create(dai.node.MonoCamera)
    monoRight = p.create(dai.node.MonoCamera)
    stereo = p.create(dai.node.StereoDepth)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    stereo.initialConfig.setConfidenceThreshold(255)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    xoutDepth = p.create(dai.node.XLinkOut)
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)
    return p, stereo


class FPSHandler:
    # Calculates FPS
    def __init__(self, cap=None):
        self.timestamp = time.time()
        self.start = time.time()
        self.frame_cnt = 0
    def next_iter(self):
        self.timestamp = time.time()
        self.frame_cnt += 1
    def fps(self):
        return self.frame_cnt / (self.timestamp - self.start)

def fill_open(ranges):
    # Use blocked intervals to construct open intervals
    # {'blocked': [[17,29], [51,68]]} -> {'open': [[0, 16], [30, 50]], 'blocked': [[17,29], [51,68]]}
    if not ranges['blocked']:
        ranges['open'] = [0, HFOV - 1]
        return ranges

    # To prevent blocked and open starting/ending at the exact same degree, use offset
    offset = 0.01
    open_ = []
    arr = ranges['blocked']
    # If the very first blocked degree is great than 0, everything between 0 and that first blocked degree is open
    if arr[0][0] > 0:
        open_.append([0, arr[0][0] - offset])
    # Loop through each pair in blocked to determine the open pairs
    # Ex: blocked: [10, 15], [20, 25] -> open = [15 + 0.01, 20 - 0.01]
    for i in range(len(arr) - 1):
        open_.append([arr[i][1] + offset, arr[i+1][0] - offset])
    # If the last blocked degree is less than the max degree, fill the rest in the final open pair
    # Ex: blocked = [[10, 20], [50, 55]] -> open = [55.01, 68] 
    if arr[-1][1] < HFOV - 1:
        open_.append([arr[-1][1] + offset, HFOV - 1])
    ranges['open'] = open_

    return ranges

def blocked_merge(ranges):
    # Many blocked intervals will intersect. Merge these intervals into as few as possible
    # in -> blocked: [[0, 10], [9, 18]] out -> blocked: [[0, 18]]
    if not ranges:
        return ranges
    arr = []
    curr = None
    for i in ranges:
        if not curr:
            curr = i
            continue
        # If the next pair's smallest value is larger than current pair's largest value, it doesn't intersect
        if i[0] > curr[1]:
            arr.append(curr)
            curr = i
        # If there's an intersection, add the maximum of either pair's largest values
        else:
            curr[1] = max(i[1], curr[1])
    arr.append(curr)
    return arr
 
def angular_size_calculator(size):
    # Calculates the angular size of an object
    return round(HFOV * (size / SHAPE), 2)

def suppress_bbox_intersection(arr):
    # Remove all bounding boxes that are entirely consumed by larger bounding boxes
    # in -> [[0, 0, 50, 50], [10, 10, 30, 30]]
    # out -> ans = [[0, 0, 50, 50]]
    if not arr:
        return arr
    arr.sort(key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
    ans = []
    for i in arr:
        if not ans:
            ans.append(i)
            continue
        ixmin, iymin, ixmax, iymax = i
        flag = False
        for j in ans:
            jxmin, jymin, jxmax, jymax = j
            # If current bbox is entirely consumed by any bbox, set flag to true to leave it out of the output array
            if ixmin >= jxmin and ixmax <= jxmax and iymin >= jymin and iymax <= jymax:
                flag = True
        # If it was determined that the current bbox is not consumed by any other bbox, add it to the output array
        if not flag:
            ans.append(i)
    return ans

def calc_dpd(depthFrame, hostSpatials):
    # dpd = distance per degree. This function mimics lidar and outputs the distance of the nearest object from 0 through HFOV degrees.
    # Calculate the distance value of all degrees between 0 and 68 (0-68 = 69 degrees = HFOV) degrees horizontally
    dpd = []
    step = SHAPE / HFOV
    y = SHAPE // 2
    degree = 0
    i = 0
    while i <= SHAPE:
        degree += 1
        spatials, _ = hostSpatials.calc_spatials(depthFrame, (int(i), y))
        dpd.append((degree - 1, round(spatials['z'] / 1000, 2)))
        i += step
    return dpd[::-1]

def fill_lidar_dict(dpd):
    lidar_publisher_data = {
        'header': {'stamp': {'sec': None, 'nanosec': None}, 'frame_id': 'laser'}
    }
    elapsed_ms = (time.time() * 1000 - START_TIME)
    lidar_publisher_data['header']['stamp']['sec']      = str(elapsed_ms / 1000)
    lidar_publisher_data['header']['stamp']['nanosec']  = str(elapsed_ms % 1000)
    lidar_publisher_data['angle_min']                   = str(math.radians(0))
    lidar_publisher_data['angle_max']                   = str(math.radians(HFOV))
    lidar_publisher_data['angle_increment']             = str(math.radians(1))
    lidar_publisher_data['time_increment']              = 0.00012443582818377763
    lidar_publisher_data['scan_time']                   = 0.13426625728607178
    lidar_publisher_data['range_min']                   = 0.21
    lidar_publisher_data['range_max']                   = 35
    lidar_publisher_data['ranges'] = []
    for i in dpd:
        if math.isnan(i[1]):
            lidar_publisher_data['ranges'].append(float('inf'))
        else:
            lidar_publisher_data['ranges'].append(i[1])
    lidar_publisher_data['intensities'] = []
    for i in lidar_publisher_data['ranges']:
        if i == 'inf':
            lidar_publisher_data['intensities'].append(0.0)
        else:
            lidar_publisher_data['intensities'].append(47.0)
    return lidar_publisher_data

def write_to_file(lidar_publisher_data):
    file = open('lidarstuff.json', 'w')
    json.dump(lidar_publisher_data, file, indent="")
    file.close()
    return

def main():
    p, stereo = depthai_cam_and_link_setup()

    with dai.Device(p) as device:
        # queued output from camera. rgb is the image, nn is the detection model
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        qNn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth")
        hostSpatials = HostSpatialsCalc(device)
        delta = 5
        hostSpatials.setDeltaRoi(delta)
        fps = FPSHandler()
        shape = (3, SHAPE, SHAPE)
        while True:

            inRgb       = qRgb.get()
            in_nn       = qNn.tryGet()
            frame       = np.array(inRgb.getData()).view(np.float16).reshape(shape).transpose(1, 2, 0).astype(np.uint8).copy()
            depthFrame  = depthQueue.get().getFrame()
            file = open('lidarstuff.txt', 'w')
            # If the neural network has output data, examine it
            if in_nn is not None:
                
                # Calculate and draw fps over image
                fps.next_iter()
                cv2.putText(frame, "Fps: {:.2f}".format(fps.fps()), (2, SHAPE - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color=(255, 255, 255))

                bb = np.array(in_nn.getLayerFp16('Identity')).reshape(25, 4)
                conf = in_nn.getLayerFp16('Identity_2')
                
                # {'open': [[0, 16], [30, 50]], 'blocked': [[17,29], [51,69]]}
                ranges = {'open': [], 'blocked': []}
                bbox_coords = []

                # Iterate through all detections. If the neural network is confident enough of an object passed the threshold, process it.
                for i in range(len(conf)):
                    if CONF_THRESHOLD < conf[i]:

                        bb_det = bb[i]
                        # Limit the bounding box to 0..1
                        bb_det[bb_det > 1] = 1
                        bb_det[bb_det < 0] = 0
                        # xy_min = [x, y] of top left of bbox
                        xy_min = (int(bb_det[1]*SHAPE), int(bb_det[0]*SHAPE))
                        # xy_max = [x, y] of bottom right of bbox
                        xy_max = (int(bb_det[3]*SHAPE), int(bb_det[2]*SHAPE))
                        # [[top left x, top left y, bottom right x, bottom right y]]
                        bbox_coords.append([xy_min[0], xy_min[1], xy_max[0], xy_max[1]])

                        angular_size = angular_size_calculator((xy_max[0] - xy_min[0]))

                        # start and end degree of object in HFOV
                        angular_position = (angular_size_calculator(xy_min[0]), angular_size_calculator(xy_max[0]))

                        # x, y of center of bbox
                        x, y = xy_min[0] + ((xy_max[0] - xy_min[0]) // 2), xy_min[1] + ((xy_max[1] - xy_min[1]) // 2)

                        # get distance using x, y
                        spatials, _ = hostSpatials.calc_spatials(depthFrame, (x, y))

                        x_val, y_val, z_val = None, None, None
                        # If an x, y, and z value is found, distance can be calculated. Otherwise leave the values blank ('-')
                        if not math.isnan(spatials['x']) and not math.isnan(spatials['y']) and not math.isnan(spatials['z']):
                            x_val = spatials['x']/1000
                            y_val = spatials['y']/1000
                            z_val = spatials['z']/1000
                            d = round((x_val ** 2 + y_val ** 2 + z_val ** 2) ** 0.5, 2)
                            w = round(2 * d * math.tan(angular_size / 2 * math.pi / 180) / 2, 2)

                        else:
                            w, d = '-', '-'

                        left, right = angular_position
                        ranges['blocked'].append([left, right])
                        #cv2.putText(frame, f"w={w}, d={d}, a=({left}, {right})", (x // 2, y), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (255,255,255), thickness = 2, lineType = cv2.LINE_AA)
                
                # blocked: [[10, 20], [0, 8], [40,60]] -> blocked: [[0, 8], [10, 20], [40, 60]]
                ranges['blocked'].sort(key=lambda x: x[0])
                # blocked: [[0, 8], [5, 20], [40, 60]] -> blocked: [[0, 20], [40, 60]]
                ranges['blocked'] = blocked_merge(ranges['blocked'])
                # Use blocked intervals to create open intervals
                ranges = fill_open(ranges)
                #print(ranges)
                # distance per degree
                dpd = calc_dpd(depthFrame, hostSpatials)
                #print(dpd)
                bbox_coords = suppress_bbox_intersection(bbox_coords)
                # draw bounding boxes onto image
                for i in bbox_coords:
                    xy_min, xy_max = i[0:2], i[2:]
                    cv2.rectangle(frame, xy_min , xy_max, (255, 0, 0), 2)

                lidar_publisher_data = fill_lidar_dict(dpd)
                write_to_file(lidar_publisher_data)

            cv2.imshow("rgb", frame)
            if cv2.waitKey(1) == ord('q'):
                break



if __name__ == '__main__':
    START_TIME = time.time() * 1000
    main()

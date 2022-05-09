from rplidar import RPLidar
import rplidar
import random
import serial
import sys
from time import sleep
import codecs
#from __future__ import print_function
#passing arguments to python script
import odrive
from odrive.enums import *
import odrive.shell
from fibre.utils import *
import time
import math
import json
import os
import tempfile
import sys
import fibre.libfibre
from odrive.utils import OperationAbortedException, yes_no_prompt

'''
    This file uses lidar to detect the largest opening in front of the rover and then rotates the servos to the angle of the 
    largest opening's midpoint.
'''

velocity = 2

def process(angle_to_distance, ranges):
    distances = []
    for i in range(315, 360):
        distances.append(angle_to_distance[str(i)])
    for i in range(0, 46):
        distances.append(angle_to_distance[str(i)])
    print(distances)

    min_dis = 1219.2 # 1524 mm = 5 ft
    binaries = [0] * len(distances)

    num_scan = 20

    for j in range(len(distances)):
        if distances[j] < min_dis:
            binaries[j] += 1
    print(binaries)

    open_path = {}

    counter = 0
    one = False
    index = -1
    for i in range(len(binaries)):
        if binaries[i] == 1:
            one = True
        else:
            one = False
            if index == -1:
                index = i
        if one:
            if index > -1:
                open_path[str(index)] = counter
            index = -1
            counter = 0
        else:
            counter += 1
    if index > -1:
        open_path[str(index)] = counter
    print(open_path)
    if open_path:
        max_key, max_value = None, None
        for i in open_path:
            if not max_value or open_path[i] > max_value:
                max_key, max_value = i, open_path[i]
                continue
    	degrees_to_turn = ranges[int(int(max_key) + max_value / 2)]
    else:
        '''
            This code is executed if there is no open path.
        '''
    	degrees_to_turn = -1
    return degrees_to_turn

def servo_angle(angle):
    left, right, middle = 50, 150, 90
	print('before: ', angle)
	ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    if angle > 354 or angle < 5:
        angle = middle
    elif angle > 5 and angle < 90:
        angle = right
    else:
        angle = left
	print('after: ', angle)
	#25-140
	
	# Read in command line arguments
	string = str(angle) + "\n"

	# Convert argument to byte cocde
	string = codecs.encode(string,"utf-8")
	
	# Send argument to arduino via serial bus
	ser.write(string)
	
	

def change_vel(velocity = 2):
    print("finding an odrive...")
    odrv0 = odrive.find_any()

    print("Odrive connected")
    if sys.argv != 0:
        print("Current state: (8 = closed loop control", odrv0.axis0.current_state)
        print("Entering velocity control mode")
        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        print("Starting input velocity")
        odrv0.axis0.controller.input_vel = velocity 

    #Electrical power estimate. VdqÂ·Idq
    power = odrv0.axis0.controller.electrical_power
    print("Power: ", power)

    #Mechanical power estimate. Torque * velocity
    odrv0.axis0.controller.mechanical_power

def run_lidar():

    lidar = RPLidar('/dev/ttyUSB0')

    info = lidar.get_info()
    lidar.motor_speed = rplidar.MAX_MOTOR_PWM
    print(info)

    health = lidar.get_health()
    print(health)

    min_lidar_delta = 15.0

    ranges = [i for i in range(315, 360)]
    ranges = ranges + [i for i in range(0, 46)]
    angle_to_distance = {str(i): None for i in ranges}
    # Loop that infinitely captures lidar data
    for i, scan in enumerate(lidar.iter_scans(scan_type='express',min_len=100,max_buf_meas=False)):
        for val in scan:
            if str(int(val[1])) in angle_to_distance:
                angle_to_distance[str(int(val[1]))] = int(val[2])
        flag = False
        for j in angle_to_distance:
            if angle_to_distance[j] == None:
                flag = True
                break
        if flag:
            continue
        #################### Do stuff below here #########################
        angle = process(angle_to_distance, ranges)
        servo_angle(angle)


    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    
change_vel()
run_lidar()

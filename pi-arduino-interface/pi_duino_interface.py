#!/user/bin/env python
import serial
#from picamera import PiCamera
# import cv2
#from time import sleep
#import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

port = "/dev/ttyACM7"
# port = "/dev/cu.usbmodem1431"
# port = "COM7"
rate = 115200

s1 = serial.Serial(port, rate)
s1.flushInput()

#rightRotations = 0
#leftRotations = 0
#time = 0

#circ = .22225

"""
RESULTS:
pwm: 50, velocity: 0.046298996733551104
pwm: 150, velocity: 0.21783677963135792
pwm: 225, velocity: 0.3449275256649557
pwm: 275, velocity: 0.42942319470368645
pwm: 300, velocity: 0.46970332186187586
pwm: 400, velocity: 0.6190840032489171

on the ground:
pwm: 50, velocity: 0
pwm: 150, velocity: 0.15415510647870426
pwm: 225, velocity: 0.26383393035654784
pwm: 275, velocity: 0.3448355756414529
pwm: 300, velocity: 0.38302206556147955
pwm: 400, velocity: 0.5508111879373542
"""

"""
pwm: 50
Right: 0.04860098717094302
Left: 0.03702932355881373

pwm: 150
Right: 0.2106042777407531
Left: 0.21291861046317895

Right: 0.1550086400732845
Left: 0.1457543929047302

pwm: 250
Right: 0.37260756831056313
Left: 0.3864935646451183

Right: 0.310120584805065
Left: 0.3147492502499167

pwm: 350
Right: 0.5506277065289807
Left: 0.5367463357761493

Right: 0.4651808772075975
Left: 0.4651808772075975

"""

"""
On Ground
Right Motor Equation:
vr = 0.0016pwm - 0.0776

Left Motor Equation:
vl = 0.0016pwm - 0.0907
"""

"""
In Air
Right Motor Equation:
vr = 0.0017pwm - 0.0471

Left Motor Equation:
vl = 0.0016pwm - 0.0261
"""

# while True:
# 	if s1.inWaiting() > 0:
# 		print(s1.readline())
# 		inputValue = s1.readline().decode().strip('\r\n')
# 		wheel = inputValue[1]
# 		if inputValue[0:4] == "Time":
# 			time = int(inputValue[6:])
# 			break
# 		if inputValue[0] == "M":
# 			val = int(inputValue[4:])
# 			if wheel == "R":
# 				rightRotations = val
# 			elif wheel == "L":
# 				leftRotations = val
# 		print("Right: %d, Left: %d" % (rightRotations, leftRotations))

# Run this in the terminal (python3 video_processing.py)
# Don't use IDLE

right = True
camera = PiCamera()
camera.resolution = (480, 272)
camera.framerate = 5
img = np.empty((272, 480, 3), dtype = np.uint8)
width = 480
height = 272
img_center = int(width/2)
lane_width = 390

def processImage():
	#camera.rotation = 180
	#rawCapture = PiRGBArray(camera, size=(480, 272))
	camera.capture(img, 'bgr')
	#time.sleep(0.1)

	for frame in camera.capture_continuous(img, format="bgr", use_video_port = True):
		image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		
		cols_with_white = []
		white_pixels = []
		cols_with_yellow = []
		yellow_pixels = []

		for x in range(width-1, int(width/2), -1):
			for y in range(int(2*height/3)-15, int(2*height/3)+15):
				#if (90 <= image[y][x][0] <= 120) and (image[y][x][1] <= 25) and (image[y][x][2] >= 200):
				if (image[y][x][1] <= 25) and (image[y][x][2] >= 200):
					white_pixels.append([x, y])
					if x not in cols_with_white:
						cols_with_white.append(x)
		for x in range(int(width/2), 0, -1):
			for y in range(int(2*height/3)-15, int(2*height/3)+15):
				if (20 <= image[y][x][0] <= 40) and (40 <= image[y][x][1] <= 255) and (200 <= image[y][x][2] <= 255):
					yellow_pixels.append([x, y])
					if x not in cols_with_yellow:
						cols_with_yellow.append(x)

		y_total = 0
		w_total = 0

		for i in range(0, len(yellow_pixels), 1):
			y_total += yellow_pixels[i][0] # add all x-values
		for i in range(0, len(white_pixels), 1):
			w_total += white_pixels[i][0]

		try:
			y_avg = int(y_total/(len(yellow_pixels)))
		except ZeroDivisionError:
			y_avg = 0
		try:
			w_avg = int(w_total/(len(white_pixels)))
		except ZeroDivisionError:
			w_avg = 0

		if (len(yellow_pixels) != 0 and len(white_pixels) != 0):

			lane_center = int((y_avg + w_avg)/2)

		elif len(yellow_pixels) != 0:
			# there are yellow pixels, but no white pixels
			# get the lane center by dividing lane_width by 2 and adding it to y_avg
			lane_center = int(lane_width/2) + y_avg
			
		elif len(white_pixels) != 0:
			# there are white pixels, but no yellow pixels
			# get the lane center by dividing lane_width by 2 and subtracting it from w_avg
			lane_center = w_avg - int(lane_width/2)

		else:
			# no white or yellow pixels
			# set lane_center to img_center as a default
			lane_center = img_center - 10 # offset so both lines will show

		turn_distance = img_center - lane_center
		print("--------------------------")
		print("Y_total ", y_total, " len(y) ", len(yellow_pixels), " y_VG ", y_avg)
		print("W_total ", w_total, " len(w) ", len(white_pixels), " w_VG ", w_avg)
		print("lane center ", turn_distance)
		print("--------------------------")

		ctr_distance = 10 # acceptable amount of distance between the two lines
		return turn_distance

			# Wrote last time
			# time.sleep(0.1)
			# threshold = 10 # how much distance between the two lines is acceptable
			# if (abs(difference) > threshold):
			# 	if (difference > 0):
			# 		return 1
			# 	return 2
			# return 0

	### remove this
	# cap.release()
	# cv2.destroyAllWindows()

def shouldTurn():
    #turn = processImage()
    #print(turn)
    #while(turn != 0):
    while(True):
        turn = str(processImage())
        #turn += ";"
        s1.write(turn.encode("utf-8"))
        s1.flush()
        print(turn)
        


def initialization():
	if (right):
		s1.write(1)
		s1.write(0)
	else:
		s1.write(0)
		s1.write(1)

# initialization()

shouldTurn()

# velocity_right = (rightRotations)*(circ/32)/(time/1000)
# velocity_left = (leftRotations)*(circ/32)/(time/1000)
# print(time)
# print(velocity_right)
# print(velocity_left)
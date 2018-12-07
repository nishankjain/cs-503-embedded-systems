#!/user/bin/env python
import serial
from picamera import PiCamera
# import cv2
from time import sleep
# port = "/dev/ttyACM0"
# port = "/dev/cu.usbmodem1431"
port = "COM7"
rate = 115200

s1 = serial.Serial(port, rate)
s1.flushInput()

rightRotations = 0
leftRotations = 0
time = 0

circ = .22225

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

def processImage():
	cap = cv2.VideoCapture() 

	while threshold > 10:
		ret, frame = cap.read()

		height, width, channels = frame.shape 
		img_center = int(width/2)
		
		cols_with_white = []
		white_pixels = []
		cols_with_yellow = []
		yellow_pixels = []

		for x in range(width-1, 0, -1):
			for y in range(int(height/2)-15, int(height/2)+15):
				if (frame[y][x][0] >= 220) and (frame[y][x][1] >= 220) and (frame[y][x][2] >= 220):
					white_pixels.append([x, y])
					if x not in cols_with_white:
						cols_with_white.append(x)
				if (100 <= frame[y][x][0] <= 190) and (220 <= frame[y][x][1] <= 255) and (220 <= frame[y][x][2]):
					yellow_pixels.append([x, y])
					if x not in cols_with_yellow:
						cols_with_yellow.append(x)

		white_x = 0
		yellow_x = 0
		white_y = 0
		yellow_y = 0

		for i in range(0, len(white_pixels)):
			if white_pixels[i][0] == cols_with_white[-1]:
				white_x = white_pixels[i][0]
				white_y = white_pixels[i][1]
		for i in range(0, len(yellow_pixels)):
			if yellow_pixels[i][0] == cols_with_yellow[0]:
				yellow_x = yellow_pixels[i][0]
				yellow_y = yellow_pixels[i][1]

		if white_y == yellow_y:
			try:
				lane_center = round((cols_with_white[-1]+cols_with_yellow[0])/2)
			except IndexError:
				lane_center = img_center # default

		else:
			# find a different centerpt using the y-coords of the rightmost point
			# in the yellow line - find the leftmost point on the white line with this
			# y-coordinate

			for i in range(len(white_pixels)-1, 0, -1):
				if white_pixels[i][1] == yellow_y: 
					lane_center = round((white_x+cols_with_yellow[0])/2)
					break

		difference = lane_center - img_center

		### remove this
		# cv2.line(frame, (0, int(height/2)-15), (width-1, int(height/2)-15), (0, 0, 0), 1)
		# cv2.line(frame, (0, int(height/2)+15), (width-1, int(height/2)+15), (0, 0, 0), 1)
		# cv2.line(frame, (lane_center, 0), (lane_center, height-1), (255, 0, 0), 3)
		# cv2.line(frame, (img_center, 0), (img_center, height-1), (0, 0, 255), 3)
		
		# cv2.imshow('frame', frame)

		# if cv2.waitKey(1) & 0xFF == ord('q'):
		# 	break
		### end of block to remove

		time.sleep(0.1)
		threshold = 10 # how much distance between the two lines is acceptable
		if (abs(difference) > threshold):
			if (difference > 0):
				return 1
			return 2
		return 0

	### remove this
	cap.release()
	cv2.destroyAllWindows()

def shouldTurn():
	turn = processImage()
	while(turn != 0):
		s1.write(turn)
		turn = processImage()


def initialization():
	if (right):
		s1.write(1)
		s1.write(0)
	else:
		s1.write(0)
		s1.write(1)

initialization()

shouldTurn()

# velocity_right = (rightRotations)*(circ/32)/(time/1000)
# velocity_left = (leftRotations)*(circ/32)/(time/1000)
# print(time)
# print(velocity_right)
# print(velocity_left)
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

camera = PiCamera()
camera.resolution = (480, 272)
camera.framerate = 1
#camera.rotation = 180
#rawCapture = PiRGBArray(camera, size=(480, 272))
img = np.empty((272, 480, 3), dtype = np.uint8)
camera.capture(img, 'bgr')

width = 480
height = 272

img_center = int(width/2)

lane_width = 390

time.sleep(0.1)

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

    for i in range(0, len(yellow_pixels), 10):
        y_total += yellow_pixels[i][0] # add all y-values
    for i in range(0, len(white_pixels), 10):
        w_total += white_pixels[i][0]

    try:
        y_avg = int(y_total/(len(yellow_pixels)/10))
    except ZeroDivisionError:
        y_avg = 0
    try:
        w_avg = int(w_total/(len(white_pixels)/10))
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
        

    ctr_distance = 10 # acceptable amount of distance between the two lines

    if turn_distance > ctr_distance:
        # turn right
        return 2
    elif turn_distance < -1 * ctr_distance:
        # turn left
        return 1
    else:
        # don't turn
        return 0


    """cv2.line(frame, (0, int(2*height/3)-15), (width-1, int(2*height/3)-15), (0, 0, 0), 1)
    cv2.line(frame, (0, int(2*height/3)+15), (width-1, int(2*height/3)+15), (0, 0, 0), 1)
    cv2.line(frame, (img_center, 0), (img_center, height-1), (0, 0, 255), 2)
    cv2.line(frame, (lane_center, 0), (lane_center, height-1), (255, 0, 0), 2)
    cv2.line(frame, (y_avg, 0), (y_avg, height-1), (0, 255, 255), 1)
    cv2.line(frame, (w_avg, 0), (w_avg, height-1), (255, 255, 0), 1)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    #rawCapture.truncate(0)

    if key == ord('q'):
        break"""



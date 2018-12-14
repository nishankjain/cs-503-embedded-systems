from picamera import PiCamera
import picamera
import picamera.array
import time
from time import sleep
import io
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import serial
from collections import deque

port = "/dev/ttyACM1"
rate = 115200

s1 = serial.Serial(port, rate)
s1.flushInput()

q = deque()
## Test camera for 60 seconds
##camera.start_preview()
##sleep(60)
##camera.stop_preview()

## Take 5 pictures in a row
##camera = PiCamera()
##camera.resolution = (320, 240)
####camera.color_effects = (128, 128)
##img = np.empty((240, 320, 3), dtype=np.uint8)
##sleep(5)
##camera.start_preview()
##start_time = time.time()
##for i in range(1):
##    sleep(0.25)
####    camera.capture('/home/pippin_student/Desktop/cs-503/camera_test/image%s.jpg' %i)
##    camera.capture(img, 'rgb', use_video_port=True)
##    plt.imshow(img)
##    plt.show()
####    print(img[:, :, 1])
####    filename = './camera_test/test_gray_' + str(i)
####    np.savetxt(filename, img[:, :, 1])
####    , 'yuv', use_video_port=True
##end_time = time.time()
##duration = end_time - start_time
##print(duration)
##fps = 5 / duration
##print(fps)
##camera.stop_preview()

## Capture series of 10 images with 1 second delay
##with picamera.PiCamera() as camera:
##    camera.resolution = (1280, 720)
##    camera.start_preview()
##    time.sleep(1)
##    for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
##        print('Captured image %s' % filename)
##        if i == 10:
##            break
##        time.sleep(1)
##    camera.stop_preview()

## Capture low res JPEGs extremely rapidly using the video port capability of capture_sequence() method. Gives 29.4 fps!
##with picamera.PiCamera() as camera:
##    camera.start_preview()
##    camera.resolution = (640, 480)
##    sleep(5)
##    start = time.time()
##    camera.capture_sequence(('image%03d.jpg' %i for i in range(120)), use_video_port=True)
##    print('Captured 120 images at %.2ffps' %(120 / (time.time() - start)))
##    camera.stop_preview()

def findLine(img):
    line = img[120, :]
##    print(line)
    return line
    
def findLastWhitePixel(line):
    seen_white = False
    for index in range(160, 320): #160
        current_pixel = line[index]
        if current_pixel > 250:
            break
##            if seen_white:
##                break
##        else:
##            seen_white = True
    return (index, current_pixel)

def rgb2gray(img):
    return np.dot(img, [0.299, 0.587, 0.114])

## Captures 120 frames at 29.74 fps!
counter = 0

with picamera.PiCamera() as camera:
    img = np.empty((240, 320, 3), dtype=np.uint8)
##    camera.start_preview()
    camera.resolution = (320, 240)
##    camera.color_effects = (128, 128)
    sleep(5)
    start = time.time()
    while counter < 10000:
        camera.capture(img, 'rgb', use_video_port=True)
        
##        camera.capture('/home/pippin_student/Desktop/cs-503/camera_test/image%s.jpg' %counter)
##        print(img[:, :, 1])
##        np.savetxt('test_gray.txt', img[:, :, 1])
        counter += 1
        grayScaleImg = rgb2gray(img)
##        print(grayScaleImg.shape)
        line = findLine(grayScaleImg)
        whitePixelIndex, pixelValue = findLastWhitePixel(line)
        #print("Index: ", blackPixelIndex, "Value: ", pixelValue)
        diff = 280 - whitePixelIndex #280
        diff = str(diff)
        diff = diff + ";"
        q.append(diff)
        print("Index diff: ", diff, "Value: ", pixelValue)
        #if time.time() - start > 3 :
        val = q.popleft()
        s1.write(val.encode("utf-8"))
        s1.flush()
        #print(q)
##        sleep(0.5)
##       camera.capture('/home/pippin_student/Desktop/cs-503/camera_test/image%s.jpg' %counter)
##    print('Captured 120 images at %.2ffps' %(120 / (time.time() - start)))
##    camera.stop_preview()



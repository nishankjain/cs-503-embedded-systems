# Run this in the terminal (python3 video_processing.py)
# Don't use IDLE

import cv2
from time import sleep

cap = cv2.VideoCapture() 

while True:
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

    turn_distance = img_center - lane_center

    ctr_distance = 10 # how much distance between the two lines is acceptable
    """if turn_distance > ctr_distance:
        # turn right
    elif turn_distance < -1 * ctr_distance:
        # turn left
    else:
        # don't turn"""

    ### remove this
    cv2.line(frame, (0, int(height/2)-15), (width-1, int(height/2)-15), (0, 0, 0), 1)
    cv2.line(frame, (0, int(height/2)+15), (width-1, int(height/2)+15), (0, 0, 0), 1)
    cv2.line(frame, (lane_center, 0), (lane_center, height-1), (255, 0, 0), 3)
    cv2.line(frame, (img_center, 0), (img_center, height-1), (0, 0, 255), 3)
    
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    ### end of block to remove

    time.sleep(0.01)

### remove this
cap.release()
cv2.destroyAllWindows()

#!/user/bin/env python
import serial
# port = "/dev/ttyACM0"
port = "/dev/cu.usbmodem1431"
rate = 115200

s1 = serial.Serial(port, rate)
s1.flushInput()

rightRotations = 0
leftRotations = 0
time = 0

circumfrence = .22225

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
while True:
    if s1.inWaiting() > 0:
        inputValue = s1.readline().decode().strip('\r\n')
        wheel = inputValue[1]
        if inputValue[0:4] == "Time":
            time = int(inputValue[6:])
            break
        if inputValue[0] == "M":
            val = int(inputValue[4:])
            if wheel == "R":
                rightRotations = val
            elif wheel == "L":
                leftRotations = val
        print("Right: %d, Left: %d" % (rightRotations, leftRotations))


velocity = ((rightRotations + leftRotations)/2)*(circumfrence/32)/(time/1000)
print(time)
print(velocity)
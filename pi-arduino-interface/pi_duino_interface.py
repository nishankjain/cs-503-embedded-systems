#!/user/bin/env python
import serial
# port = "/dev/ttyACM0"
# port = "/dev/cu.usbmodem1431"
port = "COM7"
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
Right Motor Equation:
vr = 0.0016pwm - 0.0776

Left Motor Equation:
vl = 0.0016pwm - 0.0907
"""

while True:
   if s1.inWaiting() > 0:
       print(s1.readline())
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


velocity_right = (rightRotations)*(circumfrence/32)/(time/1000)
velocity_left = (leftRotations)*(circumfrence/32)/(time/1000)
print(time)
print(velocity_right)
print(velocity_left)
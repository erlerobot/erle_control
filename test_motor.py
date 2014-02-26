#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: Interface with the DC motors.
@brief Test the motors behaviour

The call to this method is:
     python text_motor.py [speed = 10] [time = 5]
'''

from motors import Motor
import Adafruit_BBIO.GPIO as GPIO
import time
import os
import sys

speed = 10
if len(sys.argv) > 1:
	speed = int(sys.argv[1])


m1 = Motor(1);
m1.setSpeedBrushless(speed)
m1.go()

m2 = Motor(2);
m2.setSpeedBrushless(speed);
m2.go()

m3 = Motor(3);
m3.setSpeedBrushless(speed);
m3.go()

m4 = Motor(4);
m4.setSpeedBrushless(speed);
m4.go()

sleep_time = 5
if len(sys.argv) > 2:
	sleep_time = int(sys.argv[2])

time.sleep(sleep_time)


#####
# shutdown the motors
#####

speed = 0

m1.setSpeedBrushless(speed);
m1.go()

m2.setSpeedBrushless(speed);
m2.go()

m3.setSpeedBrushless(speed);
m3.go()

m4.setSpeedBrushless(speed);
m4.go()


#####
# Shut down the robot
#####
#os.system("shutdown -h now")


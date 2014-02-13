#!/usr/bin/env python
"""
	@brief It seems the brushless motors need the duty
		cycle to change to get ready for operation.
	
"""
from motors import Motor
import Adafruit_BBIO.GPIO as GPIO
import time
import os

def initMotors():

	speed = 100

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

	#time.sleep(0.5)
	time.sleep(5)

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

initMotors()

# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: Interface with the DC motors.
'''
import Adafruit_BBIO.PWM as PWM    
"""
This class is assuming that last version of Adafruit_BBIO 
(https://github.com/adafruit/adafruit-beaglebone-io-python) been installed. 
This version assumes that duties 0(off)-100(on).
To upgrade run:
   sudo pip install --upgrade Adafruit_BBIO 
"""

class Motor:
    """ Class for interfacing with the motors.

        The class controls the motors speed using the sysfs PWM provided
        by Adafruit_BBIO.
        The duty should be provided in the range (0,100) however the speed
        will be accepted in the range (-100,100) so that both directions 
        clockwise (CW) and counter-clockwise (CCW) can be implemented. 
    """ 
    def __init__(self, motor_number=1, max_speed=100, min_speed=-100):
        self.speed = 0;
        # motors: U18, U20, U19, U21
        self.motor_pins_list = [["P9_14", "P9_16"], 
                            ["P8_19", "P8_13"],
                            ["P9_22", "P9_21"],
                            ["P9_42", "P9_28"]]
        if (motor_number > 4) or (motor_number < 1):
            raise Exception("Motor number provided out of bounds! ([1-4])")
        self.motor_number = motor_number # 1, 2, 3 or 4
        self.max_speed= max_speed
        self.min_speed= min_speed
        self.frequency = 2000
        self.motor_pins = self.motor_pins_list[self.motor_number - 1]
        #perform PWM initialization
        self.duty_IN1 = 0 # duty input 1 of the motor controller IC 
        self.duty_IN2 = 0 
        PWM.start(self.motor_pins[0], self.duty_IN1, self.frequency)
        PWM.start(self.motor_pins[1], self.duty_IN2, self.frequency)

    """ Set the duties according to the speed attribute 
    """
    def setSpeed(self, speed):
        if speed<=self.max_speed or speed>=self.min_speed:
            if speed > 0:
                self.duty_IN1 = abs(speed)
                self.duty_IN2 = 0
            else:
                self.duty_IN1 = 0
                self.duty_IN2 = abs(speed)            
        else:
            raise Exception("Speed provided not in the [-100,100] range!")

    """ update the motor PWM according to the class duty attributes
    """
    def go(self):
        PWM.set_duty_cycle(self.motor_pins[0],self.duty_IN1)
        PWM.set_duty_cycle(self.motor_pins[1],self.duty_IN2)


'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches
@description: Interface with the DC motors.
'''
class Motor:
    """ Class for interfacing with the motors.

        The class controls the motors speed using the sysfs PWM 
        interface of the OS.
    """ 
    def __init__(self, pwm_pin, max_speed=100, min_speed=0):
        self.speed = 0;
        self.pwm_pin = pwm_pin
        self.max_speed= max_speed
        self.min_speed= min_speed
        #TODO perform PWM initialization

    """ Set the speed attribute to the desired speed
    """
    def setSpeed(self, speed):
        if speed<=self.max_speed or speed>=self.min_speed:
            self.speed = speed
        else:
            raise Exception("Speed provided not in the 0-100 range!")

    """ update the motor PWM according to the speed attribute
    """
    def go(self):
        #TODO implement
        pass


# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: Code for stabilizing the quadrotor. It runs and infinite
loop that adjust the 4 motors according to the IMU readings.
'''

from imu import IMU
from motors import Motor
from pid import PID
from time import sleep

""" Limits the thrust passed to the motors
    in the range (-100,100)
"""
def limitThrust(thrust, upperLimit = 100, lowerLimit = 0):
    if thrust > upperLimit:
        thrush = upperLimit
    elif thrust < lowerLimit:
        thrust = lowerLimit
    return thrust

#instantiate IMU
#TODO see how to import C interface to python
imu=IMU() 
#MyKalman=KalmanFilter(....)

#instantiate motors and put them together in a list
motor1=Motor(1)
motor2=Motor(2)
motor3=Motor(3)
motor4=Motor(4)
motors=[motor1,motor2,motor3,motor4]

#TODO instantiate PID controllers
rollPID=PID()
pitchPID=PID()
yawPID=PID()
#zPID=PID(.....)
#xposPID=PID(.....)
#yposPID=PID(.....)

#init pitch, roll and yaw:
roll = 0
pitch = 0
yaw = 0

############################
#loop
############################
while 1:
    print "------------------------"
    print "     stabilize loop     "
    print "------------------------"

    # pitch, roll and yaw DESIRED:
    #  FOR NOW THEY ARE KEPT TO 0 BUT THIS INFORMATION SHOULD
    #  COME FROM THE TELEOPERATION DEVICE/MISSION SYSTEM
    roll = 0
    pitch = 0
    yaw = 0
    
    #Measure angles    
    #roll_m, pitch_m, yaw_m = imu.read_fusedEuler()
    roll_m, pitch_m, yaw_m = imu.read_fusedEuler(1)
    #MyKalman.measure([roll,pitch, yaw])
    
    #Run the PIDs
    roll = rollPID.update(roll_m - roll, 1) # measured minus desired (error)
    pitch = pitchPID.update(pitch_m - pitch, 1)
    yaw = yawPID.update(yaw_m - yaw, 1)
    #z = zPID.update(z_m - z)
    #xpos = xposPID.update(xpos_m - xpos)
    #ypos = yposPID.update(ypos_m - ypos)

    #TODO change this parameter and see the behaviour
    #thrust is provided by the controller (NOTE: this is also treated as "z" and it should use the zPID controller)
    thrust = 0

    #Log the values:
    print "------------------------"
    print "Measured angles:"
    print "     pitch:" + str(pitch_m)
    print "     roll:" + str(roll_m)
    print "     yaw:" + str(yaw_m)
    print "PID output angles:"
    print "     pitch:" + str(pitch)
    print "     roll:" + str(roll)
    print "     yaw:" + str(yaw)
    print "thrust:" + str(thrust)
    


    """ When using the dynamical model with u:

    #Update the control inputs "u"
    u = updateU(roll, pitch, yaw)
    #u = updateU(roll, pitch, yaw, z, xpos, ypos)
    
    #Set motor speeds
    motor1.setSpeed(u)
    motor2.setSpeed(u)
    motor3.setSpeed(u)
    motor4.setSpeed(u)

    """


    """
    #QUAD_FORMATION_X
    roll = roll >> 1;
    pitch = pitch >> 1;
    motorPowerM1 = limitThrust(thrust - roll + pitch + yaw);
    motorPowerM2 = limitThrust(thrust - roll - pitch - yaw);
    motorPowerM3 =  limitThrust(thrust + roll - pitch + yaw);
    motorPowerM4 =  limitThrust(thrust + roll + pitch - yaw);
    """

    #QUAD_FORMATION_NORMAL first approach    
    #TODO use the dynamical model equation to get the motor voltage
    motorPowerM1 = limitThrust(thrust + pitch + yaw, 100);
    motorPowerM2 = limitThrust(thrust - roll - yaw, 100);
    motorPowerM3 =  limitThrust(thrust - pitch + yaw, 100);
    motorPowerM4 =  limitThrust(thrust + roll - yaw, 100);

    #Log the motor powers:
    print "------------------------"
    print "motorPowerM1:" + str(motorPowerM1)
    print "motorPowerM2:" + str(motorPowerM2)
    print "motorPowerM3:" + str(motorPowerM3)
    print "motorPowerM4:" + str(motorPowerM4)
    print "------------------------"
    
    #Set motor speeds
    motor1.setSpeed(motorPowerM1)
    motor2.setSpeed(motorPowerM2)
    motor3.setSpeed(motorPowerM3)
    motor4.setSpeed(motorPowerM4)
    
    #Start Motors
    for mot in motors:
        mot.go()
    
    #Kalman Prediction
    #MyKalman.predict()

    #delay = 4e-3 #delay ms (250 Hz) 
    # delay = 20e-3 #delay ms (50 Hz)
    # time.sleep(delay)

############################
############################


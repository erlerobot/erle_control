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
import datetime as dt
from dynamical_model import Dynamical_Model
import signal
import sys

"""
When testing is important to power off the motors when the
script is stoped through SIGINT
"""
def signal_handler(signal, frame):
        print 'Setting all motors to 0...'
        
        motor1.setSpeedBrushless(0)
        motor2.setSpeedBrushless(0)
        motor3.setSpeedBrushless(0)
        motor4.setSpeedBrushless(0)

        for mot in motors:
            mot.go()

        sys.exit(0)

""" Limits the thrust passed to the motors
    in the range (-100,100)
"""
def limitThrust(thrust, upperLimit = 100, lowerLimit = 0):
    if thrust > upperLimit:
        thrust = upperLimit
    elif thrust < lowerLimit:
        thrust = lowerLimit
    return thrust


# Set the handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)


############################
# variables
############################

logging = 1
logging_time = 1
limit_thrust = 40

roll_d = 0
pitch_d = 0
yaw_d = 0
z_d = 10
#xpos = 0
#ypos = 0

############################

#instantiate IMU
#TODO see how to import C interface to python
imu=IMU() 
#MyKalman=KalmanFilter(....)

# dynamical model instance
dyn_model = Dynamical_Model()

#instantiate motors and put them together in a list
motor1=Motor(1)
motor2=Motor(2)
motor3=Motor(3)
motor4=Motor(4)
motors=[motor1,motor2,motor3,motor4]

# instantiate PID controllers
# rollPID=PID(0.9, 0.2, 0.3) # Kp, Kd, Ki
# pitchPID=PID(0.9, 0.2, 0.3)
# yawPID=PID(0.06, 0.02, 0.01)
# zPID=PID(0.9, 0.2, 0.1)

rollPID=PID(1, 0, 0) # Kp, Kd, Ki
pitchPID=PID(1, 0, 0)
yawPID=PID(0, 0, 0)
zPID=PID(1, 0, 0)


#xposPID=PID(-0.09, -0.1, 0)
#yposPID=PID(-0.09, -0.1, 0)

# test variables
frequencies = []

if logging:
    print "------------------------"
    print "     stabilize loop     "
    print "------------------------"
############################
#loop
############################
while 1:
    start = dt.datetime.now()

    # pitch, roll and yaw DESIRED, get FROM THE TELEOPERATOR (for now, global vars are used)
    # roll_d = 0
    # pitch_d = 0
    # yaw_d = 0
    # z_d = 10
    # #xpos = 0
    # #ypos = 0
    
    #Measure angles  
    #roll_m, pitch_m, yaw_m = imu.read_fusedEuler()
    roll_m, pitch_m, yaw_m = imu.read_fusedEuler(0)
    #MyKalman.measure([roll,pitch, yaw])

    z_m = 0 # putting this is the same as telling it to always increase the thrust (go up)
    
    #Run the PIDs
    roll = rollPID.update(roll_d - roll_m, 0)
    pitch = pitchPID.update(pitch_d - pitch_m, 0)
    yaw = yawPID.update(yaw_d - yaw_m, 0)
    z = zPID.update(z_d - z_m, 0)
    #xpos = xposPID.update(xpos_d - xpos_m)
    #ypos = yposPID.update(ypos_d - ypos_m)

    #TODO change this parameter and see the behaviour
    #thrust is provided by the controller (NOTE: this is also treated as "z" and it should use the zPID controller)
    # the point of hovering is 35% duty cycle in the motors


    if logging:
        #Log the values:
        print "**************************"
        print "Desired angles:"
        print "     pitch:" + str(pitch_d)
        print "     roll:" + str(roll_d)
        print "     yaw:" + str(yaw_d)
        print "     z:" + str(z_d)    
        print "Measured angles:"
        print "     pitch:" + str(pitch_m)
        print "     roll:" + str(roll_m)
        print "     yaw:" + str(yaw_m)
        # print "     thrust (z):" + str(z_d)    # maybe using some sensor?
        print "PID output angles:"
        print "     pitch:" + str(pitch)
        print "     roll:" + str(roll)
        print "     yaw:" + str(yaw)       
        print "     z:" + str(z)       
    

    # use the dynamical_model, returns u=[u_m1, u_m2, u_m3, u_m3]
    u = dyn_model.motor_inversion(z, roll, pitch, yaw)

    motorPowerM1 = limitThrust(u[0], limit_thrust);
    motorPowerM2 = limitThrust(u[1], limit_thrust);
    motorPowerM3 = limitThrust(u[2], limit_thrust);
    motorPowerM4 = limitThrust(u[3], limit_thrust);

    if logging:
        #Log the motor powers:
        print "------------------------"
        print "motorPowerM1 (method 1):" + str(motorPowerM1)
        print "motorPowerM2 (method 1):" + str(motorPowerM2)
        print "motorPowerM3 (method 1):" + str(motorPowerM3)
        print "motorPowerM4 (method 1):" + str(motorPowerM4)
        print "**************************"

    # #Set motor speeds
    # motor1.setSpeedBrushless(motorPowerM1)
    # motor2.setSpeedBrushless(motorPowerM2)
    # motor3.setSpeedBrushless(motorPowerM3)
    # motor4.setSpeedBrushless(motorPowerM4)
    
    # #Start Motors
    # for mot in motors:
    #     mot.go()
        
    
    #QUAD_FORMATION_NORMAL first approach        
    motorPowerM1 = limitThrust(z + pitch + yaw, limit_thrust);
    motorPowerM2 = limitThrust(z - roll - yaw, limit_thrust);
    motorPowerM3 =  limitThrust(z - pitch + yaw, limit_thrust);
    motorPowerM4 =  limitThrust(z + roll - yaw, limit_thrust);

    if logging:
        #Log the motor powers:
        print "------------------------"
        print "motorPowerM1 (method 2):" + str(motorPowerM1)
        print "motorPowerM2 (method 2):" + str(motorPowerM2)
        print "motorPowerM3 (method 2):" + str(motorPowerM3)
        print "motorPowerM4 (method 2):" + str(motorPowerM4)
        print "**************************"

    
    #Set motor speeds
    motor1.setSpeedBrushless(motorPowerM1)
    motor2.setSpeedBrushless(motorPowerM2)
    motor3.setSpeedBrushless(motorPowerM3)
    motor4.setSpeedBrushless(motorPowerM4)
    
    #Start Motors
    for mot in motors:
        mot.go()
    

    #Kalman Prediction
    #MyKalman.predict()

    #delay = 4e-3 #delay ms (250 Hz) 
    # delay = 20e-3 #delay ms (50 Hz)
    # sleep(delay)

    # calculate the time each iteration takes
    time_u = (dt.datetime.now() - start).microseconds

    # if frequency > 50:
    #     sleep(20e-3 - time_u/1e6)

    if logging_time:
        print "time (s): "+str(time_u/1e6)
    frequency = 1e6/time_u
    if logging_time:
        print "frequency (Hz): "+str(frequency)

    # average frequency of the loop
    frequencies.append(frequency)
    sum = 0
    for f in frequencies:
        sum+=f
    if logging_time:
        print "average frequency (Hz): "+str(sum/len(frequencies))


############################
############################


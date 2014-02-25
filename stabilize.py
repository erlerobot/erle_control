# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: Code for stabilizing the quadrotor. It runs and infinite
loop that adjust the 4 motors according to the IMU readings.

This code is implementing the control strategy desribed by Menno Wierema
in his MS thesis: "Design, implementation and flight test of indoor navigation and control
for a quadrotor UAV"
'''

from imu import IMU
from motors import Motor
from pid import PID
from time import sleep
import datetime as dt
from dynamical_model import Dynamical_Model


""" Limits the thrust passed to the motors
    in the range (-100,100)
"""
def limitThrust(thrust, upperLimit = 100, lowerLimit = 0):
    if thrust > upperLimit:
        thrust = upperLimit
    elif thrust < lowerLimit:
        thrust = lowerLimit
    return thrust

############################
# logging variables
############################

logging = 1
logging_time = 1
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
rollPID=PID(0.9, 0.2, 0.3) # Kp, Kd, Ki
pitchPID=PID(0.9, 0.2, 0.3)
yawPID=PID(0.06, 0.02, 0.01)
#zPID=PID(8, 10, 5)
#xposPID=PID(-0.09, -0.1, 0)
#yposPID=PID(-0.09, -0.1, 0)

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

    # pitch, roll and yaw DESIRED:
    #  FOR NOW THEY ARE KEPT TO 0 BUT THIS INFORMATION SHOULD
    #  COME FROM THE TELEOPERATION DEVICE/MISSION SYSTEM
    roll_d = 0
    pitch_d = 0
    yaw_d = 0
    #z_d = 0
    #xpos = 0
    #ypos = 0
    
    #Measure angles    s
    #roll_m, pitch_m, yaw_m = imu.read_fusedEuler()
    roll_m, pitch_m, yaw_m = imu.read_fusedEuler(0)
    #MyKalman.measure([roll,pitch, yaw])
    
    #Run the PIDs
    roll = rollPID.update(roll_d - roll_m, 0)
    pitch = pitchPID.update(pitch_d - pitch_m, 0)
    yaw = yawPID.update(yaw_d - yaw_m, 0)
    #z = zPID.update(z_d - z_m)
    #xpos = xposPID.update(xpos_d - xpos_m)
    #ypos = yposPID.update(ypos_d - ypos_m)

    #TODO change this parameter and see the behaviour
    #thrust is provided by the controller (NOTE: this is also treated as "z" and it should use the zPID controller)
    # the point of hovering is 35% duty cycle in the motors
    thrust = 1

    if logging:
        #Log the values:
        print "**************************"
        print "Desired angles:"
        print "     pitch:" + str(pitch_d)
        print "     roll:" + str(roll_d)
        print "     yaw:" + str(yaw_d)    
        print "Measured angles:"
        print "     pitch:" + str(pitch_m)
        print "     roll:" + str(roll_m)
        print "     yaw:" + str(yaw_m)
        print "PID output angles:"
        print "     pitch:" + str(pitch)
        print "     roll:" + str(roll)
        print "     yaw:" + str(yaw)
        print "thrust:" + str(thrust)
    

    # use the dynamical_model, returns u=[u_m1, u_m2, u_m3, u_m3]
    u = dyn_model.motor_inversion(thrust, roll, pitch, yaw)

    if logging:
        #Log the motor powers:
        print "------------------------"
        print "u1 (motor1):" + str(u[0])
        print "u2 (motor2):" + str(u[1])
        print "u3 (motor3):" + str(u[2])
        print "u4 (motor4):" + str(u[3])
        print "**************************"

    """
    #QUAD_FORMATION_NORMAL first approach        
    motorPowerM1 = limitThrust(thrust + pitch + yaw, 40);
    motorPowerM2 = limitThrust(thrust - roll - yaw, 40);
    motorPowerM3 =  limitThrust(thrust - pitch + yaw, 40);
    motorPowerM4 =  limitThrust(thrust + roll - yaw, 40);

    if logging:
        #Log the motor powers:
        print "------------------------"
        print "motorPowerM1:" + str(motorPowerM1)
        print "motorPowerM2:" + str(motorPowerM2)
        print "motorPowerM3:" + str(motorPowerM3)
        print "motorPowerM4:" + str(motorPowerM4)
        print "**************************"
    """


    
    #Set motor speeds
    motor1.setSpeedBrushless(u[0])
    motor2.setSpeedBrushless(u[1])
    motor3.setSpeedBrushless(u[2])
    motor4.setSpeedBrushless(u[3])
    
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


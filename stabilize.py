'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches
@description: Code for stabilizing the quadrotor. It runs and infinite
loop that adjust the 4 motors according to the IMU readings.
'''

from imu_class import IMU
from sensor_class import Sensor

""" Limits the thrust passed to the motors
    in the range 0-100
"""
def limitThrust():
    #TODO implement
    pass

#instantiate IMU
#TODO see how to import C interface to python
imu=
#MyKalman=KalmanFilter(....)

#instantiate motors and put them together in a list
#TODO create motor class + api
motor1=motor(....)
motor2=motor(....)
motor3=motor(....)
motor4-motor(....)
motors=[motor1,motor2,motor3,motor4]

#TODO instantiate PID controllers
RollPID=PID(.....)
PitchPID=PID(.....)

############################
#loop
############################
while 1:
    #Measure
    roll_m, pitch_m, yaw_m = imu.read(...)
    #MyKalman.measure([roll,pitch, yaw])
    
    #Run the PIDs
    roll = rollPID.update(roll - roll_m)
    pitch = pitchPID.update(pitch - pitch_m)
    yaw = yawPID.update(yaw - yaw_m)
    #z = zPID.update(z - z_m)
    #xpos = xposPID.update(xpos - xpos_m)
    #ypos = yposPID.update(ypos - ypos_m)

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

    #QUAD_FORMATION_NORMAL
    motorPowerM1 = limitThrust(thrust + pitch + yaw);
    motorPowerM2 = limitThrust(thrust - roll - yaw);
    motorPowerM3 =  limitThrust(thrust - pitch + yaw);
    motorPowerM4 =  limitThrust(thrust + roll - yaw);
 
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
    delay = 4 #delay ms 
    time.sleep(delay)

############################
############################


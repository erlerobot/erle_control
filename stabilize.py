'''
@author: Víctor Mayoral Vilches
'''

from imu_class import IMU
from sensor_class import Sensor

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
    roll, pitch, yaw = imu.read(...)
    #MyKalman.measure([roll,pitch, yaw])
    
    #Get control inputs
    u = rollPID.update(roll)
    u = pitchPID.update(pitch)
    u = yawPID.update(yaw)
    u = zPID.update(z)
    u = xposPID.update(xpos)
    u = yposPID.update(ypos)
    
    #Set motor speeds
    motor1.setSpeed(u)
    motor2.setSpeed(u)
    motor3.setSpeed(u)
    motor4.setSpeed(u)
    
    #Start Motors
    for mot in motors:
        mot.go()
    
    #Kalman Prediction
    #MyKalman.predict()
    delay = 4 #delay ms 
    time.sleep(delay)

############################
############################


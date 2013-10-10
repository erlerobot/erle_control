# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: python interface with the MPU-9150.
'''
from ctypes import *
#from time import clock

class IMU:
    """ Interface with the Inertial Measurement Unit. 
    
    The IMU consists of an InvenSense 9-axis MPU-9150. This sensor provides
    readings from 3 accelerometers, 3 magnetometers and 3 gyroscopes.
    Furthermore, the module has a DMP (Digital Motion Processor) integrated
    that makes the calculations necessary to provide filtered outputs.
    """
    def __init__(self):
        #TODO Set I2C interface, make sure that calibrations files are available and make some readings
        # through ctypes.
        self.lib = CDLL("./imu/libimu.so")
        self.i2c_bus = 2
        self.lib.mpu9150_set_debug(0) # 1
        self.sample_rate = 50 # 50 Hz
        self.yaw_mix_factor = 3

        # initialize the IMU
        res = self.lib.mpu9150_init(self.i2c_bus, self.sample_rate, self.yaw_mix_factor)
        if res:
            Exception("Error when initializing the IMU!")
        # set calibration files
        res = self.lib.set_cal(0, "./imu/accelcal.txt")
        if res != 0:
            Exception("Error while calibration: accelcal.txt")
        res = self.lib.set_cal(0, "./imu/magcal.txt")
        if res != 0:
            Exception("Error while calibration: magcal.txt")


    """ Reads the raw gyro data from the sensor
        @return  gyroX, gyroY, gyroZ
    """
    def read_rawGyro(self):    
        #start = clock()
        while 1:
                # Parameters to be passed by reference
                x = c_short(0)
                y = c_short(0)
                z = c_short(0)
                function = self.lib.read_rawGyro
                function.argtypes = [POINTER(c_short), POINTER(c_short), POINTER(c_short)]
                res = self.lib.read_rawGyro(byref(x), byref(y), byref(z)) 
                if res == 0:
                        #time_s = clock() - start
                        #print time_s
                        return x.value, y.value, z.value



    """ Reads data from the FIFO list (250 Hz max?).
    """
    def fifoRead(self):
        #TODO implement
        pass

    """ Reads raw data (withouth using DMP).
    """
    def rawRead(self):
        #TODO implement
        pass

    def calibration(self):
        pass

"""
imu = IMU()
gyrox, gyroy, gyroz = imu.read_rawGyro()
print gyrox
print gyroy
print gyroz
"""

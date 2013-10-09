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
        res = self.lib.mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor)
        if res:
            Exception("Error when initializing the IMU!")
        # set calibration files
        res = self.lib.set_cal(0, "./imu/accelcal.txt")
        if res != 0:
            Exception("Error while calibration: accelcal.txt")
        res = self.lib.set_cal(0, "./imu/magcal.txt")
        if res != 0:
            Exception("Error while calibration: magcal.txt")


    """ Reads the fused data from the sensor
        @return roll, pitch, yaw
    """
    def fusedRead(self):
        """
        # Code for passing a parameter directly to a C function
        # however it doesn't seem to work. Some debuggin might be neccesary
        mpu = Mpudata_t()
        self.lib.mpu9150_read(addressof(mpu))
        """
        #TODO create C functions that return


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
# array class
Vector3d_t = 3*c_float
# array class
Quaternion_t = 4*c_float
# struct 
class Mpudata_t(Structure):
         _fields_ = [("rawGyro", c_short*3),
                     ("rawAccel", c_short*3),
                     ("rawQuat", c_long*4),
                     ("dmpTimestamp", c_ulong*4),
                     ("rawMag", c_short*4),
                     ("magTimestamp", c_ulong*4),
                     ("calibratedAccel", c_short*4),
                     ("calibratedMag", c_short*4),
                     ("fusedQuat", Quaternion_t),
                     ("fusedEuler", Vector3d_t),
                     ("lastDMPYaw", c_float),
                     ("lastYaw", c_float)
                    ]
"""
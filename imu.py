'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches
@description: python interface with the MPU-9150.
'''

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
        pass

    """ Reads the data from the I2C interface (50 Hz max).
    """
    def i2cRead(self):
        #TODO implement
        pass

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


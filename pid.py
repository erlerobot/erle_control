# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: PID controller class. 
'''
import time

class PID:
    """ Simple PID control.

        This class implements a simplistic PID control algorithm. When first
        instantiated all the gain variables are set to zero, so calling
        the method GenOut will just return zero.
    """
    def __init__(self):
        # initialze gains
        self.Kp = 1
        self.Kd = 0
        self.Ki = 0

        self.Initialize()

    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def update(self, error, debug = 0):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        if debug:
            print "   ****************************"
            print "   PID Debug"
            print "   ****************************"
            print "   error:"+str(error)

        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        if debug:
            print "   dt:"+str(dt)
        de = error - self.prev_err              # get delta error
        if debug:
            print "   de:"+str(de)

        self.Cp = error               # proportional term
        if debug:
            print "   Proportional term (Cp*Kp):"+str(self.Cp)
        self.Ci += error * dt                   # integral term
        if debug:
            print "   Integral term (Ci*Ki):"+str(self.Ci * self.Ki)

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term
            if debug:
                print "   Derivative term (Cd*Kd):"+str(self.Cd * self.Kd)

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        terms_sum = self.Cp * self.Kp  + (self.Ki * self.Ci) + (self.Kd * self.Cd)
        if debug:
            print "   Terms sum (self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)):"+str(terms_sum)        
            print "   ****************************"
        return terms_sum

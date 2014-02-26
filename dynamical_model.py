# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: This class implements the dynamical model of a quadrotor. 

It does so following the dynamical model studied at the Master Thesis "Design, 
implementation and ﬂight test of indoor navigation and control system for a 
quadrotor UAV"
(http://www.st.ewi.tudelft.nl/~koen/in4073/Resources/MSc_thesis_X-UFO.pdf)

The implementation of the Crazyflie is also reflected on motor_inversion2
'''

import math
import numpy as np

class Dynamical_Model:
    def __init__(self): 
        # general paramters
        self.g = 9.806 # Gravity constant [m s^-2 ]
        self.rho = 1.293 # Air density [kg m^-3]
        self.nu = 1.8e-5 # Air viscosity at 20 degrees Celsius [Pa s]

        # quadrotor parameters
        self.P = 4 # number of propellers
        self.L = 29.9974e-3 # Arm length [m]
        self.Vol = 0.00281784516 # Volume [m3] (((86.36*86.36)/1000) * 
        self.m = 60e-3 # quadrotor mass [kg]
        self.h = 17e-3 # Vertical distance between CoG and propellers plan [m]                         #                  (8*29.99/1000) * (1.5748/1000))       
        self.b = 3.13e-5 # thrust factor in hover [N s^2]
        self.d = 7.5e-7 # drag factor in hover [N m s^2]
        self.W_prop = (self.m * self.g)/self.P # weight of the quadrotor per propeller [N]
        self.Omega_H = math.sqrt(self.W_prop/self.b) # propeller speed at hover

        # propellers
        self.N = 3 # Number of blades per propeller
        self.R = 32.5e-3 # Propeller radius [m]
        self.A = math.pi*math.pow(self.R, 2)# Propeller disk area [m^2]
        self.c = 0.0394 # Chord [m]
        self.theta_0 = 0.2618 # Pitch of incidence [rad]
        self.theta_tw = 0.045 # Twist pitch [rad]
        self.sigma = self.N * self.c/(math.pi * self.R) # Solidity ratio (rotor fill ratio) [rad^-1]
        self.a = 5.7 # Lift slope
        self.C_d = 0.052 # Airfoil drag coefficient
        self.A_c = 0.005 # Helicopter center hub area [m^2]
        # Longitudinal drag coefficients
        self.Cx = 1.32
        self.Cy = 1.32
        self.Cz = 1.32

        # Inertia components [kg m^2]
        self.Ixx = 6.228e-3
        self.Iyy = 6.228e-3
        self.Izz = 1.121e-2

        # motor parameters
        # TODO complete the motor parameters        
        self.k_m = 1 #TODO # torque constant
        self.tau = 1 #TODO # motor time constant
        self.eta = 1 #TODO # motor efficiency
        self.Omega_0 = 1 #TODO # point of linearization of the rotor speeds

        self.r = 4 # Reduction ratio
        self.J_t = 6.0100e-5 # Rotor inertia [kg m^2]

        # matrix for calculating the motor voltages from the control inputs
        self.m = np.matrix( ((1/(4*self.b),0, 1/(2*self.b), -1/(4*self.b)), 
                        (1/(4*self.b),-1/(2*self.b), 0, 1/(4*self.b)), 
                        (1/(4*self.b),0, -1/(2*self.b), -1/(4*self.b)), 
                        (1/(4*self.b),1/(2*self.b), 0 ,  1/(4*self.b))) )



    """ Compute the motor voltages from the control inputs following M.Wieremma MS thesis (MSc_thesis_X-UFO). Keep in mind when
        passing parameters the following correspondences.
            - U1: thrust
            - U2: roll
            - U3: pitch
            - U4: yaw

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion1(self, thrust, roll, pitch, yaw, logging = 0):
        # the control inputs
        U = np.array( ((thrust, roll, pitch, yaw)) )
        Um = np.matrix(U).T
        # the motor voltages
        u = (self.k_m * self.tau) * ((1/self.tau + 2*self.d*self.Omega_0/(self.eta*np.power(self.r,3)*self.J_t))\
            * np.sqrt(np.dot(self.m,U))- self.d*np.power(self.Omega_0,3)/(self.eta*np.power(self.r,3)*self.J_t))

        # u comes in the form [[ 351.0911185   117.65355114  286.29403363  nan]] where nan denotes that this value
        # should be put to 0
        # values goes more or less up to 1500 so they are divided by 15 so that they fall in the 0-100 range.

        if math.isnan(u[0,0]): 
            motorPowerM1 = 0
        else:
            motorPowerM1 = u[0,0]/15 

        if math.isnan(u[0,1]):
            motorPowerM2 = 0
        else:
            motorPowerM2 = u[0,1]/15

        if math.isnan(u[0,2]):
            motorPowerM3 = 0
        else:
            motorPowerM3 = u[0,2]/15 

        if math.isnan(u[0,3]):
            motorPowerM4 = 0
        else:
            motorPowerM4 = u[0,3]/15 

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 1):" + str(motorPowerM1)
            print "motorPowerM2 (method 1):" + str(motorPowerM2)
            print "motorPowerM3 (method 1):" + str(motorPowerM3)
            print "motorPowerM4 (method 1):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur

    """ Compute the motor voltages from the control inputs 
        following bitcraze Crazyflie implementation.

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion2(self, thrust, roll, pitch, yaw, logging = 0):
        #QUAD_FORMATION_NORMAL
        motorPowerM1 = thrust + pitch + yaw
        motorPowerM2 = thrust - roll - yaw
        motorPowerM3 = thrust - pitch + yaw
        motorPowerM4 = thrust + roll - yaw

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 2):" + str(motorPowerM1)
            print "motorPowerM2 (method 2):" + str(motorPowerM2)
            print "motorPowerM3 (method 2):" + str(motorPowerM3)
            print "motorPowerM4 (method 2):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited        
        return ur


    """ Compute the motor voltages from the control inputs 
        using a HACKED version of the implementation used in the crazyflie.
        This hack is because the reference frames of Erle are different from the ones
        adopted by the crazyflie.

        M1 <-> M3
        M2 <-> M4

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion3(self, thrust, roll, pitch, yaw, logging = 0):
        #QUAD_FORMATION_NORMAL
        motorPowerM3 = thrust + pitch + yaw
        motorPowerM4 = thrust - roll - yaw
        motorPowerM1 = thrust - pitch + yaw
        motorPowerM2 = thrust + roll - yaw

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 3):" + str(motorPowerM1)
            print "motorPowerM2 (method 3):" + str(motorPowerM2)
            print "motorPowerM3 (method 3):" + str(motorPowerM3)
            print "motorPowerM4 (method 3):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited        
        return ur

    """ Compute the motor voltages from the control inputs following M.Wieremma MS thesis (MSc_thesis_X-UFO). 

        THE DYNAMICAL MODEL DOCUMENTED IN THE MS Thesis HAS BEEN HACKED (M1 <-> M3, M2 <-> M4) BECAUSE THE REFERENCE FRAMES ADOPTED
        IN THE DOCUMENT INDICATES THAT THE MOTORS ROTATE IN OPPOSITE DIRECTIONS.

        Keep in mind when passing parameters the following correspondences.

            - U1: thrust
            - U2: roll
            - U3: pitch
            - U4: yaw

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion4(self, thrust, roll, pitch, yaw, logging = 0):
        # the control inputs
        U = np.array( ((thrust, roll, pitch, yaw)) )
        Um = np.matrix(U).T
        # the motor voltages
        u = (self.k_m * self.tau) * ((1/self.tau + 2*self.d*self.Omega_0/(self.eta*np.power(self.r,3)*self.J_t))\
            * np.sqrt(np.dot(self.m,U))- self.d*np.power(self.Omega_0,3)/(self.eta*np.power(self.r,3)*self.J_t))

        # u comes in the form [[ 351.0911185   117.65355114  286.29403363  nan]] where nan denotes that this value
        # should be put to 0
        # values goes more or less up to 1500 so they are divided by 15 so that they fall in the 0-100 range.

        # FIXED APPLIED DUE TO THE DIFFERENCE IN THE REFERENCE FRAMES

        if math.isnan(u[0,0]): 
            motorPowerM3 = 0
        else:
            motorPowerM3 = u[0,0]/15 

        if math.isnan(u[0,1]):
            motorPowerM4 = 0
        else:
            motorPowerM4 = u[0,1]/15

        if math.isnan(u[0,2]):
            motorPowerM1 = 0
        else:
            motorPowerM1 = u[0,2]/15 

        if math.isnan(u[0,3]):
            motorPowerM2 = 0
        else:
            motorPowerM2 = u[0,3]/15 

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 4):" + str(motorPowerM1)
            print "motorPowerM2 (method 4):" + str(motorPowerM2)
            print "motorPowerM3 (method 4):" + str(motorPowerM3)
            print "motorPowerM4 (method 4):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur


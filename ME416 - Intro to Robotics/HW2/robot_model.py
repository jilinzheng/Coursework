"""Functions for modeling ROSBot"""

# from math import cos, sin
# import numpy as np


def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d

def twist_to_speeds(speed_linear, speed_angular):
    """
    Calculates the speed of each wheel 
    given the currect linear and rotational speed of the robot
    """
    # gives the code a range around zero,
    # could be decreased dependent on the noise of the data recieveds
    close0 = 0.05

    # if angular speed is around zero, both the speed of the left and right motors have to be equal
    if -close0 < speed_angular < close0:
        speed_left = speed_linear
        speed_right = speed_left

    # linear speed is the motor speed closest to zero,
    # and angular speed tells which motor is greater
    elif close0 < speed_angular < 1:
        #in this case, assuming right motor has greater output
        speed_left = speed_linear
        #adding arbtiuary ratio including angular speed
        speed_right = speed_linear * (1+speed_angular)

    # in this case, assuming left motor is greater, adding the same ratio to left motor
    elif -1 < speed_angular < -close0:
        speed_left = speed_linear * (1+speed_angular)
        speed_right = speed_linear

    # impossible to have angular speed of 1 with linear speed as
    elif speed_angular == 1:
        # that would require that one of the motors exceed their max output
        speed_left = 0
        speed_right = 1

    # same story here
    elif speed_angular == -1:
        speed_left = 1
        speed_right = 0
    return speed_left, speed_right

def system_matrix(theta):
    # pylint: disable=all
    '''Returns a numpy array with the A(theta) matrix for a differential drive robot'''
    return A

def system_field(z, u):
    # pylint: disable=all
    '''Computes the field at a given state for the dynamical model'''
    return dot_z

def euler_step(z, u, step_size):
    # pylint: disable=all
    '''Integrates the dynamical model for one time step using Euler's method'''
    return zp

class KeysToVelocities():
    '''
    Class to translate cumulative key strokes to speed commands
    '''
    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.SPEED_DELTA = 0.2
        self.text_description = 'Initial values'

    def limit_to_threshold(self, speed):
        '''
        Helper function to threshold the input speed between 1.0 and -1.0
        '''
        if speed > 1.0:
            speed = 1.0
        elif speed < -1.0:
            speed = -1.0

        return speed

    def update_speeds(self, key):
        '''
        Increments/decrements speeds given an input key.
        '''
        if key in ('w', 'W'):
            self.speed_linear += self.SPEED_DELTA
            self.speed_linear = self.limit_to_threshold(self.speed_linear)
            self.text_description = 'Increase linear speed'
        elif key in ('s', 'S'):
            self.speed_linear -= self.SPEED_DELTA
            self.speed_linear = self.limit_to_threshold(self.speed_linear)
            self.text_description = 'Decrease linear speed'
        elif key in ('a', 'A'):
            self.speed_angular += self.SPEED_DELTA
            self.speed_angular = self.limit_to_threshold(self.speed_angular)
            self.text_description = 'Increase angular speed'
        elif key in ('d', 'D'):
            self.speed_angular -= self.SPEED_DELTA
            self.speed_angular = self.limit_to_threshold(self.speed_angular)
            self.text_description = 'Decrease angular speed'
        elif key in ('z', 'Z'):
            self.speed_linear = 0.0
            self.text_description = 'Set linear speed to zero'
        elif key in ('c', 'C'):
            self.speed_angular = 0.0
            self.text_description = 'Set angular speed to zero'
        elif key in ('x', 'X'):
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            self.text_description = 'Set linear and angular speed to zero'
        elif key in ('q', 'Q'):
            self.text_description = 'Quit key translation to velocities'
        else:
            self.text_description = 'Invalid command'

        return self.speed_linear, self.speed_angular, self.text_description

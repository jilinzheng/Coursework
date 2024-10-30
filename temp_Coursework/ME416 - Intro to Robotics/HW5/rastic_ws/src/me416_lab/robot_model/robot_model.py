""" Functions for modeling ROSBot """


from math import cos, sin
import numpy as np
from me416_utilities import stamp_difference


def model_parameters():
    """Returns two constant model parameters"""
    param_k = 0.35
    param_d = 0.06
    #param_k = 3.0
    #param_d = 3.0
    return param_k, param_d

def twist_to_speeds(speed_linear, speed_angular):
    """
    Calculates the speed of each wheel 
    given the currect linear and rotational speed of the robot
    """
    [k, d] = model_parameters()

    speed_left = (1./k) * speed_linear
    speed_right = speed_left

    speed_left -= (d/k) * speed_angular
    speed_right += (d/k) * speed_angular
    
    # limit speed_left between [-1, 1]
    if speed_left < -1.0:
        speed_left = -1.0
    elif speed_left > 1.0:
        speed_left = 1.0

    # limit speed_right between [-1, 1]
    if speed_right < -1.0:
        speed_right = -1.0
    elif speed_right > 1.0:
        speed_right = 1.0

    return speed_left, speed_right

def system_matrix(theta):
    '''Returns a numpy array with the A(theta) matrix for a differential drive robot'''
    [k, d] = model_parameters()
    A = (k/2) * np.array([[cos(theta), cos(theta)],
                  [sin(theta), sin(theta)],
                  [-1./d, 1/d]])
    return A

def system_field(z, u):
    '''Computes the field at a given state for the dynamical model'''
    return dot_z

def euler_step(z, u, step_size):
    '''Integrates the dynamical model for one time step using Euler's method'''
    A = system_matrix(z[2, 0]) # [2][0] is the theta/angle of z
    return (A@u) * step_size + z

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
            self.speed_linear = self.speed_linear + self.SPEED_DELTA
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

class StampedMsgRegister():
    """ Computes the delay, in seconds, betweeen two ROS messages. """
    def __init__(self):
        """
        Initialize the internal variables msg_previous to None
        """
        self.msg_previous = None

    def replace_and_compute_delay(self, msg):
        """
        Given a new stamped message as input,
        computes the delay (in seconds) between
        the time stamp of this message and the value in time_previous,
        and then replaces the internal copy of the previous message with the current message.
        """
        # save old message to msg_previous
        """msg_previous = self.msg_previous"""

        # only calculate the time delay if msg_previous is not None
        """if msg_previous is not None:
            time_delay = stamp_difference(msg.header.stamp, msg_previous.header.stamp)
        # set the time_delay to a tuple of 0 seconds and nanoseconds
        # if there was no previous message
        else:
            time_delay = (0,0)

        # save new message to previous message
        self.msg_previous = msg"""

        if self.msg_previous is None:
            msg_previous = None
            self.msg_previous = msg
            time_delay = 0
        else:
            msg_previous = self.msg_previous
            self.msg_previous = msg
            time_delay = stamp_difference(msg.header.stamp,
                                             msg_previous.header.stamp)

        return time_delay, msg_previous
    
    def previous_stamp(self):
        return None if self.msg_previous is None else self.msg_previous.header.stamp

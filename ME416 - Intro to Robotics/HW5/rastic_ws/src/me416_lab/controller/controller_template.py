#!/usr/bin/env python
""" A first template for a PID controller """


class PID():
    """ Computes proportional, derivative, and integral terms for a PID controller """
    def __init__(self, gain_kp=1.0, gain_kd=1.0, gain_ki=1.0):
        """Initializes gains and internal state (previous error and integral error)"""
        self.gain_kp = gain_kp
        self.gain_kd = gain_kd
        self.gain_ki = gain_ki

        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        """ Compute proportional term (with gain) """
        #TODO: This is a stub. Write your code here.
        return control_p

    def integral(self, error_signal, time_delay):
        """ Compute integral term (with gain) """
        #TODO: This is a stub. Write your code here.
        return control_i

    def derivative(self, error_signal, time_delay):
        """ Compute derivative term (with gain) """
        #TODO: This is a stub. Write your code here.
        return control_d

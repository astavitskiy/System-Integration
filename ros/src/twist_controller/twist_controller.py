#!/usr/bin/env python

from pid import PID
from yaw_controller import YawController

#GAS_DENSITY = 2.858
#ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.linController = PID(0.5, 0.01, 0.2, 0, 1)
        self.yawController = YawController(wheel_base, steer_ratio, min_speed, 
        										max_lat_accel, max_steer_angle)
        pass

    def control(self, propLinVel, propAngVel, curLinVel, dbwEnabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linVelError = propLinVel - curLinVel       
        throttle = self.linController.step(linVelError, 0.02)
        steer = self.yawController.get_steering(propLinVel, propAngVel, curLinVel)
        print(throttle, steer)
        return throttle, 0., steer

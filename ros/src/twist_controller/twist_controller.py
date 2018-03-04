#!/usr/bin/env python

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_radius = kwargs['wheel_radius']
        self.accel_filter = LowPassFilter(tau=3.0, ts=1.0)
        self.steer_filter = LowPassFilter(tau=3.0, ts=1.0)
        self.total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY

        self.linController = PID(0.5, 0.01, 0.2, kwargs['decel_limit'], kwargs['accel_limit'])
        self.yawController = YawController(kwargs['wheel_base'], 
                                           kwargs['steer_ratio'],
                                           kwargs['min_speed'],
                                           kwargs['max_lat_accel'],
                                           kwargs['max_steer_angle'])
        pass


    def control(self, propLinVel, propAngVel, curLinVel, dbwEnabled, dt):
        linVelError = propLinVel - curLinVel

        # Lateral Control
        steer = self.yawController.get_steering(propLinVel, propAngVel, curLinVel)
        steer = self.steer_filter.filt(steer)
        
        # Longitudinal Control
        linAccel = self.linController.step(linVelError, dt)
        if linAccel > 0.0:
            brake = 0.0
            throttle = self.accel_filter.filt(linAccel)
        else:
            throttle = 0.0
            linDecel = abs(linAccel)
            brake = linDecel * self.total_mass * self.wheel_radius
            if linDecel < self.brake_deadband:
                brake = 0.0

        print(throttle, steer)
        return throttle, brake, steer

from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, freq):
        # TODO: Implement
        self.sample_time = 1. / freq
        self.yaw_controller = yaw_controller
        self.throttle_pid = PID(self.sample_time * 1.5, 0.002, 0.0001, 0., 1.)
        self.brake_pid = PID(self.sample_time, 0.002, 0.0001, 0., 1.)

    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        err = linear_velocity - current_velocity
        throttle = self.throttle_pid.step(err, self.sample_time)
        brake = self.brake_pid.step(-err, self.sample_time)

        return throttle, brake, steer

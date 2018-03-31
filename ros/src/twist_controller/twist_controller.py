from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, freq):
        # TODO: Implement
        self.sample_time = 1. / freq
        self.yaw_controller = yaw_controller
        self.throttle_pid = PID(self.sample_time * 1.5, 0.002, 0., 0., 0.4)
        self.brake_pid = PID(self.sample_time*1.5, 0.002, 0., 0., 0.4)

	self.throttle_filt = LowPassFilter(1.,1.)
	self.brake_filt = LowPassFilter(1.,1.)
	self.steer_filt = LowPassFilter(1.,1.)

    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        err = linear_velocity - current_velocity
        throttle = self.throttle_pid.step(err, self.sample_time)
        brake = self.brake_pid.step(-err, self.sample_time)

	steer = self.steer_filt.filt(steer)
	if(current_velocity<linear_velocity):
		throttle = self.throttle_filt.filt(throttle)
		brake = 0.
	else:
		if(linear_velocity<0.1):
			throttle = 0.
			brake = 0.4
		else:
			throttle = 0.
			brake = self.brake_filt.filt(brake)



        return throttle, brake, steer

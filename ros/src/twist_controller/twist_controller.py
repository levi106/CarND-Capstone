
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller):
        # TODO: Implement
        self.yaw_controller = yaw_controller

    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        throttle = 0.
        brake = 0.
        if linear_velocity > current_velocity:
            throttle = min((linear_velocity - current_velocity) / 10., 0.5)
        elif linear_velocity < current_velocity:
            brake = -min((current_velocity - linear_velocity) / 10., 3.)
        return throttle, brake, steer

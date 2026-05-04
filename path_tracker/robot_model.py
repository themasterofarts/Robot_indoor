import math

class DifferentialRobot:

    def __init__(self, wheel_base=0.3):
        self.wheel_base = wheel_base

    def update(self, state, v_left, v_right, dt):
        v     = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        new_x     = state['x'] + v * math.cos(state['theta']) * dt
        new_y     = state['y'] + v * math.sin(state['theta']) * dt
        new_theta = state['theta'] + omega * dt

        return {
            'x'    : new_x,
            'y'    : new_y,
            'theta': new_theta,
            'v'    : v
        }

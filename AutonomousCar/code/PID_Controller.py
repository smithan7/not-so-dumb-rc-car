import math

from step_response import step_response_metrics


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_error_sum=math.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # anti-windup limit on the integrated error; inf = no limit (was 0, which disabled ki entirely)
        self.max_error_sum = max_error_sum
        self.reset()

    def reset(self):
        self.s = 0.0
        self.s_d = 0.0
        self.s_e = 0.0
        self.u_p = 0.0
        self.u_i = 0.0
        self.u_d = 0.0
        self.u = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.error_rate = 0.0
        self.initialized = False

    def tic(self, s, s_d, dt=1.0):
        '''One controller step. Pass the real dt (s) so gains don't depend on the loop rate.

        dt defaults to 1.0 so gains tuned against the old per-tick implementation behave the same.
        '''
        self.s = s
        self.s_d = s_d
        self.s_e = s_d - s
        self.u_p = self.kp * self.s_e

        self.u_d = 0.0
        if self.initialized:
            self.error_rate = (self.s_e - self.last_error) / dt
            self.u_d = self.kd * self.error_rate
        self.initialized = True
        self.last_error = self.s_e

        self.error_sum += self.s_e * dt
        self.error_sum = self.clamp(self.error_sum, -self.max_error_sum, self.max_error_sum)
        self.u_i = self.ki * self.error_sum

        self.u = self.u_p + self.u_i + self.u_d
        return self.u

    def clamp(self, x, x_min, x_max):
        '''This ensures  x_min <= x <= x_max'''
        return min(x_max, max(x_min, x))

    def extract_performance(self, ts, ss, s_d):
        return step_response_metrics(ts, ss, s_d)

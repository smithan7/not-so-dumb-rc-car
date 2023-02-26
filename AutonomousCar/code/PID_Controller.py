class PID:
    kp = 0.0
    ki = 0.0
    kd = 0.0
    s = 0.0
    s_d = 0.0
    s_e = 0.0
    u_p = 0.0
    u_i = 0.0
    u_d = 0.0
    u = 0.0
    
    error_sum = 0.0
    max_error_sum = 0.0
    last_error = 0.0
    error_rate = 0.0
    initialized = False
    
    def __init__(self):
        self.error_sum = 0.0
        self.last_error = 0.0
        self.initialized = False
        
    def tic(self, s, s_d):
        self.s = s
        self.s_d = s_d
        s_e = s_d - s
        self.u_p = self.kp * s_e
        
        self.u_d = 0.0
        if self.initialized:
            self.error_rate = s_e - self.last_error
            self.u_d = self.kd * self.error_rate
            self.last_error = s_e
        else:
            self.initialized = True
            self.last_error = s_e

        self.error_sum += s_e
        self.error_sum = self.clamp(self.error_sum, -self.max_error_sum, self.max_error_sum)
        self.u_i = self.ki * self.error_sum

        self.u = self.u_p + self.u_i + self.u_d        
        return self.u
    
    def clamp(self, x, x_min, x_max):
        '''This ensures  x_min <= x <= x_max'''
        return min(x_max, max(x_min, x))
    
    def extract_performance(self, ts, ss, s_d):
        # get pid inputs
        rise_time = 0.0
        rise_set = False
        settling_time = 0.0
        overshoot = 0.0
        rise_set = False
        s_range = s_d - ss[0]
        for t, s in zip(ts, ss):
            # get rise time
            if not rise_set and s > ss[0] + 0.7 * s_range:
                rise_time = t
                rise_set = True
            
            # get overshoot
            if (s - s_d)/s_range > overshoot:
                overshoot = (s-s_d) / s_range
            
            # get settling time
            if abs(s_d - s)/s_range > 0.02:
                settling_time = t

        return [rise_time, overshoot, settling_time]
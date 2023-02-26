class Control_Response_Evaluator:

    def __init__(self):
        pass

    def evaluate_performance(self, ts, ss, sd):
        # get pid inputs
        rise_time = 0.0
        rise_set = False
        settling_time = 0.0
        overshoot = 0.0
        rise_set = False
        s_range = sd - ss[0]
        for t, s in zip(ts, ss):
            # get rise time
            if not rise_set and abs(sd - s) < abs(0.3 * s_range):
                rise_time = t
                rise_set = True
            
            # get overshoot
            if (s - sd)/s_range > overshoot:
                overshoot = (s-sd) / s_range
            
            # get settling time
            if abs(sd - s)/abs(s_range) > 0.02:
                settling_time = t

        return [rise_time, overshoot, settling_time]
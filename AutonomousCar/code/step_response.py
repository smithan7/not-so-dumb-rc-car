def step_response_metrics(ts, ss, s_d, rise_fraction=0.7, settle_band=0.02):
    """Return [rise_time, overshoot, settling_time] for a step from ss[0] to s_d.

    Works for steps in either direction. rise_time is the first time the response has
    covered ``rise_fraction`` of the step; overshoot is the fraction of the step travelled
    past s_d; settling_time is the last time the response was outside ``settle_band`` of s_d.
    """
    s_range = s_d - ss[0]
    if s_range == 0:
        return [0.0, 0.0, 0.0]

    rise_time = 0.0
    rise_set = False
    settling_time = 0.0
    overshoot = 0.0
    for t, s in zip(ts, ss):
        progress = (s - ss[0]) / s_range  # 0 at start, 1 at target, >1 past it
        if not rise_set and progress >= rise_fraction:
            rise_time = t
            rise_set = True
        overshoot = max(overshoot, progress - 1.0)
        if abs(s_d - s) / abs(s_range) > settle_band:
            settling_time = t
    return [rise_time, overshoot, settling_time]

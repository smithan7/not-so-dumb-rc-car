from PID_Controller import PID


class CarController:
    def __init__(self):
        # per-instance controllers; class-level PIDs would share state between every CarController
        self.speed_pid = PID()
        self.steer_pid = PID()
        self.crosstrack_pid = PID()

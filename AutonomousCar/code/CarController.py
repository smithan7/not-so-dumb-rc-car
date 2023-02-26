from PID_Controller import PID

class CarController:
    speed_pid = PID()
    steer_pid = PID()
    crosstrack_pid = PID()
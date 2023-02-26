import math

mm_to_in = 0.03937008
in_to_mm = 1.0 / mm_to_in
mph_to_mps = 0.44704
in_to_m = 0.0254
ft_lbs_to_Nm = 1.0 / 1.355818
oz_in_to_ft_lbs = 1.0 / 192.00000012288
oz_in_to_Nm = 1.0 / 141.61193227806

class DriveTrain:
    motor_rpm = 0.0
    pinion = 34.0
    spur = 34.0
    differential_ratio = 3.308
    final_drive_ratio = differential_ratio * spur / pinion
    tire_diam = 0.100 # mm
    tire_radius = tire_diam / 2.0
    tire_circumference = math.pi * tire_diam 
    
    f_wheel = 0.0
    t_wheel = 0.0
    wheel_rpm = 0.0
    t_motor = 0.0
    motor_rpm = 0.0
    
    def __init__(self):
        pass
    
    def print(self):
        print("Drive Train: Pinion: ", self.pinion, "T")
        print("Drive Train: Spur: ", self.spur, "T")
        print("Drive Train: Differential Ratio: ", self.differential_ratio)
        print("Drive Train: Final Drive Ratio: ", self.final_drive_ratio, ": 1 (Motor Rotations:Wheel Rotation)")
        print("Drive Train: Tire Diameter: ", self.tire_diam, " m")
        print("Drive Train: Tire Circumference: ", self.tire_circumference, " m")
        
    def calc_car_speed_mph(self, motor_rpm):
        # calculate car speed based on drive train config and motor RPM
        self.motor_rpm = motor_rpm
        self.speed_mps = motor_rpm / self.final_drive_ratio * self.tire_circumference / 60.0
        return self.speed_mps / mph_to_mps
    
    def calc_car_speed_mps(self, motor_rpm):
        # calculate car speed based on drive train config and motor RPM
        self.motor_rpm = motor_rpm
        self.speed_mps = motor_rpm / self.final_drive_ratio * self.tire_circumference / 60.0
        return self.speed_mps
    
    def calc_motor_rpm_from_speed_mps(self, speed_mps):
        # calculate motor rpm from car speed based on drive train config
        self.speed_mps = speed_mps
        self.motor_rpm = speed_mps * 60 / self.tire_circumference * self.final_drive_ratio
        return self.motor_rpm
    
    def calc_wheel_rpm_from_motor_rpm(self, motor_rpm):
        self.motor_rpm = motor_rpm
        self.wheel_rpm = motor_rpm / self.final_drive_ratio
        return self.wheel_rpm
    
    def calc_motor_rpm_from_wheel_rpm(self, wheel_rpm):
        self.wheel_rpm = wheel_rpm
        self.motor_rpm = wheel_rpm * self.final_drive_ratio
        return self.motor_rpm
    
    def calc_wheel_torque_from_motor_torque(self, t_motor):
        self.t_motor = t_motor
        self.t_wheel = t_motor * self.final_drive_ratio
        return self.t_wheel

    def calc_wheel_force_from_wheel_torque(self, t_wheel):
        self.f_wheel = t_wheel / self.tire_radius
        return self.f_wheel
    
    def calc_wheel_force_from_motor_torque(self, t_motor):
        self.f_wheel = (t_motor * self.final_drive_ratio) / self.tire_radius
        return self.f_wheel
    
    def calc_motor_torque_from_wheel_torque(self, t_wheel):
        self.t_motor = t_wheel / self.final_drive_ratio
        return self.t_motor
    
    def calc_wheel_torque_from_wheel_force(self, f_wheel):
        self.f_wheel = f_wheel
        self.t_wheel = f_wheel * self.tire_radius
        return self.t_wheel
    
    def calc_motor_torque_from_wheel_force(self, f_wheel):
        self.f_wheel = f_wheel
        self.t_motor = (f_wheel * self.tire_radius) / self.final_drive_ratio
        return self.t_motor
        
        
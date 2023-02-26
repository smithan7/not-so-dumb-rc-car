import math

class Steering_Assembly:
    steer_angle = 0.0 # what is the seering wheel angle
    wheel_angle = 0.0 # what is the tire wheel angle
    
    steer_to_tire = 1.0 # ratio of steering wheel to tire wheel angles
    
    steer_rate = 0.0 # rate of steering wheel
    
    max_steer_angle = 0.5435 # rad
    max_steer_rate = 0.3294 # rad/s
    max_steer_accel = 0.1745 # rad/s^2
    
    lag_constant = 0.3 # delay in inputs
    
    inertia = 100.0
    spring = 10.0
    damping = 10.0
    
    def __init__(self):
        pass
    
    def get_steeing_wheel_from_tire_wheel(self, t_d):
        '''Calculates steering wheel angle from desired tire angle'''
        s_d = t_d / self.steer_to_tire
        return s_d 
    
    def simple_dynamics(self, s_d, dt):
        '''Takes in desired steering angle and moves the steering and tire angle in that direction'''
        #Effectively this acts as a servo in the car hooked to the steering wheel
        
        # get steering force
        u_s = 1 / self.lag_constant * (s_d - self.steer_angle) # user applied
        f_s = self.spring * self.steer_angle - self.steer_rate * self.damping # physics
        s_accel = (u_s + f_s) / self.inertia
        
        # constrain steering accel
        s_accel = min(self.max_steer_accel, max(-self.max_steer_accel, s_accel))
        
        # appply kinematics
        self.steer_angle += self.steer_rate * dt + s_accel * 0.5*dt**2
        self.steer_rate += s_accel * dt
        self.wheel_angle = self.steer_angle * self.steer_to_tire
        
        
        
    
    
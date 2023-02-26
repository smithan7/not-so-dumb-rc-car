import numpy as np
import math
from PID_Controller import PID_Controller
from steering_assembly import Steering_Assembly
from speed_assembly import Speed_Assembly

class Test_Car_2D:

    # x = x position, y position, heading, steering angle
    x = np.array([0.0, 0.0, 0.0, 0.0])
    # xd = x speed, y speed, heading rate, steering rate
    xd = np.array([0.0, 0.0, 0.0, 0.0])
    
    speed = 0.0
    
    wb_length = 1.0
    max_accel = 0.5 * 9.81 # 0.5g
    max_accel_force = 100.0 # N
    mass = 100.0 # kg
    aero_drag_coef = 0.01 # 0.5 * density of ait * area * drag coef
    roll_drag_coef = 0.01 # proportional to weight
    
    speed_pid = PID_Controller()
    speed_pid.k_p = 10.0
    speed_pid.k_i = 1.0
    speed_pid.k_d = 0.0
    speed_pid.max_error_sum = 10.0

    steering = Steering_Assembly()
    speed = Speed_Assembly()
    
    acceleration_time_constant = 0.3 # first order lag

    def __init__(self):
        pass
    
    def kinematic_update(self, u, dt):
        '''Takes in the current state and accelerations'''
        u_steer = u[0]
        u_accel = u[1]
        
        # print("u_accel: ", u_accel)
        # constrain acceleration
        u_accel = max(min(u_accel, self.max_accel), -self.max_accel)
        self.speed += u_accel
        # print("self.speed:", self.speed)

        # constrain steering angle rate
        u_steer = max(min(u_steer, self.max_steering_accel), -self.max_steering_accel)
        # print("u_steer: ", u_steer)
        u_steer += self.xd[3]

        # given control inputs, apply to get velocities / rates
        steer_A = np.array([math.cos(self.x[2]), # x speed
                            math.sin(self.x[2]), # y speed
                            math.tan(self.x[3])/self.wb_length, # heading rate
                            0.0]) # steering rate
        steer_B = np.array([0.0,
                            0.0,
                            0.0,
                            1.0])
        
        self.xd = steer_A * self.speed + steer_B * u_steer
        # print("steering rate = ", self.xd[3])
                
        # get positions
        self.x += self.xd * dt
        # constrain steering angle
        # print("steering angle = ", self.x[3])
        
        self.x[2] = self.x[2] % (2.0 * np.pi)
        self.x[3] = max(min(self.x[3], self.max_steering_angle), -self.max_steering_angle)

    def linear_speed_dynamics(self, u):
        '''Takes in a throttle and brake input and returns the resulting acceleration'''
        throttle = u[0]
        brake = u[1]
        
        #TODO - add in Brake and Throttle time lag at some point
        #TODO - it will take some time to change pedal positions
        
        f_u = throttle - brake
        # print("fu: ", f_u)
        f_d = self.aero_drag_coef * self.speed**2 + self.roll_drag_coef * self.speed
        # print("f_d: ", f_d)
        accel = (f_u - f_d) / self.mass
        # print("accel: ", accel)
        return accel
    
    def pid_speed_control(self, s_d):
        '''Uses PID controller to control the speed of the vehicle'''
        f_d = self.speed_pid.control(s_d, self.speed)        
        return f_d
    
    def get_controls_from_accel(self, u_accel):
        '''Convert desried acceleration to control inputs'''
        if u_accel > 0:
            
            return [min(self.max_accel_force, u_accel), 0.0]
        else:
            return [0.0, min(self.max_accel_force, abs(u_accel))]
        
    def pid_steering_control(self, s_d):
        '''Uses PID controller to control the angle of the steering wheel'''
        f_d = self.steering_pid.control(s_d, self.x[3])
        return f_d
    
    def simple_steering_dynamics(self, s_d):
        '''We should treat this as a servo/stepper motor - it needs to accelerate, has a max speed, etc, but always gets to the right spot'''
        
        
        
        # ARCHIVE
        '''This assumes that we can get to the right angle but it takes time to get there'''
        # takes in desired steering angle and uses it to provide steering angle speed which is then applied in kinematic model
        # model is taken from 'real-time Motion planning with applications to autonomous urban driving
        # u_steer = (s_d - self.x[3]) / self.steering_time_constant #
        # return u_steer
        
        
        
        
        
    def simple_steering_control_law(self, cross_track_error):
        steering_desired = self.heading + math.atan(k * cross_track_error / self.speed)


    def dynamic_control_law(self):
        pass                
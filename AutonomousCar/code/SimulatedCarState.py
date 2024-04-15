import numpy as np
import math

def clamp(x, x_min, x_max):
    '''This ensures  x_min <= x <= x_max'''
    return min(x_max, max(x_min, x))

class SimulatedCarState:
    x = np.array([0, #0 - x position
                  0, #1 - x speed
                  0, #2 - y position
                  0, #3 - y speed 
                  0, #4 - forward speed
                  0, #5 - forward acceleration
                  0, #6 - forward jerk
                  0, #7 - heading angle
                  0, #8 - heading rate
                  0, #9 - steering angle
                  0]) #10 - steering rate
    
    xi = 0
    xdi = 1
    yi = 2
    ydi = 3
    fsi = 4
    fai = 5
    fji = 6
    hai = 7
    hri = 8
    sai = 9
    sri = 10
    
    def __init__(self, x, car_params):
        self.x = x
        self.carParams = car_params
        
    
    def input_steering(self, u_steer):
        '''Take in a desired steering angle and get the resulting steering rate'''
        # ensure the desired steering angle is possible 
        u_steer = clamp(u_steer, -self.carParams.max_steering_angle, self.carParams.max_steering_angle)
        # set the rate - will push to actual steering angle in update function
        self.x[self.sri] = self.carParams.steer_damping * (u_steer - self.x[self.sai])
        # ensure the resulting steering rate is possible
        self.x[self.sri] = clamp(self.x[self.sri], -self.carParams.max_steering_rate, self.carParams.max_steering_rate)
        
    def input_accel(self, u_accel):
        '''Take in a desired acceleration and get the resulting acceleration rate (jerk)'''
        # ensure the desired acceleration is possible 
        u_accel = clamp(u_accel, self.carParams.max_braking_accel , self.carParams.max_forward_accel)
        # set the rate - will push to actual steering angle in update function
        self.x[self.fji] = self.carParams.steer_damping * (u_accel - self.x[self.fai])
        
    def tic(self, dt):
        #
        Gss = 1 / (1.0 + (self.x[self.fsi]/self.carParams.characteristic_velocity)**2)
        A = np.array([[1.0,  dt,   0,  0,                                                    0,   0,         0,   0,  0,   0,   0], # x_k+1 = x_k + xdot*dt
                      [  0,   0,   0,  0,                           math.cos(self.x[self.hai]),   0,         0,   0,  0,   0,   0], # xdot = forward speed * cos(heading)
                      [  0,   0, 1.0, dt,                                                    0,   0,         0,   0,  0,   0,   0], # y_k+1 = y_k + ydot * dt 
                      [  0,   0,   0,  0,                           math.sin(self.x[self.hai]),   0,         0,   0,  0,   0,   0], # ydot = forward speed * sin(heading)
                      [  0,   0,   0,  0,                                                  1.0,  dt, 0.5*dt**2,   0,  0,   0,   0], # forward speed_k+1 = forward speed_k + forward acceleration * dt + 0.5*forward jerk_k * dt^2 
                      [  0,   0,   0,  0,                                                    0, 1.0,        dt,   0,  0,   0,   0], # forward acceleration_k+1 = forward_acceleration_k + forward_jerk * dt
                      [  0,   0,   0,  0,                                                    0,   0,       1.0,   0,  0,   0,   0], # forward jerk_k+1 = forward_jerk_k
                      [  0,   0,   0,  0,                                                    0,   0,         0, 1.0, dt,   0,   0], # heading angle_k+1 = heading_angle_k + heading_angle rate_k * dt
                      [  0,   0,   0,  0, math.tan(self.x[self.sai])*Gss/self.carParams.length,   0,         0,   0,  0,   0,   0], # heading angle rate_k+1= forward_speed_k * 1/L*tan(steering_angle)*Gss
                      [  0,   0,   0,  0,                                                    0,   0,         0,   0,  0, 1.0,  dt], # steering angle_k+1 = steering angle_k + steering angle_rate_k * dt
                      [  0,   0,   0,  0,                                                    0,   0,         0,   0,  0,   0, 1.0]]) # steering angle rate_k+1 = steering angle rate_k
        
        self.x = A @ self.x
        
    def get_steering_angle(self):
        return self.x[self.sai]
    
    def get_steering_rate(self):
        return self.x[self.sri]
    
    def get_heading_angle(self):
        return self.x[self.hai]
    
    def get_heading_rate(self):
        return self.x[self.hri]
    
    def get_speed(self):
        return self.x[self.fsi]
    
    def get_2D_pos(self):
        return [self.x[self.xi], self.x[self.yi]]
    
    
    
    
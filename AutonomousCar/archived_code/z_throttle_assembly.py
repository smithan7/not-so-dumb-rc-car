
class Throttle_Assembly:
    
    engine_speed = 0.0
    max_engine_speed = 5000.0
    force = 0.0
    idle_speed = 1000.0
    
    throttle = 0.0
    max_throttle = 1.0
    min_throttle = 0.0
    throttle_speed = 0.1
    
    def __init__(self) -> None:
        pass
    
    
    def get_throttle_response(self, t_d, dt):
        '''Account for time delay in throttle response'''
        t_e = t_d - self.throttle
        t_step = self.throttle_speed * dt
        if abs(t_e) < t_step:
            self.throttle = t_d
        elif t_e > 0.0:
            self.throttle += t_step
        else:
            self.throttle -= t_step
            
        self.throttle = min(self.max_throttle, max(self.min_throttle, self.trhottle))
            
    def get_engine_response_curve(self):
        '''The engines 'no-load' desired speed at a given throttle'''
        # 2 pices - below/above idle
        
        pass
    
    def get_engine_force(self):
        '''Given the current engine speed and throttle - get the resulting force applied to the motor'''

        # throttle is greater than amount required to maintain engine RPM - positive force
        
        # throttle is less than amount required to maintain engine RPM - negative force (engine braking)
        
        pass
    
    def apply_wheel_force(self, wf):
        # the car is moving at a given speed and this is applying a force to the engine
        # assuming there is sufficient traction, etc.
        # this effects engine braking and acceleration -> RPM
        pass
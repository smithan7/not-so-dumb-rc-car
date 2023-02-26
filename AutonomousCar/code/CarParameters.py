
g = 9.81
mph_to_mps = 0.44704
in_to_m = 0.0254
lbs_to_kg = 0.45359237

class CarParam:
    max_steering_angle = 0.5435 # radians
    max_steering_rate = 0.3294 # radians / second
    max_speed = 140 * mph_to_mps # m/s - est limitless = 140 mph
    characteristic_velocity = 20.0 # m/s - drives side slip
    steer_damping = 1.0 / 0.3 # seconds
    accel_damping = 1.0 / 0.3 # seconds
    max_braking_accel = -1.0*g # m/s^2 - stanley is -6.0 m/s^2, est limitless = -1g
    max_forward_accel = 3.3 # m/s^2 - stanley is 1.8 m/s^2, est limitless = 3.3 m/s^2
    max_centripetal_accel = 1.0*g # m/s^2 - est limitless is 1.0g 
    length = 28.0 * in_to_m # stanley is 2.885 meters, est limitless = 28"
    
    mass = 17.0 * lbs_to_kg # kg
    c_y = 145.0 # lateral tire stiffness
    a = length / 2.0 # center of mass to front wheels
    b = length / 2.0 # center of mass to rear wheels
    
    def __init__(self):
        pass
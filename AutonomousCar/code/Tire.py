import math
import numpy as np
import matplotlib.pyplot as plt

class Tire:
    #GRP Tires GT - TO3 Revo Belted Pre-Mounted 1/8 Buggy Tires (Black) (2) (XB1) w/FLEX Wheel
    # want to try cheaper foams: CONJB35B Contact GT8/Rally 1/8 On Road Foam GT Tires 35 Shore Black rims GRP
    diam = 0.099 # m
    inner_diam = 0.077 # m
    width = 0.042 # m
    temp = 21.12 # deg C
    specific_heat_coeff = 1.88 # # J/(g * deg C) - specific heat capacity of rubber
    max_temp = 54.4 #TODO - I think it  is around 130F
    tire_hysteresis_coeff = 0.02 # TODO - did some mild tuning
    
    contact_patch = 0.0
    radius = 0.0
    tread_depth = 0.0
    mass = 0.0
    specific_heat_mass = 0.0
    surface_area = 0.0
    
    
    def calc_values(self):
        density = 1100 * 1000 # kg / m^3 is an approximate for rubber
        volume = (math.pi*(self.diam/2.0)**2 - math.pi*(self.inner_diam/2)**2 ) * self.width
        self.mass = 0.5 * volume * density # 0.5 because there is foam inside the tire too
        self.surface_area = math.pi*self.diam*self.width # ignore ends - they're small!
        self.radius = self.diam / 2.0
        self.specific_heat_mass = self.specific_heat_coeff * self.mass
        self.tread_depth = (self.diam - self.inner_diam) / 2.0
        self.contact_patch = self.width * 0.1 * self.tread_depth # TODO - this is WAG
        
    def print(self):
        print("TireParams: Diameter: ", self.diam, " m")
        print("TireParams: Radius: ", self.radius, " m")
        print("TireParams: Radius: ", self.radius, " m")
        print("TireParams: Tread Depth: ", self.tread_depth, " m")
        print("TireParams: Length: ", self.width, " m")
        print("TireParams: Surface Area: ", self.surface_area, " m^2")
        print("TireParams: Contact Patch: ", self.contact_patch, " m^2")
        print("TireParams: Temp: ", self.temp, " C")
        print("TireParams: Max Temp: ", self.max_temp, " C")
        print("TireParams: Mass: ", self.mass, " g")
        print("TireParams: Specific Heat Coeff: ", self.specific_heat_coeff, " J/(g * deg C)")
        print("TireParams: Specific Heat Mass: ", self.specific_heat_mass, " J/(deg C)")
    
    def calc_heat_generated(self, forces, speed):
        '''Estimate heat generated over this time step'''
        # heat is the result of the forces on the tire to the ground * speed
        heat_generated = np.sum(forces) * self.tire_hysteresis_coeff * speed
        return heat_generated
    
    def calc_temp_change(self, power_in, T_prior, dt):
        #Q = c * m * (T2 - T1)
        Q = power_in * dt # heat input J - power in (w=J/s) * dt (s)
        T1 = T_prior # deg C
        T2 = Q / self.specific_heat_mass + T1 # deg C
        return T2
                
    def update_heat_model(self, forces, car_speed, air_speed, air_temp, ground_temp, dt):
        '''Estimate current temperature based upon incoming heat and expected dissipation'''
        # calc the heat generated over the last tic
        heat_gen = self.calc_heat_generated(forces, car_speed)
        # calc the heat expelled due to convection on the Tire
        heat_exp = self.calc_heat_expelled(self.temp, air_temp, ground_temp)
        # get how the temp changes
        self.temp = self.calc_temp_change(heat_gen - heat_exp, self.temp, dt)
        
    def calc_heat_expelled(self, external_temp, air_temp, ground_temp):
        # convection equation
        #q = h_c * A * dT =  # heat transfered per unit time - watts
        A = self.surface_area # area of heat transfer surface - m^2
        h_c = 200.0 # convective heat transfer coefficient of the process - W / (m^2 C) - 200 is an ~upper limit for a cylinder
        dT = external_temp - air_temp # temperature difference between the surface and the fluid - C
        Q_conv = h_c * A * dT # rate of heat expulsion - W - J/s
        # now for tire in contact with the road
        A = self.contact_patch #  # area of heat transfer surface - m^2
        h_c = 1.0 # TODO - this is an esimtate
        dT = external_temp - ground_temp
        Q_cond = h_c * A * dT # rate of heat expulsion - W - J/s
        
        return Q_conv + Q_cond# * dt # heat expelled over the last period
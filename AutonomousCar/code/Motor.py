mm_to_in = 0.03937008
in_to_mm = 1.0 / mm_to_in
mph_to_mps = 0.44704
in_to_m = 0.0254
ft_lbs_to_Nm = 1.0 / 1.355818
oz_in_to_ft_lbs = 1.0 / 192.00000012288
oz_in_to_Nm = 1.0 / 141.61193227806

import math

class Motor:
    
    name = ''
    kv = 0.0
    max_battery_cells = 0.0
    kt = 0.0
    resistance = 0.0 # 0.0856092660801802 # pulled from Raz Shifrin's speed secrets via screen capture
    diam = 0.0
    radius = 0.0
    length = 0.0
    length_scalar_for_fins = 2.0
    max_temp = 0.0 # 82.222 # deg C = 180 F
    surface_area = 0.0
    max_voltage = 0.0
    max_specd_rpm = 0.0
    max_rpm = 0.0
    specific_heat_coeff = 0.0
    specific_heat_mass = 0.0
    convective_heat_coeff = 0.0
    mass = 0.0
    heat_generated = 0.0
    heat_expelled = 0.0
    current = 0.0
    voltage = 0.0
    torque_Nm = 0.0
    rpm = 0.0
    
    def read_params(self, motorParams):
        self.temp = 21.12 #TODO - set from env
        self.name = motorParams.name
        self.kv = motorParams.kv
        self.max_battery_cells = motorParams.max_battery_cells
        self.kt = motorParams.kt
        self.resistance = motorParams.resistance # 0.0856092660801802 # pulled from Raz Shifrin's speed secrets via screen capture
        self.diam = motorParams.diam
        self.length = motorParams.length
        self.length_scalar_for_fins = motorParams.length_scalar_for_fins
        self.max_temp = motorParams.max_temp # 82.222 # deg C = 180 F
        self.max_voltage = motorParams.max_voltage
        self.max_specd_rpm = motorParams.max_specd_rpm
        self.specific_heat_coeff = motorParams.specific_heat_coeff
        self.convective_heat_coeff = motorParams.convective_heat_coeff
        self.mass = motorParams.mass
        
        
    def calc_vals(self):
        self.surface_area = 2.0 * math.pi*(self.diam/2.0)**2 + math.pi*self.diam*self.length*self.length_scalar_for_fins
        self.max_voltage = self.max_battery_cells * 3.7
        self.max_rpm = self.max_voltage * self.kv
        self.kt = 1352.0 / self.kv
        self.radius = self.diam / 2.0
        self.specific_heat_mass = self.specific_heat_coeff * self.mass

    def print(self):
        print("Motor: Name: ", self.name)
        print("Motor: KV: ", self.kv)
        print("Motor: Max Battery Cells: ", self.max_battery_cells)
        print("Motor: Max Voltage: ", self.max_voltage, " V")
        print("Motor: Max spec'd RPM: ", self.max_specd_rpm)
        print("Motor: KT: ", self.kt)
        print("Motor: Resistance: ", self.resistance, " Ohms")
        print("Motor: Diameter: ", self.diam, " m")
        print("Motor: Length: ", self.length, " m")
        print("Motor: Length Scalar for Fins: ", self.length_scalar_for_fins)
        print("Motor: Surface Area: ", self.surface_area, " m^2")
        print("Motor: Max Temp: ", self.max_temp, " C")
        print("Motor: Mass: ", self.mass, " g")
        print("Motor: Specific Heat Coeff: ", self.specific_heat_coeff, " J/(g * deg C)")
        print("Motor: Convective Heat Coeff: ", self.convective_heat_coeff, " w/(m^2 deg C)")
        print("Motor: Current: ", self.current, " A")
        print("Motor: Voltage: ", self.voltage, " g")
        print("Motor: Torque: ", self.torque_Nm, " Nm")
        print("Motor: Speed: ", self.rpm, " RPM")
        print("Motor: Temperature: ", self.temp, " C") 
        
    def calc_heat_generated(self, current):
        '''Estimate heat generated over this time step'''
        self.heat_generated = self.resistance * current**2
        return self.heat_generated
    
    def calc_motor_temp_change(self, power_in, dt):
        #Q = c * m * (T2 - T1) # heat input J - power in (w=J/s) * dt (s)
        self.temp = (power_in * dt) / self.specific_heat_mass + self.temp # deg C
        return self.temp
           
    def update_heat_model(self, current, air_speed, air_temp, dt):
        '''Estimate current temperature based upon incoming heat and expected dissipation'''
        # calc the heat generated over the last tic
        p_gen = self.calc_heat_generated(current)
        # calc the heat expelled due to convection on the motor
        p_expelled = self.calc_heat_expelled(air_speed, air_temp)
        # update moor temp
        self.temp = self.calc_motor_temp_change(p_gen - p_expelled, dt)
        
    def calc_heat_expelled(self, air_speed, air_temp):
        # convection equation
        #q = h_c * A * dT =  # heat transfered per unit time - watts
        self.heat_expelled = self.convective_heat_coeff * self.surface_area * (self.temp - air_temp) # rate of heat expulsion - W - J/s
        return self.heat_expelled # power expelled as heat
        
    def calc_motor_torque(self, current, rpm):
        '''Estimate total torque based on current'''
        # torque decreases linearly based on RPM
        self.current = current
        self.torque_Nm = self.kt * current* oz_in_to_Nm * (1 - rpm / self.max_rpm)
        return self.torque_Nm
    
    def calc_current_from_torque(self, torque_Nm, rpm):
        '''Estimate current used based on torque requested'''
        self.torque_Nm = torque_Nm
        self.current = torque_Nm / (self.kt * oz_in_to_Nm * (1 - rpm / self.max_rpm))
        return self.current
    
    def calc_motor_rpm_from_voltage(self, voltage):
        self.voltage = voltage
        self.rpm = voltage * self.kv  
        return self.rpm
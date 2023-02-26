import math

class MotorParams:
    # castle 171 pulled from Raz Shifrin's Speed Secrets
    name = ''
    kv = 0.0
    max_battery_cells = 0.0
    kt = 0.0
    resistance = 0.0 # 0.0856092660801802 # pulled from Raz Shifrin's speed secrets via screen capture
    diam = 0.0
    length = 0.0
    length_scalar_for_fins = 2.0
    max_temp = 0.0 # 82.222 # deg C = 180 F
    max_voltage = 0.0
    max_specd_rpm = 0.0
    specific_heat_coeff = 0.0
    convective_heat_coeff = 0.0
    mass = 0.0
    
    def print(self):
        print("MotorParams: Name: ", self.name)
        print("MotorParams: KV: ", self.kv)
        print("MotorParams: Max Battery Cells: ", self.max_battery_cells)
        print("MotorParams: Max Voltage: ", self.max_voltage, " V")
        print("MotorParams: Max spec'd RPM: ", self.max_specd_rpm)
        print("MotorParams: KT: ", self.kt)
        print("MotorParams: Resistance: ", self.resistance, " Ohms")
        print("MotorParams: Diameter: ", self.diam, " m")
        print("MotorParams: Length: ", self.length, " m")
        print("MotorParams: Length Scalar for Fins: ", self.length_scalar_for_fins)
        print("MotorParams: Max Temp: ", self.max_temp, " C")
        print("MotorParams: Mass: ", self.mass, " g")
        print("MotorParams: Specific Heat Coeff: ", self.specific_heat_coeff, " J/(g * deg C)")
        print("MotorParams: Convective Heat Coeff: ", self.convective_heat_coeff, " w/(m^2 deg C)")
        
        

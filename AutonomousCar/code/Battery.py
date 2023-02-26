class Battery:
    capacity = 0.0
    max_capacity = 0.0
    number_of_cells = 0.0
    voltage = 0.0
    max_voltage = 0.0
    min_voltage = 0.0
    c_rating = 0.0
    percent_capacity = 100.0
    
    def print(self):
        print("Battery: Capacity: ", self.capacity)
        print("Battery: Max Capacity: ", self.max_capacity)
        print("Battery: Voltage: ", self.voltage)
        print("Battery: Max Voltage: ", self.max_voltage)
        print("Battery: Min Voltage: ", self.min_voltage)
        print("Battery: Number of Cells: ", self.number_of_cells)
        print("Battery: C Rating: ", self.c_rating)
    
    def read_params(self, batteryParams):
        self.c_rating = batteryParams.c_rating
        self.max_capacity = batteryParams.max_capacity
        self.min_voltage = batteryParams.min_voltage
        self.max_voltage = batteryParams.max_voltage
        self.number_of_cells = batteryParams.number_of_cells
        
     
    def calc_vals(self):
        self.capacity = self.max_capacity
        self.percent_capacity = 100.0
        self.voltage = self.max_voltage
        self.max_voltage = self.number_of_cells * 4.1
        self.min_voltage = self.number_of_cells * 3.5
    
    def draw_current(self, c_out, dt):
        # reduce the battery capacity by pushing out c_out
        self.capacity = self.capacity - c_out * dt / 3600.0
        self.percent_capacity = 100.0 * self.capacity / self.max_capacity
        # assume battery voltage drops linearly with current
        self.voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * (self.capacity / self.max_capacity)
        
    def ask_for_current(self, c_ask, dt):
        # controller asks for c_ask current, the battery gives c_get
        # ensures we're only getting a realistic amount
        pass
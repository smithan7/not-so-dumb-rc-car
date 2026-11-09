class Battery:
    capacity = 0.0
    max_capacity = 0.0
    number_of_cells = 0.0
    voltage = 0.0
    cell_max_voltage = 4.2
    cell_min_voltage = 3.5
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
        self.cell_min_voltage = batteryParams.cell_min_voltage
        self.cell_max_voltage = batteryParams.cell_max_voltage
        self.number_of_cells = batteryParams.number_of_cells
        
     
    def calc_vals(self):
        # pack limits first, then start fully charged
        self.max_voltage = self.number_of_cells * self.cell_max_voltage
        self.min_voltage = self.number_of_cells * self.cell_min_voltage
        self.capacity = self.max_capacity
        self.percent_capacity = 100.0
        self.voltage = self.max_voltage
    
    def draw_current(self, c_out, dt):
        # reduce the battery capacity by pushing out c_out
        self.capacity = self.capacity - c_out * dt / 3600.0
        self.percent_capacity = 100.0 * self.capacity / self.max_capacity
        # assume battery voltage drops linearly with remaining capacity
        self.voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * (self.capacity / self.max_capacity)
        
    def ask_for_current(self, c_ask, dt):
        # controller asks for c_ask current, the battery gives c_get
        # ensures we're only getting a realistic amount
        pass
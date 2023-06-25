import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import numpy as np


class GUI:
    def __init__(self):
        # create the main application window
        self.root = tk.Tk()
        self.root.title("Real-time Serial Plotter")

        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack()
        self.mid_frame = tk.Frame(self.root)
        self.mid_frame.pack(side=tk.TOP)
        self.btm_frame = tk.Frame(self.root)
        self.btm_frame.pack(side=tk.BOTTOM)


        self.t = 0.0
        self.dt = 0.01
        self.speed_pwm = 1500
        self.steer_pwm = 1500
        self.data = [[0], # time
                    [0], # steer_pwm
                    [0], # heading
                    [0], # speed pwm
                    [0], # speed
                    [0], # steer_cmd
                    [0]] # speed_cmd

        # create the serial connection
        # ser = serial.Serial('COM3', 9600)  # replace with your own port and baud rate

        # create a list of figure and axes objects to plot the data
        self.figures = []
        self.axes = []
        self.n_figures = 4
        for _ in range(self.n_figures):
            fig = Figure(figsize=(4, 2), dpi=100)
            ax = fig.add_subplot(1, 1, 1)
            ax.grid()
            self.figures.append(fig)
            self.axes.append(ax)

        # create a list of canvas objects for the plots and add them to the window
        self.canvases = []
        for fi, fig in enumerate(self.figures):
            if fi % 2 == 0:
                canvas = FigureCanvasTkAgg(fig, master=self.top_frame)
            else:
                canvas = FigureCanvasTkAgg(fig, master=self.mid_frame)
            canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
            self.canvases.append(canvas)


        self.entry_canvas = tk.Canvas(self.btm_frame, width=600, height=300)

        self.speed_label = tk.Label(self.btm_frame, text="Enter Speed Command (900-2100)")
        self.entry_canvas.create_window(150,60,window=self.speed_label)

        self.speed_entry = tk.Entry(self.root)
        self.entry_canvas.create_window(300,60, window=self.speed_entry)

        label_widget2 = tk.Label(self.btm_frame, text="Enter Steer Command (900-2100)")
        self.entry_canvas.create_window(150,100,window=label_widget2)

        self.steer_entry = tk.Entry(self.btm_frame)
        self.entry_canvas.create_window(300,100, window=self.steer_entry)

        self.submit_button = tk.Button(text='Submit', command=self.submit_data)
        self.entry_canvas.create_window(225,150, window=self.submit_button)
        
        
        self.reset_button = tk.Button(text='Reset', command=self.reset_data)
        self.entry_canvas.create_window(225,200, window=self.reset_button)
        
        
        self.entry_canvas.pack(side=tk.LEFT, fill=tk.BOTH)
    
    def submit_data(self):
        try:
            speed_pwm_in = int(self.speed_entry.get())
            steer_pwm_in = int(self.steer_entry.get())
            
            if speed_pwm_in >= 900 and speed_pwm_in <= 2100:
                self.speed_pwm = speed_pwm_in
                
            if steer_pwm_in >= 900 and steer_pwm_in <= 2100:
                self.steer_pwm = steer_pwm_in
            
            
            print("speed_pwm: ", self.speed_pwm)
            print("steer_pwm: ", self.steer_pwm)

        
        except:
            tk.messagebox.showinfo('Error', "Your data was not formatted properly")  
        
        self.speed_entry.delete(0, tk.END)
        self.steer_entry.delete(0, tk.END)

    def reset_data(self):
        self.steer_pwm = 1500
        self.speed_pwm = 1500

    # create a function to read the serial data and update the plots
    def update_plots(self):
        try:
            # read the serial data
            # data = ser.readline().decode().strip()
            # split the data into separate values
            # values = data.split(',')
            
            # generate fake data to test
            for _ in range(5):
                values = np.random.randn(self.n_figures) * np.array([0, 1000, 0, 1000])
                # convert the values to numbers and add them to the plots
                self.t += self.dt/5.0
                self.data[0].append(self.t)
                self.data[1].append(values[0])
                self.data[2].append(values[1])
                self.data[3].append(values[2])
                self.data[4].append(values[3])
                self.data[5].append(self.speed_pwm)
                self.data[6].append(self.steer_pwm)
                
            for d in self.data:
                while len(d) > 100:
                    d.pop(0)
            
            self.axes[0].clear()
            self.axes[0].plot(self.data[0], self.data[1], label="Received")
            self.axes[0].plot(self.data[0], self.data[5], label="Set")
            self.axes[0].grid()
            self.axes[0].set_ylabel("Speed PWM")
            self.axes[0].set_title("Speed Commands vs Time")
            self.axes[0].legend(loc='upper left')
            self.axes[0].set_ylim([900, 2100])
            
            
            self.axes[1].clear()
            self.axes[1].plot(self.data[0], self.data[2], label="Speed")
            self.axes[1].grid()
            self.axes[1].set_ylabel("Speed (m/s)")
            self.axes[1].set_title("Vehicle Speed vs Time")
            # axes[1].legend()
            self.axes[1].set_ylim([-5, 15])
            
            
            self.axes[2].clear()
            self.axes[2].plot(self.data[0], self.data[3], label="Received")
            self.axes[2].plot(self.data[0], self.data[6], label="Set")
            self.axes[2].grid()
            self.axes[2].set_ylabel("Steering PWM")
            self.axes[2].set_title("Steering Commands vs Time")
            self.axes[2].legend(loc='upper left')
            self.axes[2].set_ylim([900, 2100])
            
            
            self.axes[3].clear()
            self.axes[3].plot(self.data[0], self.data[4], label='Heading Rate')
            self.axes[3].grid()
            self.axes[3].set_ylabel("Heading rate (rads/s)")
            self.axes[3].set_title("Heading Rate vs Time")
            # axes[3].legend()
            self.axes[3].set_ylim([-5.0, 5.0])
            
            # update the plots on the canvases
            for canvas in self.canvases:
                canvas.draw()
        except:
            pass
        
        # schedule the function to run again after 100ms
        self.root.after(int(self.dt*1000), self.update_plots)
        
    def run(self):
        # start the function to update the plots
        self.update_plots()

        # start the main event loop
        self.root.mainloop()


if __name__ == "__main__":

    gui = GUI()
    gui.run()
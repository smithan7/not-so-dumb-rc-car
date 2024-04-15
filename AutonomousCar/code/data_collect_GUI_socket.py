import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import numpy as np
import socket
import time
import matplotlib.ticker as ticker

class GUI:
    
    time_i = 0
    
    cmd_header = 'C'
    # time_i = 0
    steve_latency_i = 1
    steve_status_i = 2
    steve_cntrl_i = 3
    steve_speed_i = 4
    steve_steer_i = 5
    xavier_latency_i = 6
    xavier_status_i = 7
    xavier_speed_i = 8
    xavier_steer_i = 9
    
    nav_header = 'N'
    # time_i = 0
    odom_x_i = 1
    odom_y_i = 2
    heading_i = 3
    speed_i = 4
    lat_i = 5
    lon_i = 6
    
    
    def __init__(self):
        
        UDP_IP = "127.0.0.1"
        UDP_PORT = 5005
        
        self.start_time = time.time()

        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.01)
        
        # create the main application window
        self.root = tk.Tk()
        self.root.title("Real-time Serial Plotter")

        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack()
        self.mid_frame = tk.Frame(self.root)
        self.mid_frame.pack(side=tk.TOP)
        self.btm_frame = tk.Frame(self.root)
        self.btm_frame.pack(side=tk.BOTTOM)

        self.speed_pwm = 1500
        self.steer_pwm = 1500
        self.cmd_data = []
        for _ in range(10):
            self.cmd_data.append([])
            
        self.nav_data = []
        for _ in range(7):
            self.nav_data.append([])
        
        # create a list of figure and axes objects to plot the data
        self.figures = []
        self.axes = []
        self.n_figures = 6
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


        self.entry_canvas = tk.Canvas(self.btm_frame, width=900, height=110)

        self.speed_label = tk.Label(self.btm_frame, text="Enter Speed Command (900-2100)")
        self.entry_canvas.create_window(90,30,window=self.speed_label)

        self.speed_entry = tk.Entry(self.root)
        self.entry_canvas.create_window(300,30, window=self.speed_entry)

        label_widget2 = tk.Label(self.btm_frame, text="Enter Steer Command (900-2100)")
        self.entry_canvas.create_window(90,60,window=label_widget2)

        self.steer_entry = tk.Entry(self.btm_frame)
        self.entry_canvas.create_window(300,60, window=self.steer_entry)

        self.submit_button = tk.Button(text='Submit', command=self.submit_data)
        self.entry_canvas.create_window(50,95, window=self.submit_button)
        
        self.reset_button = tk.Button(text='Reset', command=self.reset_data)
        self.entry_canvas.create_window(125,95, window=self.reset_button)
        
        self.steve_status_led = self.entry_canvas.create_oval(450, 10, 475, 35)  # Create a circle on the Canvas
        self.entry_canvas.itemconfig(self.steve_status_led, fill="green")  # Fill the circle with GREED
        
        self.steve_latency_label_var = tk.StringVar()  # Set the string variable for Label widget
        self.steve_latency_label_var.set("Steve Latency: -1.0")
        self.steve_latency_label = tk.Label(self.btm_frame, textvariable=self.steve_latency_label_var)
        self.entry_canvas.create_window(550,22,window=self.steve_latency_label)
        
        self.xavier_status_led = self.entry_canvas.create_oval(450, 50, 475, 75)  # Create a circle on the Canvas
        self.entry_canvas.itemconfig(self.xavier_status_led, fill="green")  # Fill the circle with GREED
        
        self.xavier_latency_label_var = tk.StringVar()  # Set the string variable for Label widget
        self.xavier_latency_label_var.set("Xavier Latency: -1.0")
        self.xavier_latency_label = tk.Label(self.btm_frame, textvariable=self.xavier_latency_label_var)
        self.entry_canvas.create_window(550,62,window=self.xavier_latency_label)
        
        self.cntrl_led = self.entry_canvas.create_oval(700, 10, 790, 100)  # Create a circle on the Canvas
        self.entry_canvas.itemconfig(self.cntrl_led, fill="green")  # Fill the circle with GREED
        
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
            flag = False
            while True:
                try:
                    msg, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
                except socket.timeout:
                    print("timeout exception")
                    break
                except socket.error:
                    print("socket error")
                    break
                else:
                    msg = msg.decode().split(",")
                    # print("msg: ", msg)
                    if msg[0] == 'C':
                        flag = True
                        self.cmd_data[self.time_i].append(float(msg[1]))
                        self.cmd_data[self.steve_latency_i].append(float(msg[2]))
                        self.cmd_data[self.steve_status_i].append(float(msg[3]))
                        self.cmd_data[self.steve_cntrl_i].append(float(msg[4]))
                        self.cmd_data[self.steve_speed_i].append(float(msg[5]))
                        self.cmd_data[self.steve_steer_i].append(float(msg[6]))
                        self.cmd_data[self.xavier_latency_i].append(float(msg[7]))
                        self.cmd_data[self.xavier_status_i].append(float(msg[8]))
                        self.cmd_data[self.xavier_speed_i].append(float(msg[9]))
                        self.cmd_data[self.xavier_steer_i].append(float(msg[10]))
                        
                        for d in self.cmd_data:
                            while len(d) > 50:
                                d.pop(0)
                        
                        self.steve_latency_label_var.set("Steve Latency: {:.1f}".format(np.mean(self.cmd_data[self.steve_latency_i])))
                        self.xavier_latency_label_var.set("Xavier Latency: {:.1f}".format(np.mean(self.cmd_data[self.xavier_latency_i])))
                        
                        if msg[4] > 1500:
                            self.entry_canvas.itemconfig(self.cntrl_led, fill="green")
                        else:
                            self.entry_canvas.itemconfig(self.cntrl_led, fill="red")
                        

                                
                    elif msg[0] == 'N':
                        flag = True
                        self.nav_data[self.time_i].append(float(msg[1]))
                        self.nav_data[self.odom_x_i].append(float(msg[2]))
                        self.nav_data[self.odom_y_i].append(float(msg[3]))
                        self.nav_data[self.heading_i].append(float(msg[4]))
                        self.nav_data[self.speed_i].append(float(msg[5]))
                        self.nav_data[self.lat_i].append(float(msg[6]))
                        self.nav_data[self.lon_i].append(float(msg[7]))
                        
                        for d in self.nav_data:
                            while len(d) > 50:
                                d.pop(0)
            
            if flag:
                self.axes[0].clear()
                self.axes[0].plot(self.cmd_data[self.time_i], self.cmd_data[self.steve_speed_i], label="Steve")
                self.axes[0].plot(self.cmd_data[self.time_i], self.cmd_data[self.xavier_speed_i], label="Xavier")
                self.axes[0].grid()
                self.axes[0].set_ylabel("Speed Cmd (PWM)")
                self.axes[0].set_title("Speed Cmd vs Time")
                self.axes[0].legend()
                self.axes[0].set_ylim([900, 2000])
                # self.axes[0].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[0].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                self.axes[1].clear()
                self.axes[1].plot(self.cmd_data[self.time_i], self.cmd_data[self.steve_steer_i], label="Steve")
                self.axes[1].plot(self.cmd_data[self.time_i], self.cmd_data[self.xavier_steer_i], label="Xavier")
                self.axes[1].grid()
                self.axes[1].set_ylabel("Steering PWM")
                self.axes[1].set_title("Steering Commands vs Time")
                self.axes[1].legend(loc='upper left')
                self.axes[1].set_ylim([900, 2100])
                # self.axes[1].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[1].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                self.axes[2].clear()
                self.axes[2].plot(self.nav_data[self.odom_x_i], self.nav_data[self.odom_y_i], label="Odom")
                self.axes[2].plot(self.nav_data[self.odom_x_i][-1], self.nav_data[self.odom_y_i][-1], 'ro', label="Pos")
                # self.axes[2].plot(self.nav_data[self.time_i], self.nav_data[self.xavier_latency_i], label="Xavier")
                self.axes[2].grid()
                self.axes[2].set_ylabel("Odom Y (m)")
                self.axes[2].set_ylabel("Odom X (m)")
                self.axes[2].set_title("Odom Position")
                # self.axes[2].legend(loc='upper left')
                # self.axes[2].set_ylim([-1, 200])
                # self.axes[2].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[2].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                self.axes[2].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                self.axes[3].clear()
                self.axes[3].plot(self.nav_data[self.time_i], self.nav_data[self.speed_i], label='Odom')
                # self.axes[3].plot(self.nav_data[self.time_i], self.nav_data[self.steve_status_i], label='Xavier')
                self.axes[3].grid()
                self.axes[3].set_ylabel("Speed (m/s)")
                self.axes[3].set_title("Speed vs Time")
                self.axes[3].legend()
                self.axes[3].set_ylim([-5.0, 15.0])
                # self.axes[3].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[3].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                self.axes[4].clear()
                self.axes[4].plot(self.nav_data[self.time_i], self.nav_data[self.heading_i], '.', label="Heading")
                # self.axes[4].plot(self.nav_data[self.time_i], self.nav_data[self.xavier_latency_i], label="Xavier")
                self.axes[4].grid()
                self.axes[4].set_ylabel("Heading (Deg)")
                self.axes[4].set_title("Heading vs Time")
                self.axes[4].legend(loc='upper left')
                self.axes[4].set_ylim([-5, 365])
                # self.axes[4].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[4].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                self.axes[5].clear()
                self.axes[5].plot(self.nav_data[self.time_i], self.nav_data[self.steve_status_i], label='Steve')
                self.axes[5].plot(self.nav_data[self.time_i], self.nav_data[self.steve_status_i], label='Xavier')
                self.axes[5].grid()
                self.axes[5].set_ylabel("Status (#)")
                self.axes[5].set_title("Status vs Time")
                self.axes[5].legend()
                self.axes[5].set_ylim([-5.0, 5.0])
                # self.axes[5].xaxis.set_major_locator(ticker.MultipleLocator(10))
                self.axes[5].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
                
                # update the plots on the canvases
                for canvas in self.canvases:
                    canvas.draw()
                    
                self.root.update()
                
        except:
            pass
        
        # schedule the function to run again after 100ms
        self.root.after(int(0.01 * 1000), self.update_plots)
        
    def run(self):
        # start the function to update the plots
        self.update_plots()

        # start the main event loop
        self.root.mainloop()


if __name__ == "__main__":

    gui = GUI()
    gui.run()
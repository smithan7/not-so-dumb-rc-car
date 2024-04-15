import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import socket
import time
import matplotlib.ticker as ticker
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

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

        self.speed_pwm = 1500
        self.steer_pwm = 1500
        self.cmd_data = []
        for _ in range(10):
            self.cmd_data.append([])
            
        self.nav_data = []
        for _ in range(7):
            self.nav_data.append([])
        
        self.fig = Figure(figsize=(10, 6), tight_layout=True)
        gs = GridSpec(2, 4, figure=self.fig)
            
        self.root = tk.Tk()
        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack()
        self.btm_frame = tk.Frame(self.root)
        self.btm_frame.pack(side=tk.BOTTOM)

        # label = Tk.Label(self.root,text="Playing with the gui plots").grid(column=0, row=0)

        plot_canvas = FigureCanvasTkAgg(self.fig, master=self.top_frame)
        plot_canvas.get_tk_widget().grid(column=0,row=1)

        self.n_figures = 5
        self.axs = []
        self.lines = []
        titles = ["Speed Cmd vs Time", "Steering Commands vs Time", "Speed vs Time", "Heading vs Time", "Odom Position"]
        ylabs = ["Speed (PWM)", "Steer (PWM)", "Speed (m/s)", "Heading (Degs)", "Position Y"]
        gss = [gs[0,0], gs[1,0], gs[0,1], gs[1,1], gs[0:,2:]]

        x = np.linspace(1, 1+2*np.pi, 100)        # x-array
            
        for i in range(self.n_figures):          
            ax = self.fig.add_subplot(gss[i])
            ax.set_title(titles[i])
            ax.set_ylabel(ylabs[i])
            ax.grid()
            
            # handle multiple lines
            if i==0:
                line, = ax.plot(x, 1500 + 100 * np.sin(x), label="S")
                self.lines.append(line)
                line2, = ax.plot(x, 1500 + 100 * np.cos(x), label="X")
                self.lines.append(line2)
                ax.legend(loc='upper left', prop={'size': 6})
                ax.set_ylim(900, 2000)
                ax.set_xlim(-6, 0.1)
                
            elif i==1:
                line, = ax.plot(x, 1500 + 100 * np.sin(x), label="S")
                self.lines.append(line)
                line2, = ax.plot(x, 1500 + 100 * np.cos(x), label="X")
                self.lines.append(line2)
                ax.legend(loc='upper left', prop={'size': 6})
                ax.set_ylim(900, 2000)
                ax.set_xlim(-6, 0.1)
                
            elif i==2:
                line, = ax.plot(x, np.sin(x))
                self.lines.append(line)
                ax.set_ylim(-5, 25)
                ax.set_xlim(-6, 0.1)
            elif i==3:
                line, = ax.plot(x, np.sin(x), '.')
                self.lines.append(line)
                ax.set_ylim(-5, 365)
                ax.set_xlim(-6, 0.1)
            elif i==4:
                ax.set_xlabel("Position X")
                ax.set_ylim(-20, 20)
                ax.set_xlim(-20, 20)
                line, = ax.plot(x, np.sin(x))
                self.lines.append(line)
                line, = ax.plot(x[0], np.sin(x[0]), 'o')
                self.lines.append(line)
                
            
                
            # handle xlables
            if i==1 or i==3:
                ax.set_xlabel("Time (s)")
            
            self.axs.append(ax)
        
        self.ani = animation.FuncAnimation(self.fig, self.animate, np.arange(1, 20000), interval=25, blit=True)
        
        
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
        self.steve_latency_label_var.set("Steve Latency:      ")
        self.steve_latency_label = tk.Label(self.btm_frame, textvariable=self.steve_latency_label_var)
        self.entry_canvas.create_window(550,22,window=self.steve_latency_label)
        
        self.xavier_status_led = self.entry_canvas.create_oval(450, 50, 475, 75)  # Create a circle on the Canvas
        self.entry_canvas.itemconfig(self.xavier_status_led, fill="green")  # Fill the circle with GREED
        
        self.xavier_latency_label_var = tk.StringVar()  # Set the string variable for Label widget
        self.xavier_latency_label_var.set("Xavier Latency:      ")
        self.xavier_latency_label = tk.Label(self.btm_frame, textvariable=self.xavier_latency_label_var)
        self.entry_canvas.create_window(550,62,window=self.xavier_latency_label)
        
        self.cntrl_led = self.entry_canvas.create_oval(700, 10, 790, 100)  # Create a circle on the Canvas
        self.entry_canvas.itemconfig(self.cntrl_led, fill="green")  # Fill the circle with GREED
        
        self.entry_canvas.pack(side=tk.LEFT, fill=tk.BOTH)
        self.update_plots()
        tk.mainloop()
        
    
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

    def animate(self, i):
        if len(self.cmd_data[0]) < 5:
            # print("N: ", len(self.cmd_data[0]))
            return self.lines
        
        # Speed PWM
        self.lines[0].set_xdata(self.cmd_data[self.time_i])
        self.lines[0].set_ydata(self.cmd_data[self.steve_speed_i])
        self.lines[1].set_xdata(self.cmd_data[self.time_i])
        self.lines[1].set_ydata(self.cmd_data[self.xavier_speed_i])
        self.axs[0].set_xlim(self.cmd_data[self.time_i][0], self.cmd_data[self.time_i][-1]+0.1)
        # print("self.cmd_data[self.time_i][0]- self.cmd_data[self.time_i][-1]+0.1: ", self.cmd_data[self.time_i][0] - self.cmd_data[self.time_i][-1]+0.1)
            
        # Steering PWM
        self.lines[2].set_xdata(self.cmd_data[self.time_i])
        self.lines[2].set_ydata(self.cmd_data[self.steve_steer_i])
        self.lines[3].set_xdata(self.cmd_data[self.time_i])
        self.lines[3].set_ydata(self.cmd_data[self.xavier_steer_i])
        self.axs[1].set_xlim(self.cmd_data[self.time_i][0], self.cmd_data[self.time_i][-1]+0.1)                
        
        # Speed
        self.lines[4].set_xdata(self.nav_data[self.time_i])
        self.lines[4].set_ydata(self.nav_data[self.speed_i])
        self.axs[2].set_xlim(self.nav_data[self.time_i][0], self.nav_data[self.time_i][-1]+0.2)
        # print("self.nav_data[self.speed_i]: ", self.nav_data[self.speed_i])
        # print("self.nav_data[self.time_i]: ", self.nav_data[self.time_i])
        # print("self.nav_data[self.time_i][0]- self.nav_data[self.time_i][-1]+0.1: ", self.nav_data[self.time_i][0] - self.nav_data[self.time_i][-1]+0.1)
        
        # Heading
        self.lines[5].set_xdata(self.nav_data[self.time_i])
        self.lines[5].set_ydata(self.nav_data[self.heading_i])
        self.axs[3].set_xlim(self.nav_data[self.time_i][0], self.nav_data[self.time_i][-1]+0.2)
        
        # Odom
        self.lines[6].set_xdata(self.nav_data[self.odom_x_i])
        self.lines[6].set_ydata(self.nav_data[self.odom_y_i])
        self.lines[7].set_xdata(self.nav_data[self.odom_x_i][-1])
        self.lines[7].set_ydata(self.nav_data[self.odom_y_i][-1])
        
        mx = 0.75 * self.nav_data[self.odom_x_i][-1] + 0.25 * self.nav_data[self.odom_x_i][0]
        my = 0.75 * self.nav_data[self.odom_y_i][-1] + 0.25 * self.nav_data[self.odom_y_i][0]
        
        
        self.axs[4].set_xlim(mx - 20.0, mx+20.0)
        self.axs[4].set_ylim(my - 20.0, my+20.0)
        
        
        # print("x: ", self.nav_data[self.odom_x_i])
        # print("y: ", self.nav_data[self.odom_y_i])
        
        return self.lines
        
    # create a function to read the serial data and update the plots
    def update_plots(self):
        try:
            flag = False
            while True:
                try:
                    msg, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
                except socket.timeout:
                    # print("timeout exception")
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
                            while len(d) > 100:
                                d.pop(0)
                        
                        self.steve_latency_label_var.set("Steve Latency: {:.1f} ms".format(np.mean(self.cmd_data[self.steve_latency_i])))
                        self.xavier_latency_label_var.set("Xavier Latency: {:.1f} ms".format(np.mean(self.cmd_data[self.xavier_latency_i])))
                        
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
                            while len(d) > 100:
                                d.pop(0)
                
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
import cv2
import numpy as np

class Realtime_Plotter:
    def __init__(self, plot_width, plot_height, min_val, max_val, label="Plot"):
        self.width = plot_width
        self.height = plot_height
        self.color = (255, 0 ,0)
        self.val = []
        self.plot = np.ones((self.height, self.width, 3))*255
        self.key = -1
        self.min_val = min_val
        self.max_val = max_val
        self.val_range = self.max_val - self.min_val
        self.label = label
        
    # Show plot using opencv imshow
    def build_plot(self, val):
        self.val.append(int(val))
        
        while len(self.val) > self.width:
            self.val.pop(0)
        
        self.plot = np.ones((self.height, self.width, 3))*255
        cv2.line(self.plot, (0, int(self.height/2)), (int(self.width), int(self.height/2)), (0,255,0), 3)
        for i in range(len(self.val)-1):
            h0 = int(self.height - self.height * (self.val[i] - self.min_val)/self.val_range)
            h1 = int(self.height - self.height * (self.val[i+1] - self.min_val)/self.val_range)
            cv2.line(self.plot, (i, h0), (i+1, h1), self.color, 3)

    def show_plot(self):
        cv2.imshow(self.label, self.plot)
        self.key = cv2.waitKey(10)
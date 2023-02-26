import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

class Track:
    def __init__(self, filename="../data/track_data.csv") :
        # pull in the track - default is laguna seca
        self.track_df = pd.read_csv(filename)
        
    def clamp(self, x, x_min, x_max):
        '''This ensures  x_min <= x <= x_max'''
        return min(x_max, max(x_min, x))

    def bearing_align(self, a,b):
        '''Aligns b with a'''
        if a - b > math.pi:
            b += 2.0*math.pi
        if b - a > math.pi:
            b -= 2.0*math.pi
        return b
    
    def interp_between_edges(track, i, t, col_name):
        
        if t < 0.5 and i>0:
            mt = t+0.5
            em1 = track.bearing_align(track.track_df.iloc[i][col_name], track.track_df.iloc[i-1][col_name])
            return mt*track.track_df.iloc[i][col_name] + (1-mt)*em1
        elif t > 0.5 and i < len(track.track_df)-1:
            mt = t-0.5
            ep1 = track.bearing_align(track.track_df.iloc[i][col_name], track.track_df.iloc[i+1][col_name])     
            return (1-mt)*track.track_df.iloc[i][col_name] + mt*ep1
        else:
            return track.track_df.iloc[i][col_name]
        
    def nearest_point_on_line_segment(self, A, B, C):
        """Calculate the distance of point C to line segment spanned by points A, B.

        """
        a = np.asarray(A)
        b = np.asarray(B)
        c = np.asarray(C)

        #project c onto line spanned by a,b but consider the end points
        #should the projection fall "outside" of the segment    
        n, v = b - a, c - a

        #the projection q of c onto the infinite line defined by points a,b
        #can be parametrized as q = a + t*(b - a). In terms of dot-products,
        #the coefficient t is (c - a).(b - a)/( (b-a).(b-a) ). If we want
        #to restrict the "projected" point to belong to the finite segment
        #connecting points a and b, it's sufficient to "clip" it into
        #interval [0,1] - 0 corresponds to a, 1 corresponds to b.

        t = max(0, min(np.dot(v, n)/np.dot(n, n), 1))
        if t == 0:
            return [a[0], a[1], np.linalg.norm(c - a), 0]
        if t == 1:
            return [b[0], b[1], np.linalg.norm(c - b), 1]
        p = a + t*n
        return [p[0], p[1], np.linalg.norm(c -p), t]
        #return np.linalg.norm(c - (a + t*n)) #or np.linalg.norm(v - t*n)
    
    def find_nearest_edge(self, p):
        dists = np.array(self.track_df.apply(lambda x: self.nearest_point_on_line_segment([x["x"], x["y"]], [x["xn"], x["yn"]], p), axis=1).to_list())
        # print("dists: ", dists)
        
        mindex = np.argmin(dists[:,2])
        mindist = dists[mindex, 2]
        mint = dists[mindex, 3]
        return mindex, mindist, mint
    
    def find_nearest_edge_with_seed(self, p, i, window_size=5):
        cnt = min(i+window_size, len(self.track_df))
        
        dists = np.array(self.track_df[i:cnt].apply(lambda x: self.nearest_point_on_line_segment([x["x"], x["y"]], [x["xn"], x["yn"]], p), axis=1).to_list())
        # print(dists)
        mindex = np.argmin(dists[:,2])
        
        # print(dists[mindex,:])
        mindist = dists[mindex, 2]
        mint = dists[mindex, 3]
        return mindex+i, mindist, mint
    
    def find_cross_track_error_to_segment(self, A, B, C):
        # Find if point C is left right of line AB
        a = np.asarray(A)
        b = np.asarray(B)
        c = np.asarray(C)
        # uses the determinant - # https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line#:~:text=1%20If%20point%27s%20x%20%3E%20line%27s%20x%2C%20the,line%27s%20x%2C%20the%20point%20is%20on%20the%20line.
        return np.sign(np.cross(b-a,c-a))
    
    def find_cross_track_error_to_edge(self, i, C):
        # Find if point C is left right of line AB
        a = np.asarray([self.track_df.iloc[i]['x'], self.track_df.iloc[i]['y']])
        b = np.asarray([self.track_df.iloc[i]['xn'] , self.track_df.iloc[i]['yn']])
        c = np.asarray(C)
        # uses the determinant - # https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line#:~:text=1%20If%20point%27s%20x%20%3E%20line%27s%20x%2C%20the,line%27s%20x%2C%20the%20point%20is%20on%20the%20line.
        return -np.sign(np.cross(b-a,c-a))
        
    def plot_track(self, edges=[], points=[], point_formats=[], point_labels=[], window_center=[], window_size=-1.0):
        '''Show the entire track and highlight edges as requested by index, and plot points upon request'''
        # plot all edges
        plt.plot(self.track_df["x"], self.track_df["y"], 'k--', label="Centerline")
        plt.plot(self.track_df["inner_x"], self.track_df["inner_y"], 'k')
        plt.plot(self.track_df["outer_x"], self.track_df["outer_y"], 'k')
        plt.plot([self.track_df["x"].tail(1), self.track_df["x"][0]], [self.track_df["y"].tail(1), self.track_df["y"][0]], 'k--')
        plt.plot([self.track_df["inner_x"].tail(1), self.track_df["inner_x"][0]], [self.track_df["inner_y"].tail(1), self.track_df["inner_y"][0]], 'k')
        plt.plot([self.track_df["outer_x"].tail(1), self.track_df["outer_x"][0]], [self.track_df["outer_y"].tail(1), self.track_df["outer_y"][0]], 'k')
               
        # highlight desired edges
        for e in edges:
            plt.plot([self.track_df["x"][e], self.track_df["xn"][e]], [self.track_df["y"][e], self.track_df["yn"][e]], 'r')
            
        # plot points with desired formats
        f = 'ro'
        l = ''
        for pi, p in enumerate(points):
            if len(point_formats) > pi:
                f = point_formats[pi]
            elif len(point_formats) == 1:
                f = point_formats[0]
            if len(point_labels) > pi:
                l = point_labels[pi]
            elif len(point_labels) == 1:
                l = point_labels[0]
            
            plt.plot(p[0], p[1], f, label=l)
            
        # zoom in window
        if window_size > 0 and len(window_center)==2:
            plt.axis([window_center[0]-window_size/2, window_center[0]+window_size/2, window_center[1]-window_size/2, window_center[1]+window_size/2])
        else:
            plt.axis('equal')
        
        # show the plot
        plt.rcParams['figure.figsize'] = [10, 7]
        plt.legend()
        plt.grid()
        plt.show()
    
    def computeRadiusOfCurvature(self, si):
        # this uses the midpoint of the current segment and the points on either side to estimate the radius of curvature at this point in the path
        z1 = 0 + 0j
        z2 = 0 + 0j
        z3 = 0 + 0j
        # verify I am a middle point
        if si > 0 and si < len(self.track_df)-1:
            # z1 = previous segment
            z1 = complex(self.track_df['x'][si-1], self.track_df['y'][si-1])
            # z2 = current segment
            z2 = complex(self.track_df['x'][si], self.track_df['y'][si])
            # z3 = next segment
            z3 = complex(self.track_df['x'][si+1], self.track_df['y'][si+1])
        
        # if I am the start point
        elif si == 0 and 2 < len(self.track_df):
            # z1 = current segment
            z1 = complex(self.track_df['x'][si], self.track_df['y'][si])
            # z2 = next segment
            z2 = complex(self.track_df['x'][si+1], self.track_df['y'][si+1])
            # z3 = following segment
            z3 = complex(self.track_df['x'][si+2], self.track_df['y'][si+2])
            
        # I am the end point
        elif si > 1 and si == len(self.track_df)-1:
            # z1 = current segment
            z1 = complex(self.track_df['x'][si], self.track_df['y'][si])
            # z2 = previous segment
            z2 = complex(self.track_df['x'][si-1], self.track_df['y'][si-1])
            # z3 = previous segment
            z3 = complex(self.track_df['x'][si-2], self.track_df['y'][si-2])
        else:
            # there is something wrong with the path
            raise ValueError("Path is not of correct type")
        
        # https://math.stackexchange.com/questions/213658/get-the-equation-of-a-circle-when-given-3-points#:~:text=Follow%20these%20steps%3A%201%20Consider%20the%20general%20equation,%E2%88%92%20yc%292%20%E2%88%92%20r2%20%3D%200%20More%20items
        if (z1 == z2) or (z2 == z3) or (z3 == z1):
            raise ValueError(f"Duplicate points: {z1}, {z2}, {z3}")
            
        w = (z3 - z1)/(z2 - z1)
        
        # You should change 0 to a small tolerance for floating point comparisons
        if abs(w.imag) <= 0.001:
            # raise ValueError(f"Points are collinear: {z1}, {z2}, {z3}")
            radius_of_curvature = 100000.0
            return radius_of_curvature
            
        # print("z1: ", z1)
        # print("z2: ", z2)
        # print("z3: ", z3)
        
        c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1;  # Simplified denominator
        radius_of_curvature = abs(z1 - c)
        # print("c: ", c)
        # print("r: ", radius_of_curvature)
        return radius_of_curvature
    
    def get_speed_limits_from_roc(self, carParams):
        rocs = []
        for i in range(len(self.track_df)):
            rocs.append(self.computeRadiusOfCurvature(i))

        self.track_df['roc'] = rocs
        self.track_df['speed limit'] = self.track_df.apply(lambda x: min(carParams.max_speed, math.sqrt(x['roc'] * carParams.max_centripetal_accel)), axis=1)
    
    def back_propagate_speed_limits_to_set_speed(self, carParam):
        # start at edge 0 and go to end - 1
        spds = self.track_df['speed limit'].to_list()
        flag = True
        cntr = 0
        while flag and cntr < 100:
            cntr += 1
            if cntr > 99:
                print("Track::back_propagate_speed_limits_to_set_speed: Failed to converge")
                return
            flag = False
            for i in range(len(spds) - 1):
                vi = spds[i+1]
                d = self.track_df['dist'][i]
                vf = math.sqrt(vi**2 - 2.0*carParam.max_braking_accel*d)
                if vf < spds[i]:
                    spds[i] = vf
                    flag = True
        self.track_df['speed'] = spds
        
    def computeCentripetalAcceleration(self, speed):
        return pow(speed,2) / self.radius_of_curvature
    
    def computeLinearAcceleration(self, s0, s1):
        return (s1**2 - s0**2) / (2.0 * self.length_m)
import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

class Track:
    def __init__(self, filename="../data/track_data.csv", closed=None):
        # pull in the track - default is laguna seca
        self.track_df = pd.read_csv(filename)
        if closed is None:
            # a closed circuit's last segment ends where the first one starts
            closed = bool(np.hypot(self._b[-1, 0] - self._a[0, 0], self._b[-1, 1] - self._a[0, 1]) < 1e-6)
        self.closed = closed

    @property
    def _a(self):
        return self.track_df[["x", "y"]].to_numpy(dtype=float)

    @property
    def _b(self):
        return self.track_df[["xn", "yn"]].to_numpy(dtype=float)
        
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
    
    def _nearest_on_segments(self, idx, p):
        """Vectorized nearest_point_on_line_segment over the segments in idx. Returns (dist, t) arrays."""
        a = self._a[idx]
        n = self._b[idx] - a
        v = np.asarray(p, dtype=float) - a
        nn = np.einsum("ij,ij->i", n, n)
        with np.errstate(invalid="ignore", divide="ignore"):
            t = np.where(nn > 0, np.einsum("ij,ij->i", v, n) / nn, 0.0)
        t = np.clip(t, 0.0, 1.0)
        d = np.linalg.norm(v - t[:, None] * n, axis=1)
        return d, t

    def find_nearest_edge(self, p):
        idx = np.arange(len(self._a))
        d, t = self._nearest_on_segments(idx, p)
        k = int(np.argmin(d))
        return k, d[k], t[k]

    def find_nearest_edge_with_seed(self, p, i, window_size=5):
        """Search window_size segments starting at i; wraps past the start/finish line on a closed track."""
        n = len(self._a)
        if self.closed:
            idx = (i + np.arange(min(window_size, n))) % n
        else:
            idx = np.arange(i, min(i + window_size, n))
        d, t = self._nearest_on_segments(idx, p)
        k = int(np.argmin(d))
        return int(idx[k]), d[k], t[k]
    
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
        n = len(self.track_df)
        if self.closed and n > 2:
            # on a circuit every point has neighbours on both sides
            z1 = complex(self.track_df['x'][(si-1) % n], self.track_df['y'][(si-1) % n])
            z2 = complex(self.track_df['x'][si], self.track_df['y'][si])
            z3 = complex(self.track_df['x'][(si+1) % n], self.track_df['y'][(si+1) % n])
        # verify I am a middle point
        elif si > 0 and si < len(self.track_df)-1:
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
        """Turn per-segment corner limits into a drivable speed profile.

        Backward pass: you must be able to brake (max_braking_accel) down to every later limit.
        Forward pass: you can only gain speed at max_forward_accel.
        On a closed track both passes wrap around the start/finish line until nothing changes.
        """
        spds = self.track_df['speed limit'].to_list()
        dists = self.track_df['dist'].to_list()
        n = len(spds)
        a_brake = abs(carParam.max_braking_accel)
        a_fwd = abs(carParam.max_forward_accel)
        # segment i runs from point i to point i+1; the last one only connects back on a closed track
        segs = range(n) if self.closed else range(n - 1)

        for _ in range(100):
            changed = False
            for i in reversed(segs):  # backward: braking
                j = (i + 1) % n
                v = math.sqrt(spds[j]**2 + 2.0*a_brake*dists[i])
                if v < spds[i] - 1e-9:
                    spds[i] = v
                    changed = True
            for i in segs:  # forward: acceleration
                j = (i + 1) % n
                v = math.sqrt(spds[i]**2 + 2.0*a_fwd*dists[i])
                if v < spds[j] - 1e-9:
                    spds[j] = v
                    changed = True
            if not changed:
                break
        else:
            print("Track::back_propagate_speed_limits_to_set_speed: Failed to converge")
            return
        self.track_df['speed'] = spds
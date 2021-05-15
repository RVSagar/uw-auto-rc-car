#!/usr/bin/python
import rospkg
import numpy as np
import math
#from scipy.interpolate import interp1d

from world_building import WorldBuilder



if __name__ == "__main__":
    print("Running World Generator")
    name = 'concave_world.sdf'

    for blank in [True, False]:
        if blank:
            world_name = "blank_" + name
        else:
            world_name = name
        wb = WorldBuilder(world_name, scale=0.125)
        contents = wb.get_sources()

        xs = []
        ys = []

        R = 65

        A = R
        B = (1.0 + 0.03)*R

        def get_xy(th):
            # Ovals of Cassini
            # From Schaums 4th ed
            c = -math.pow(B,4) + math.pow(A,4)
            b = -2*A*A*math.cos(2.0*th)
            a = 1
            rr = -b + math.sqrt(b*b - 4*a*c)
            rr = rr / (2.0*a)
            r = math.sqrt(rr)

            x = math.cos(th)*r
            y = math.sin(th)*r
            return x, y
        
        for th in np.linspace(0, 6.29, 100):
            x, y = get_xy(th)
            xs.append(x)
            ys.append(y)
        
        road = []
        for x, y in zip(xs, ys):
            road.append((x, y, 0))
        wb.add(contents, wb.generate_road_contents(road))

        
        if not blank:
            outer_houses = [1, 2, 3, 3, 2, 3, 2, 1, 2, 1 ,2, 3, 3, 2, 2, 3]
            inner_houses = [2, 3, 2, 1, 1, 1, 2, 3, 1]

            def get_house_xy(th, pol):
                x1, y1 = get_xy(th)
                x2, y2 = get_xy(th+0.01)
                dx = x2 - x1
                dy = y2 - y1
                mag = math.sqrt(dx*dx + dy*dy)
                x = x1 + dy/mag*10*pol
                y = y1 - dx/mag*10*pol
                th = math.atan2(dy, dx)
                return x, y, th

            for i in range(0, len(outer_houses)):
                th = float(i) / len(outer_houses) * 6.28
                x, y, th = get_house_xy(th, 1)
                model = outer_houses[i]
                wb.add(contents, wb.generate_house_contents(model, x, y, th))

            for i in range(0, len(inner_houses)):
                th = float(i) / len(inner_houses) * 6.28
                x, y, th = get_house_xy(th, -1)
                model = inner_houses[i]
                wb.add(contents, wb.generate_house_contents(model, x, y, th))

        wb.write_dest(contents)


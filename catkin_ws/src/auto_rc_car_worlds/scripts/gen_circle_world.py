#!/usr/bin/python
import rospkg
import numpy as np
import math

from world_building import WorldBuilder



if __name__ == "__main__":
    print("Running World Generator")
    wb = WorldBuilder("circle_world.sdf", scale=0.125)
    contents = wb.get_sources()

    R = 65
    road = []
    for th in np.linspace(0, 6.29, 100):
        x = math.cos(th)*R
        y = math.sin(th)*R
        y = y + R # Places (0,0) on the road
        road.append((x, y, 0))
    wb.add(contents, wb.generate_road_contents(road))

    
    outer_houses = [1, 2, 3, 3, 2, 3, 2, 1, 2, 1 ,2, 3, 3, 2]
    inner_houses = [2, 3, 2, 1, 1, 1, 2, 3, 1]

    for i in range(0, len(outer_houses)):
        th = float(i) / len(outer_houses) * 6.28
        x = math.cos(th)*(R+10)
        y = math.sin(th)*(R+10) + R
        model = outer_houses[i]
        wb.add(contents, wb.generate_house_contents(model, x, y, th-1.5707))

    for i in range(0, len(inner_houses)):
        th = float(i) / len(inner_houses) * 6.28
        x = math.cos(th)*(R-10)
        y = math.sin(th)*(R-10) + R
        model = inner_houses[i]
        wb.add(contents, wb.generate_house_contents(model, x, y, th+1.5707))

    wb.write_dest(contents)


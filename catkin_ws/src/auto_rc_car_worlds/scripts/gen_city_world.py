#!/usr/bin/python
import rospkg
import numpy as np
import math

from world_building import WorldBuilder



if __name__ == "__main__":
    print("Running World Generator")
    name = 'small_city_world.sdf'

    for blank in [True, False]:
        if blank:
            world_name = "blank_" + name
        else:
            world_name = name

        wb = WorldBuilder(world_name, scale=0.125)
        contents = wb.get_sources()

        wb.add(contents, wb.generate_road_contents([(wb.brs, 0, 0),
                                                    (40-wb.brs, 0, 0)]))

        wb.add(contents, wb.generate_road_contents([(0, wb.brs, 0),
                                                    (0, 40-wb.brs, 0)]))

        wb.add(contents, wb.generate_road_contents([(40, wb.brs, 0),
                                                    (40, 40-wb.brs, 0)]))

        wb.add(contents, wb.generate_road_contents([(wb.brs, 40, 0),
                                                    (40-wb.brs,   40, 0)]))

        wb.add(contents, wb.generate_corner(0, 0, 0))
        wb.add(contents, wb.generate_corner(0, 40, 1.5707))
        wb.add(contents, wb.generate_3way(40, 0, 0))
        wb.add(contents, wb.generate_3way(40, 40, 3.1415))

        # Create Curved Road
        angles = np.linspace(0, 3.1415, 10)
        points = []
        x0 = 40 + wb.brs
        y0 = 20
        points.append((x0, y0-20, 0))
        for a in angles:
            x = x0 + 20*math.sin(a)+1
            y = y0 - 20*math.cos(a)
            points.append((x, y, 0))
        points.append((x0, y0+20, 0))
        wb.add(contents, wb.generate_road_contents(points))
        

        if not blank:
            wb.add(contents, wb.generate_house_contents(1, 10, 10, 0))
            wb.add(contents, wb.generate_house_contents(1, 15, 30, 3.1415))

            wb.add(contents, wb.generate_house_contents(2, 30, 25, 1.5707)) 

            wb.add(contents, wb.generate_house_contents(3, 50, 20, 1.5707))


        wb.write_dest(contents)


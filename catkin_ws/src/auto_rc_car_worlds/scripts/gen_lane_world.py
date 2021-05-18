#!/usr/bin/python
import rospkg
import numpy as np
import math

from world_building import WorldBuilder



if __name__ == "__main__":
    print("Running World Generator")
    name = 'lane_world.sdf'

    for blank in [True, False]:
        if blank:
            world_name = "blank_" + name
        else:
            world_name = name

        wb = WorldBuilder(world_name, scale=0.125)
        contents = wb.get_sources()

        L = 150

        wb.add(contents, wb.generate_road_contents([(-wb.brs, 0, 0),
                                                    (L, 0, 0)]))

        if not blank:
            left_houses = [1, 2, 3, 3, 2, 3, 2, 1, 2]
            right_houses = [2, 3, 2, 1, 1, 1, 2, 3]

            for i in range(0, len(left_houses)):
                x = float(i) / len(left_houses) * L
                model = left_houses[i]
                wb.add(contents, wb.generate_house_contents(model, x, 10, 0))

            for i in range(0, len(right_houses)):
                x = float(i) / len(right_houses) * L
                model = right_houses[i]
                wb.add(contents, wb.generate_house_contents(model, x,-10, 3.1415))

        wb.write_dest(contents)


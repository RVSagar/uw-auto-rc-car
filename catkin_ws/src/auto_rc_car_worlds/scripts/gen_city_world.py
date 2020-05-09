#!/usr/bin/python
import rospkg
import numpy as np
import math

class WorldBuilder:
    def __init__(self):

        self.world_scale = 0.5
        self.base_road_width = 7.34

        # Backfill Road Square
        #  - Start this far back so that there is a square sec of road at 0
        self.brs = 3.52

        self.cur_road_id = 0
        self.curr_home_id = 0

        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path("auto_rc_car_worlds")
        self.source_world = self.pkg_dir + "/worlds/gen_city_world_base.sdf"
        self.dest_world = self.pkg_dir + "/worlds/small_city_world.sdf"

    def get_file_lines(self, file):
        with open(file, 'r') as f:
            sources = f.readlines()
        return sources

    def get_sources(self):
        return self.get_file_lines(self.source_world)

    def write_dest(self, contents):
        with open(self.dest_world, 'w') as f:
            f.writelines(contents)

    def generate_road_contents(self, points):
        self.cur_road_id += 1
        contents = []
        contents.append("<road name='gen_road%d'>\n" % self.cur_road_id)
        contents.append("<width>%f</width>\n" % (self.base_road_width * self.world_scale))

        for pt in points:
            x = pt[0]
            y = pt[1]
            contents.append("<point>%f %f %f</point>\n" % (x*self.world_scale, y*self.world_scale, 0))
        contents.append("</road>\n")
        return contents

    def generate_house_contents(self, n, x, y, yaw):
        self.curr_home_id += 1
        filename = self.pkg_dir + "/worlds/gen_city_world_house_1.sdf"# + str(n) + ".sdf"
        contents = self.get_file_lines(filename)

        for i in range(0, len(contents)):
            contents[i] = contents[i].replace("house_1", "house_"+str(n))
            contents[i] = contents[i].replace("House_1", "House_"+str(n))

            if "<model" in contents[i]:
                contents[i] = "<model name='House %d_%d'>" % (n, self.curr_home_id)

            if "<scale>" in contents[i]:
                contents[i] = "<scale>%f %f %f</scale>" % (self.world_scale, self.world_scale, self.world_scale)

            if "<pose" in contents[i]:
                contents[-2] = "<pose frame=''>%f %f 0 0 -0 %f</pose>\n" % (x*self.world_scale, y*self.world_scale, yaw)

        return contents

    def add(self, contents, new):
        for n in new:
            contents.insert(-2, n)

    def run(self):
        contents = self.get_sources()

        self.add(contents, self.generate_road_contents([(-self.brs, 0, 0),
                                                        (40, 0, 0)]))

        self.add(contents, self.generate_road_contents([(0, -self.brs, 0),
                                                        (0, 40, 0)]))

        self.add(contents, self.generate_road_contents([(40, -self.brs, 0),
                                                        (40, 40+self.brs, 0)]))

        self.add(contents, self.generate_road_contents([(-self.brs, 40, 0),
                                                        (40+self.brs,   40, 0)]))

        # Create Curved Road
        angles = np.linspace(0, 3.1415, 10)
        points = []
        for a in angles:
            x = 40 + 20*math.sin(a)
            y = 20 - 20*math.cos(a)
            points.append((x, y, 0))
        self.add(contents, self.generate_road_contents(points))
        
        self.add(contents, self.generate_house_contents(1, 10, 10, 0))
        self.add(contents, self.generate_house_contents(1, 15, 30, 3.1415))

        self.add(contents, self.generate_house_contents(2, 30, 25, 1.5707)) 

        self.add(contents, self.generate_house_contents(3, 50, 20, 1.5707))                                              

        self.write_dest(contents)


if __name__ == "__main__":
    print("Running World Generator")
    wb = WorldBuilder()
    wb.run()


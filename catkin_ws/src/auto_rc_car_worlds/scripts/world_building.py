#!/usr/bin/python
import rospkg
import numpy as np
import math

class WorldBuilder:
    def __init__(self, outfile, scale=0.125):

        self.world_scale = 0.125
        self.base_road_width = 7.34

        # Backfill Road Square
        #  - Start this far back so that there is a square sec of road at 0
        self.brs = 3.52

        self.cur_road_id = 0
        self.curr_home_id = 0

        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path("auto_rc_car_worlds")
        self.source_world = self.pkg_dir + "/worlds/base_world.sdf"
        self.dest_world = self.pkg_dir + "/worlds/" + outfile #small_city_world.sdf"

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
        contents.append("<material>")
        contents.append("<script>")
        contents.append("<uri>package://auto_rc_car_worlds/media/materials/scripts/our_road.material</uri>")
        #contents.append("<uri>package://auto_rc_car_worlds/materials/textures/road2.png</uri>")
        contents.append("<name>OurRoad</name>")
        contents.append("</script>")
        contents.append("<shader type='normal_map_tangent_space'>")
        contents.append("<normal_map>road1.png</normal_map>")
        contents.append("</shader>")
        contents.append("</material>")
        contents.append("</road>\n")
        return contents

    def generate_house_contents(self, n, x, y, yaw):
        # n: 1, 2, or 3 (model of house)
        self.curr_home_id += 1
        filename = self.pkg_dir + "/worlds/base_house.sdf"# + str(n) + ".sdf"
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


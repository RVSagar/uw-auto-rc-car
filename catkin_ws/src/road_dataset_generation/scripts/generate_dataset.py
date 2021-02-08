#!/usr/bin/env python

import os
import cv2
import numpy as np
import math
import random
import yaml
import uuid
import rospkg

HEIGHT = 480
WIDTH = 640

'''
Coordinates
X+ is right
Y+ is forward
Z+ is up
'''

class RoadPoint:
    def __init__(self, x, y, z, yaw, width):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.width = width

    def nextPoint(self, dy, dTheta):
        dx = dy - math.sin(self.yaw)
        dy = dy + math.cos(self.yaw)
        return RoadPoint(self.x+dx,  self.y+dy,  self.z, self.yaw + dTheta, self.width)

    def getLeftCorner(self):
        return self.getCornerPoint(-1)

    def getRightCorner(self):
        return self.getCornerPoint(1)

    def getCornerPoint(self, direction):
        dx = direction * self.width/2 * math.cos(self.yaw)
        dy = direction * self.width/2 * math.sin(self.yaw)
        return np.array([[self.x+dx], [self.y+dy], [self.z]])


class GeneratorConfigSample:
    def __init__(self, settings):
        self.road_i = settings['road_image_files'].index(random.choice(settings['road_image_files']))
        self.road_file = settings['road_image_files'][self.road_i]
        self.road_image = settings['road_images'][self.road_i]

        self.background_i = settings['background_image_files'].index(random.choice(settings['background_image_files']))
        self.background_file = settings['background_image_files'][self.background_i]
        self.background_image = settings['background_images'][self.background_i]

        self.X0 = self.sample_uni(settings['X0'])
        self.Y0 = self.sample_uni(settings['Y0'])
        self.CAM_HEIGHT = self.sample_uni(settings['CAM_HEIGHT'])
        self.YAW0 = self.sample_uni(settings['YAW0'])
        self.ROAD_WIDTH = self.sample_uni(settings['ROAD_WIDTH'])

        self.straight = self.sample_uni([0.0, 1.0]) < settings['prob_straight']
        self.inv_rad = self.sample_uni(settings['inv_radius'])

        self.DY = self.sample_uni(settings['DY'])
        self.Segments = int(self.sample_uni(settings['Segments']))

        self.FOCAL_LENGTH = self.sample_uni(settings['CAM_FOCAL_LENGTH'])
        self.PIXEL_SIZE = self.sample_uni(settings['CAM_PIXEL_SIZE'])

        self.notes = settings['notes']

    def sample_uni(self, data):
        return random.uniform(data[0], data[1])

    def gen_info_dict(self):
        return {
            'road_file': self.road_file,
            'background_file': self.background_file,
            'X0': self.X0,
            'Y0': self.Y0,
            'Z0': self.CAM_HEIGHT,
            'YAW0': self.YAW0,
            'cam_height': self.CAM_HEIGHT,
            'road_width': self.ROAD_WIDTH,
            'straight': self.straight,
            'inv_rad': self.inv_rad,
            'dy': self.DY,
            'segments': self.Segments,
            'pixel_size': self.PIXEL_SIZE,
            'focal_length': self.FOCAL_LENGTH,
            'notes': self.notes
        }

class Generator:
    def __init__(self, settings, pkg_dir):
        self.settings = settings

        print("Creating Directory Structure")
        self.base_dir = pkg_dir
        
        self.out_dir = self.base_dir + "/output/" + self.settings['subdir'] + "/"
        self.out_data_dir = self.out_dir + "data/"
        self.out_info_dir = self.out_dir + "info/"
        self.out_media_roads = self.out_dir + "source_media/roads/"
        self.out_media_backgrounds = self.out_dir + "source_media/backgrounds/"

        print("Directory generation may produce error if dir exists, or for true errors")
        for directory in [self.out_dir, self.out_data_dir, self.out_info_dir, self.out_media_backgrounds, self.out_media_roads]: 
            try:
                os.makedirs(directory)
            except OSError as e:
                print(e)
                pass

        if settings['clear'] and raw_input('Confirm Delete Existing files in ' + settings['subdir'] +' (y/n): ') == 'y':
            print('Clearing Existing Files')
            def clear_dir(directory):
                for filename in os.listdir(directory):
                    os.remove(directory + filename)
            clear_dir(self.out_data_dir)
            clear_dir(self.out_info_dir)
            clear_dir(self.out_media_backgrounds)
            clear_dir(self.out_media_roads)
        else:
            print('Not Clearing')

        self.media_dir = self.base_dir + "/source_media/"
        self.roads_dir = self.media_dir + "roads/"
        self.backgrounds_dir = self.media_dir + "backgrounds/"

        self.settings['road_images'] = []
        for image_file in self.settings['road_image_files']:
            self.settings['road_images'].append(cv2.imread(self.roads_dir + image_file, cv2.IMREAD_COLOR))
        self.settings['background_images'] = []
        for image_file in self.settings['background_image_files']:
            self.settings['background_images'].append(cv2.imread(self.backgrounds_dir + image_file, cv2.IMREAD_COLOR))

    def run(self):
        print("Generating %d Samples" % self.settings['N'])
        for i in range(0, self.settings['N']):
            config = GeneratorConfigSample(self.settings)
            self.generate_image(config)

        print("Saving Source Media")
        for name, image in zip(self.settings['road_image_files'], self.settings['road_images']):
            cv2.imwrite(self.out_media_roads + name, image)

        for name, image in zip(self.settings['background_image_files'], self.settings['background_images']):
            cv2.imwrite(self.out_media_roads + name, image)

    def generate_image(self, config):
        segment = config.road_image
        seg_width, seg_height, _ = segment.shape
        road = config.background_image

        centerpoints = []
        left_corners = []
        right_corners = []

        centerpoints.append(RoadPoint(config.X0, config.Y0, -config.CAM_HEIGHT, -config.YAW0, config.ROAD_WIDTH))
        left_corners.append(centerpoints[0].getLeftCorner())
        right_corners.append(centerpoints[0].getRightCorner())

        dTheta = 0.0
        if not config.straight:
            dTheta = config.DY / (1.0 / config.inv_rad)
            dTheta = dTheta * random.uniform(1.0, -1.0)
        else:
            config.inv_rad = 0.0
        

        for i in range(0, config.Segments):
            pt = centerpoints[-1].nextPoint(config.DY, dTheta=dTheta)
            centerpoints.append(pt)
            left_corners.append(pt.getLeftCorner())
            right_corners.append(pt.getRightCorner())
        
        for i in range(1, len(centerpoints)):
            bl = left_corners[i-1]
            br = right_corners[i-1]

            ul = left_corners[i]
            ur = right_corners[i]

            source = np.float32([[0,0], [seg_width, 0], [0, seg_height], [seg_width, seg_height]])
            dest = np.float32([self.pointToPixels(ul, config),
                               self.pointToPixels(ur, config),
                               self.pointToPixels(bl, config),
                               self.pointToPixels(br, config)])

            transformation = cv2.getPerspectiveTransform(source, dest)
            new = cv2.warpPerspective(segment, transformation, (WIDTH,HEIGHT))

            road[np.where((new != [0,0,0]).all(axis=2))] = [0,0,0]
            road = cv2.add(road, new)

        sample_uuid = str(uuid.uuid4())
        cv2.imwrite(self.out_data_dir + '/' + sample_uuid + '.png', road)
        with open(self.out_info_dir + "/" + sample_uuid + ".yaml", 'w') as file:
            yaml.dump(config.gen_info_dict(), file)

    def pointToPixels(self, pos, config):
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture13.pdf
        '''World Coordinates
        X+ is left
        Y+ is up
        Z+ is out/forward
        '''
        X = -pos[0]
        Y = pos[2]
        Z = pos[1]
        f = 1

        '''Image Coordinates
        Same as before
        '''
        x = config.FOCAL_LENGTH * X/Z
        y = config.FOCAL_LENGTH * Y/Z

        '''Pixel Coordinates
        From top-left corner
        u+ is right
        v+ is down
        '''
        u = x/config.PIXEL_SIZE + WIDTH/2
        v = y/config.PIXEL_SIZE + HEIGHT/2

        v = HEIGHT - v

        return [u,v]

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path("road_dataset_generation")

    with open(package_dir + '/config/generation_config.yaml') as file:
        settings = yaml.load(file, Loader=yaml.SafeLoader)
    g = Generator(settings, package_dir)
    g.run()
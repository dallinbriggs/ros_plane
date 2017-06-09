#!/usr/bin/env python

import numpy as np
from math import *
import rospy
from PointInPolygon import *


class inBounds:

    def __init__(self):
        self.bound_file = "boundaries.txt"

        self.init_lat = 38.
        self.init_lon = -76.

        self.EARTH_RADIUS = 6370027.0


        self.bounds = []

        with open(self.bound_file) as f:
            for line in f:
                latlon = line.split(' ')
                lat = float(latlon[0])
                lon = float(latlon[1])
                # print lat
                # print lon
                N = self.EARTH_RADIUS*(lat - self.init_lat)*np.pi/180.0;
                E = self.EARTH_RADIUS*cos(lat*np.pi/180.0)*(lon - self.init_lon)*np.pi/180.0;
                p = Point(N, E)
                self.bounds.append(p)

    def pointInBounds(self, p_N, p_E):
        p = Point(p_N, p_E)
        ans = pointInPolygon(p, self.bounds)
        return ans



# polygon = [Point(0,3), Point(5,3), Point(0,0)]
#
# p = Point(0.5,1)
#
# ans = pointInPolygon(p, polygon)
#
# print ans

bounders = inBounds()

print bounders.pointInBounds(0,0)

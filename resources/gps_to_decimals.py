#!/usr/bin/env python
import numpy as np

class DMS:
    def __init__(self):
        self.lat = 0
        self.lon = 0
    def calc(self):
        self.lat_dec = self.lat[0] + (self.lat[1]/60.0) + (self.lat[2]/(60.0*60.0))
        self.lon_dec = self.lon[0] + (self.lon[1]/60.0) + (self.lon[2]/(60.0*60.0))
        self.lon_dec *= -1

A1 = DMS()
A1.lat = [38, 8, 46.57]
A1.lon = [76, 25, 41.39]

A2 = DMS()
A2.lat = [38, 9, 5.85]
A2.lon = [76, 25, -43.26]

A3 = DMS()
A3.lat = [38, 9, 6.8]
A3.lon = [76, 25, 53.28]

bounds = [A1, A2, A3]

for point in bounds:
    point.calc()
    print point.lat_dec, point.lon_dec

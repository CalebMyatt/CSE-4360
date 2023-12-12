#!/usr/bin/env pybricks-micropython
# README
# desmos sim at https://www.desmos.com/calculator/uz68isf59x
# MicroPython docs at https://pybricks.com/ev3-micropython/
# =============================================================================
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from math import pi, cos, sin, radians, degrees, sqrt
def dist(*point):
    return sqrt(sum(num**2 for num in point))

# =============================================================================
class Robot:
    def reset(self):
        self.tm.run_until_stalled(45, Stop.HOLD)
        self.tm.reset_angle(330)

        self.bm.run_until_stalled(-45, Stop.HOLD)
        self.bm.reset_angle(330-195)

        self.tm.run_target(45,180, Stop.HOLD)
        self.bm.run_target(45,90, Stop.HOLD)

        self.tm.stop()
        self.bm.stop()

        self.hm.reset_angle(0)

    def __init__(self):
        self.tm = Motor(Port.C, Direction.COUNTERCLOCKWISE) # top motor
        self.bm = Motor(Port.D, Direction.COUNTERCLOCKWISE) # bottom motor
        self.hm = Motor(Port.B, Direction.COUNTERCLOCKWISE) # hand motor

        self.L = 100
        self.V = 50

        self.reset()

    def cur_thetas(self):
        t1 = radians(self.bm.angle())
        t2 = radians(self.tm.angle())
        return t1, t2
    def cur_pos(self):
        t1, t2 = self.cur_thetas()
        L = self.L
        x = -L*(cos(t2)-cos(t1))
        y = -L*(sin(t2)-sin(t1))
        return x, y
    def dist_to(self, pos):
        x1, y1 = self.cur_pos()
        x2, y2 = pos
        dx = x1-x2
        dy = y1-y2
        return dx, dy
    def verify_point(self, pos):
        L = self.L
        if dist(*pos) > 2*L:
            raise ValueError('vertex is too far away')

    def move_to(self, pos, d_thr=2):
        V, L = self.V, self.L
        self.verify_point(pos)

        while True:
            dx, dy = self.dist_to(pos)
            d = dist(dx, dy)
            if d < d_thr:
                return
            t1, t2 = self.cur_thetas()
            v1 = -(1/d)*(cos(t2)*dx+sin(t2)*dy)/L
            v2 = -(1/d)*(cos(t1)*dx+sin(t1)*dy)/L
            self.bm.run(degrees(v1)*V)
            self.tm.run(degrees(v2)*V)
            wait(1)
    def draw_polygon(self, poly, off=(0,0)):
        poly.translate(off)
        for point in poly:
            self.verify_point(point)

        self.move_to(poly[0])
        self.start_drawing()

        for point in poly:
            wait(100)
            self.move_to(point)
            print(point, self.cur_pos())

        self.stop_drawing()

    def start_drawing(self):
        self.hm.run_target(180, 90)
    def stop_drawing(self):
        self.hm.run_target(180, 0)
# =============================================================================
class Polygon:
    def __init__(self, points):
        if len(points) == 0:
            raise ValueError('polygon must have at least one vertex')
        self.verticies = points
    
    def circle(pos, radius, num_verts=100):
        cx = pos[0]
        cy = pos[1]

        points = []
        for step in range(num_verts):
            theta = 2*pi * (step/num_verts)
            x = cx + radius*cos(theta)
            y = cy + radius*sin(theta)
            points.append((x,y))

        return Polygon(points)

    def translate(self, off):
        dx, dy = off
        self.verticies = [(x+dx, y+dy) for x, y in self.verticies]
    def __iter__(self):
        for pos in self.verticies:
            yield pos
        yield self.verticies[0]
    def __getitem__(self, i):
        return self.verticies[i]
# =============================================================================
def main():
    poly = Polygon([(10,10),
                    (140,10),
                    (140,90),
                    (10,90),])
    poly.translate((-17,36))
    rob = Robot()
    rob.draw_polygon(poly)

    rob.tm.stop()
    rob.bm.stop()

    # while True:
    #     print(rob.cur_pos())
    #     wait(1000)
    
if __name__ == '__main__':
    main()

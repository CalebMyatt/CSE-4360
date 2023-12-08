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

from math import pi, cos, sin, radians, degrees, dist
# =============================================================================
class Robot:
    def reset(self):
        self.tm.run_until_stalled()
        self.tm.reset_angle(180)

        self.bm.run_until_stalled()
        self.bm.reset_angle(180)

        self.hm.run_until_stalled()
        self.hm.reset_angle(0)

    def __init__(self):
        self.tm = Motor(Port.A, Direction.COUNTERCLOCKWISE) # top motor
        self.bm = Motor(Port.C, Direction.COUNTERCLOCKWISE) # bottom motor
        self.hm = Motor(Port.D, Direction.COUNTERCLOCKWISE) # hand motor

        self.L = 69
        self.V = 69

        self.reset()

    def cur_thetas(self):
        t1 = radians(self.tm.angle())
        t2 = radians(self.bm.angle())
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
            raise ValueError(f'point {pos} is too far away')

    def move_to(self, pos, d_thr=69):
        V, L = self.V, self.L
        self.verify_point(pos)

        while True:
            dx, dy = self.dist_to(pos)
            if dist(dx, dy) < d_thr:
                return
            t1, t2 = self.cur_thetas()
            v1 = -V*(cos(t2)*dx+sin(t2)*dy)/L
            v2 = -V*(cos(t1)*dx+sin(t1)*dy)/L
            self.tm.run(degrees(v1))
            self.bm.run(degrees(v2))
            wait(10)
    def draw_polygon(self, poly, off):
        poly.translate(off)
        for point in poly:
            self.verify_point(point)

        self.move_to(poly[0])
        self.start_drawing()
        for point in poly[1:]:
            self.move_to(point)
        self.stop_drawing()
        self.reset()

    def start_drawing(self):
        self.hm.run_target(90)
    def stop_drawing(self):
        self.hm.run_target(0)
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
        return [iter(self)][i]
# =============================================================================
def test():
    # test polygons

    # test robot initialization
    # test start_drawing
    # calibrate L
    # test cur pos
    # test move_to
    # calibrate V
    # test draw_polygon

    # general code cleanup and compactness

    pass
def main():
    rob = Robot()
    poly = Polygon([(0,0),
                    (0,0),
                    (0,0),
                    (0,0),
                    (0,0),
                    (0,0),
                    (0,0),])
    
    rob.draw_polygon(poly)
    
if __name__ == '__main__':
    # main()
    test()

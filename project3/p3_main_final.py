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
        speed = 45
        self.tm.run_until_stalled(speed, Stop.HOLD)
        self.tm.reset_angle(330)

        self.bm.run_until_stalled(-speed, Stop.HOLD)
        self.bm.reset_angle(330-195)

        self.hm.run_until_stalled(speed,Stop.BRAKE,None)
        self.hm.reset_angle(0)

        self.tm.run_target(speed,180, Stop.HOLD)
        self.bm.run_target(speed,90, Stop.HOLD)

        self.tm.stop()
        self.bm.stop()

        #Create something to adjust pen


    def __init__(self):
        self.tm = Motor(Port.C, Direction.COUNTERCLOCKWISE) # top motor
        self.bm = Motor(Port.D, Direction.COUNTERCLOCKWISE) # bottom motor
        self.hm = Motor(Port.B, Direction.COUNTERCLOCKWISE) # hand motor

        self.L = 112
        self.V = 50
        self.pen_offset = -80

        self.reset()

    
    def cur_thetas(self):
        t1 = radians(self.bm.angle())
        t2 = radians(self.tm.angle())
        return t1, t2
    def get_x_y(self):
        t1, t2 = self.cur_thetas()
        L = self.L
        x = -L*(cos(t2)-cos(t1))
        y = -L*(sin(t2)-sin(t1))
        return x, y
  
    def safe_position(self,pos):
        L = self.L
        if dist(*pos) > 2*L:
            raise ValueError('vertex is too far away')
    
    def dist_to(self, pos):
        x1, y1 = self.get_x_y()
        x2, y2 = pos
        dx = x1-x2
        dy = y1-y2
        return dx, dy

    def move_to(self, pos, d_thr=2):
        V, L = self.V, self.L
        self.safe_position(pos)

    
        while True:
            dx, dy = self.dist_to(pos)
            d = dist(dx, dy)
            if d < d_thr:
                break
            t1, t2 = self.cur_thetas()
            v1 = -(1/d)*(cos(t2)*dx+sin(t2)*dy)/L
            v2 = -(1/d)*(cos(t1)*dx+sin(t1)*dy)/L
            self.bm.run(degrees(v1)*V)
            self.tm.run(degrees(v2)*V)
            #print(self.get_x_y(),d)
            wait(1)
        self.bm.brake()
        self.tm.brake()
        

    def draw_polygon(self, poly, off=(0,0)):
        poly.translate(off)
        for point in poly:
            self.safe_position(point)

        self.move_to(poly[0])
        
        self.ready_pen(True)

        for point in poly:
            wait(100)
            self.move_to(point)
            #print(point, self.get_x_y())

        self.ready_pen(False)

    def ready_pen(self,ready):
        if ready:
          print("ready")
          self.hm.run_target(45, self.pen_offset, then=Stop.BRAKE, wait=True)
        else:
          self.hm.run_target(45, 0, then=Stop.BRAKE, wait=True)


# =============================================================================

class circle_n:
  def __init__(self, origin, radius, number_of_points):
    #Selects the origin of the circle
    self.origin = origin # (x, y, rads from +x)
    #Selects the radius of the circle
    self.radius = radius # (x,y)
    #Creates the vertex surounding the vertex
    self.number_of_points = number_of_points # (x_size, y_size)
    self.points = []
    for x in range(number_of_points):
      percentage_of_circle = x/number_of_points
      new_point = self.radius * cos(2*pi*percentage_of_circle) + self.origin[0] , self.radius * sin(2*pi*percentage_of_circle)+ self.origin[1]
      self.points.append(new_point)
    
  def translate(self,off):
      dx, dy = off
      self.points = [(x+dx, y+dy) for x, y in self.points]

  def __iter__(self):
        for pos in self.points:
            yield pos
        yield self.points[0]
  def __getitem__(self, i):
        return self.points[i]



# =============================================================================
class Polygon:
    def __init__(self, points):
        if len(points) == 0:
            raise ValueError('polygon must have at least one vertex')
        self.verticies = points
    
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
    #print("HAW")

    #rob.draw_polygon(poly)

    cool_circle = circle_n((55,75), 30, 3)

    #triangle = Polygon([(25,10),
    #                (140,10),
    #                (140,80),])
    #triangle.translate((-17,36))

    #Polygon = circle_n((50,75),) 
    rob.draw_polygon(cool_circle)

    #print("LOL")
    rob.tm.stop()
    rob.bm.stop()

    # while True:
    #     print(rob.cur_pos())
    #     wait(1000)
    
if __name__ == '__main__':
    main()





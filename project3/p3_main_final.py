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

        #Create something to adjust pen
        self.hm.run_until_stalled(self.SPEED,Stop.BRAKE,None)
        self.hm.reset_angle(0)

    def __init__(self):
        self.tm = Motor(Port.C, Direction.COUNTERCLOCKWISE) # top motor
        self.bm = Motor(Port.D, Direction.COUNTERCLOCKWISE) # bottom motor
        self.hm = Motor(Port.B, Direction.COUNTERCLOCKWISE) # hand motor

        self.L = 112
        self.V = 50

        self.pen_offset = -60


        self.reset()

    def safe_position(self,x,y):
      safety_dist = 0
      if dist((x,y)) >= (2*self.length - safety_dist):
        self.bottom.brake()
        self.top.brake()
        return False
      return True
    
    def cur_thetas(self):
        t1 = radians(self.bm.angle())
        t2 = radians(self.tm.angle())
        return t1, t2
    def get_x_y(self):
      theta2 , theta1 = (degrees(self.top.angle()), degrees(self.bottom.angle())) 
      return (-self.length *(cos(theta2)- cos(theta1)),-self.length *(sin(theta2)- sin(theta1)))
  
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
      new_point = self.radius * math.cos(2*PI*percentage_of_circle) + self.origin[0] , self.radius *math.sin(2*PI*percentage_of_circle)+ self.origin[1]
      self.points.append(new_point)

  #Description: Gives you the points for the problem you want to solve
  #Args:    Self
  #Returns: An array of points, floats
  def get_points(self):
    return self.points
  
  def solve_problem(self,robot):
    end = (self.get_points())[-1]
    robot.ready_pen(False)
    print(end[0],end[1])

    robot.move_to(end[0],end[1])
    robot.ready_pen(True)
    for point in self.get_points():
      robot.move_to(point[0],point[1])
      robot.wait_motor()

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




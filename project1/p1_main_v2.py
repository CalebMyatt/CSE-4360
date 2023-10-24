#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time
import sys
from math import sqrt

PI = 3.1415926535897932384626
FT = 0.3048
# =============================================================================
def deg_to_rad(deg):
  return deg*PI/180
def rad_to_deg(rad):
  return rad*180/PI

# ENUMS
class Abs_Direction:
  EAST =  (1,0)
  NORTH = (0,1)
  WEST =  (-1,0)
  SOUTH = (0,-1)
class Rel_Direction:
  FORWARD =       (0.0*PI)/4.0
  FORWARD_LEFT =  (1.0*PI)/4.0
  LEFT =          (2.0*PI)/4.0
  BACK_LEFT =     (3.0*PI)/4.0
  BACK =          (4.0*PI)/4.0
  BACK_RIGHT =    (5.0*PI)/4.0
  RIGHT =         (6.0*PI)/4.0
  FORWARD_RIGHT = (7.0*PI)/4.0


class Robot:
  def __init__(self, speed):
    # calibrated from distance travel
    self.WHEEL_RADIUS = .039 # wheel radius (m)
    self.WHEEL_CIRCUMFRENCE = 2*PI*self.WHEEL_RADIUS

    # calibrated from 90 deg turns
    self.AXLE_RADIUS = .087 # axle radius (m)
    self.AXLE_CIRCUMFRENCE = 2*PI*self.AXLE_RADIUS

    self.SPEED = speed # 1.0 is the standard speed
    self.comp = 1.007

    self.lm = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    self.rm = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# units radians and meters
  def turn_left(self, rad):
    req_axle_rot = rad*self.AXLE_RADIUS # arclength of axle circumfrence
    req_wheel_rot = 2*PI*(req_axle_rot/self.WHEEL_CIRCUMFRENCE) # required rotation of wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.SPEED, -rad_to_deg(req_wheel_rot), wait=False)
    self.rm.run_target(self.comp*self.SPEED, self.comp*rad_to_deg(req_wheel_rot))
  def turn_right(self, rad):
    self.turn_left(-rad)
  def move_forward(self, dist):
    if dist == 0: return
    req_wheel_rot = 2*PI*(dist/self.WHEEL_CIRCUMFRENCE) # required rotation of wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.SPEED, rad_to_deg(req_wheel_rot), wait=False)
    self.rm.run_target(self.comp*self.SPEED, self.comp*rad_to_deg(req_wheel_rot))

  def solve_problem(self, problem):
    ev3 = EV3Brick()

    ev3.speaker.beep() # start
    route = self.calculate_route(problem)
    route = self.simplify_route(route)
    print(route)

    ev3.speaker.beep() # start moving
    self.execute_route(route)

    ev3.speaker.beep() # done
  def calculate_route(self, problem):
    (goal_x, goal_y) = Problem.corner_point_to_field(problem.goal)
    (cur_x, cur_y, direction) = problem.start
    (cur_x, cur_y) = Problem.corner_point_to_field((cur_x,cur_y))

    if problem.field[cur_x+1][cur_y] == -1:
      print("No solution found")
      return []

    actions = []
    while cur_x != goal_x or cur_y != goal_y:
      # getting absolute directions
      east = problem.field[cur_x+1][cur_y]
      west = problem.field[cur_x-1][cur_y]
      north = problem.field[cur_x][cur_y+1]
      south = problem.field[cur_x][cur_y-1]

      # getting the frame that the robot sees
      absolute_counterclockwise = [east,north,west,south]
      abs_direction = int( rad_to_deg(direction)/90.0 ) # this should only be integer increments
      relative_counterclockwise = absolute_counterclockwise[abs_direction:] + absolute_counterclockwise[:abs_direction]

      # extracting values
      forward = relative_counterclockwise[0]
      left = relative_counterclockwise[1]
      back = relative_counterclockwise[2]
      right = relative_counterclockwise[3]

      # chosing action to take
      lowest = min(relative_counterclockwise)
      chosen_dir = 0
      if lowest == forward:
        pass
      elif lowest == left:
        chosen_dir = 1
        actions.append((self.turn_left, deg_to_rad(90.0)))
        direction += PI/2.0
      elif lowest == right:
        chosen_dir = -1
        actions.append((self.turn_right, deg_to_rad(90.0)))
        direction -= PI/2.0
      else:
        chosen_dir = 2
        actions.append((self.turn_right, deg_to_rad(180.0)))
        direction += PI
      actions.append((self.move_forward, 0.5*FT))

      # helper dict
      move = [(1,0),(0,1),(-1,0),(0,-1)]

      
      # updating position
      (t_x, t_y) = move[(chosen_dir + abs_direction) % 4]
      cur_x += t_x
      cur_y += t_y

    return actions
  def simplify_route(self, route):
    new_route = []

    dist = 0
    for (action, args) in route:
      if action.__name__ == self.move_forward.__name__:
        dist += args
      else:
        new_route.append((self.move_forward, dist))
        dist=0
        new_route.append((action, args))
    new_route.append((self.move_forward, dist))

    return new_route
  def execute_route(self, route):
    for (action, arg) in route:
      action(arg)
      time.sleep( 1.0 )

class Problem:
  def __init__(self, start, goal, size, obstacles, obstacle_size=2):
    self.start = start # (x, y, rads from +x)
    self.goal = goal # (x,y)
    self.size = size # (x_size, y_size)

    self.init_field()
    self.bounding_box()
    for (p1, p2) in obstacles:
      self.add_obstacle(p1, p2, obstacle_size)
    self.floodfill()
  
  def corner_point_to_field(pos):
    (x,y) = pos
    return (x*2,y*2)
    
  def init_field(self):
    (x_size, y_size) = Problem.corner_point_to_field(self.size)

    self.field = [ [-1]*(y_size+1) for i in range(x_size+1)] # field[x][y]
    return self.field
  def bounding_box(self):
    # bounding box
    for i in range(len(self.field)):           # top and bottom side bounding box
      self.field[i][0] = sys.maxsize
      self.field[i][-1] = sys.maxsize
    for i in range(len(self.field[0])):           # left and right side bounding box
      self.field[0][i] = sys.maxsize
      self.field[-1][i] = sys.maxsize

    return self.field
  def floodfill(self):
    # setup goal
    (goal_x, goal_y) = Problem.corner_point_to_field(self.goal)
    self.field[goal_x][goal_y] = 0
    fringe = [(goal_x, goal_y, self.field[goal_x][goal_y])]

    # floodfill from goal
    while len(fringe) > 0:
      (x, y, val) = fringe.pop(0)

      if self.field[x+1][y] < 0:  # right
        self.field[x+1][y] = val+1
        fringe.append((x+1, y, self.field[x+1][y]))
      if self.field[x][y+1] < 0:  # up
        self.field[x][y+1] = val+1
        fringe.append((x, y+1, self.field[x][y+1]))
      if self.field[x-1][y] < 0:  # left
        self.field[x-1][y] = val+1
        fringe.append((x-1, y, self.field[x-1][y]))
      if self.field[x][y-1] < 0:  # down
        self.field[x][y-1] = val+1
        fringe.append((x, y-1, self.field[x][y-1]))

    return self.field

  def print_field(self):
    transpose = [[self.field[j][i] for j in range(len(self.field))] for i in range(len(self.field[0]))]

    (s_x, s_y) = Problem.corner_point_to_field(self.start[:2])
    s_y = len(self.field[0]) - s_y - 1
    print(s_x, s_y)
    
    for (i, row) in enumerate(reversed(transpose)):
      for (j, weight) in enumerate(row):
        if weight == sys.maxsize:
          weight = 999

        if i == s_y and j == s_x:
          print("<%3d>," % (weight), end='')
        elif (i)%2 == 0 and (j)%2 == 0:
          print("[%3d]," % (weight), end='')
        else:
          print("%4d ," % (weight), end='')
      print()

  # size is radius of an obstacle. 1=normal, 2=safe
  def add_obstacle(self, p1, p2, size):
    # extracting values
    (x1,y1) = Problem.corner_point_to_field(p1)
    (x2,y2) = Problem.corner_point_to_field(p2)
    # size of obstacle
    len_x = abs(x2-x1) + 1 + 2*size
    len_y = abs(y2-y1) + 1 + 2*size
    # get bottom left corner
    blx = min(x1,x2) - size
    bly = min(y1,y2) - size
    for dx in range(len_x):
      for dy in range(len_y):
        self.field [blx + dx][bly + dy] = sys.maxsize
# ===============
robot = Robot(90.0)

start = (1, 5, 0.0)
goal = (9,5)
size = (16,10)
obstacles = [((1,1),(2,2)),
             ((2,7),(2,8)),
             ((5,4),(5,6)),
             ((8,2),(10,2)),
             ((8,8),(8,8))]
problem = Problem(start, goal, size, obstacles)
problem.print_field()

robot.solve_problem(problem)

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
import math

    #self.top.run_target(self.SPEED, (self.top.angle() + 25), Stop.HOLD, True)  
    #self.bottom.run_target(self.SPEED, (self.bottom.angle() - 25), Stop.HOLD, True) 
    #self.bottom.run_target(self.SPEED, -160, Stop.HOLD, True)

    #      C   
    #     / \
    #    /   \
    #   /     \
    #  B       D
    #   \     / \
    #    \   /   \
    #     \ /     \
    #      A       E

    #   origin(0,0)
    #   Reset/Calibrate position - Both motors turn towards each other to straighten arm, 
    #       -then both motors turn counter clockwise until unable
    #   Go to Origin of grid - get angle in memory function motor.angle, bottom left of grid is 0,0
    #   Array of vectors which creates shape, (x,y)
    #   Create points using bezier curve
    #   Marker move up and down function
    #   Move to first point, start drawing
    #   Once back at first point, go to sleep

   # time:      ms
   # Distance:  mm
   # Speed:     mm/s

#Local Pi value
PI = 3.1415926535897932384626

#Helper Functions
#------------------------------------
def deg_to_rad(deg):
  return deg*PI/180
def rad_to_deg(rad):
  return rad*180/PI
def abs_distance(location):
  return math.sqrt(location[0]**2 + location[1]**2)

class Robot:
  #Description: Create self plus gets a speed
  #Args:    Speed
  #Returns: Robot Class
  def __init__(self, speed):
    #Argus
    self.SPEED = speed # 1.0 is the standard speed

    #Adjustable in coding
    #NOTE: we could just make all of these args
    #self.length is in mm(Millimeters) 
    self.length = 112
    #self.safety_distance is in mm(Millimeters) 
    self.safety_distance = 0
    # How close do we move per second(Higher number equals slower movement)
    self.steps = 25
    #How close do we have to be in mm(Millimeters) 
    self.precision = 2
    #How fast we change the velocity
    self.time_per_move = .1
    # limit how fast we can move
    self.limit = 360
    #How many degrees to move the pen
    self.pen_offset = -60
  
    #Sets motor
    self.top = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    self.bottom = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    self.pen = Motor(Port.B,Direction.COUNTERCLOCKWISE)
    
  #Description: Return current angle as a tuple (Radians, Module to 2*PI )
  #Args:    None
  #Returns: Top angle     (Theta 2)
  #         Bottom angle  (Theta 1)
  def get_angle(self):
    return deg_to_rad(self.bottom.angle())%(2*PI), deg_to_rad(self.top.angle())%(2*PI)


  #Description: Resets position to (0,0)
  #Args:    None
  #Returns: None
  def resetPosition_old(self):
    #Get the "painting" motor to 0
    self.pen.run_until_stalled(self.SPEED,Stop.BRAKE,None)
    self.pen.reset_angle(0)
    self.pen.run_target(self.SPEED, self.pen_offset*2, then=Stop.BRAKE, wait=True)

    
    self.bottom.run_until_stalled(self.SPEED, Stop.BRAKE, None)
    self.top.hold()
    self.bottom.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.bottom.hold()
    self.top.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.rotate_all(55)
    self.top.reset_angle(180)
    self.bottom.reset_angle(178)
    self.bottom.run_target(self.SPEED,90,then=Stop.BRAKE, wait=True)

  def resetPosition(self):
    
    self.bottom.run_until_stalled(self.SPEED, Stop.HOLD, None)
    self.top.hold()
    self.bottom.run_until_stalled(-self.SPEED, Stop.HOLD, None)

    self.top.reset_angle(330)
    self.bottom.reset_angle(130)
    self.top.run_target(-self.SPEED,180,Stop.HOLD,True)
    self.bottom.run_target(-self.SPEED,90,Stop.HOLD,True)

    #Get the "painting" motor to 0
    self.pen.run_until_stalled(self.SPEED,Stop.BRAKE,None)
    self.pen.reset_angle(0)

   
  #Description: Rotate the entire robot
  #Args:    None
  #Returns: None
  def rotate_all(self,deg):
    self.top.run_target(self.SPEED, (self.top.angle() + deg), Stop.HOLD, False)  
    self.bottom.run_target(self.SPEED, (self.bottom.angle() + deg), Stop.HOLD, True)
    
  #Description: Checks if given (x,y) is safe to move to
  #Args:    X location
  #         Y location
  #Returns: Boolean value
  def safe_position(self,x,y):
    # Checks if (x,y) is out of range
    #print(abs_distance((x,y)))
    if abs_distance((x,y)) >= (2*self.length - self.safety_distance):
      self.bottom.brake()
      self.top.brake()
      return False
    # Checks if (x,y) leaves our painting zone
    #elif x < 0 or y < 0:
    #  print("Below 0")
    #  return False
    return True
  
  
  #Description: Gets current (x,y) location
  #Args:    None
  #Returns: X Location
  #         Y: Location
  def get_x_y(self):
    #Top is theta2 and bot is theta1
    theta2 , theta1 = (deg_to_rad(self.top.angle()), deg_to_rad(self.bottom.angle())) 
    return (-self.length *(math.cos(theta2)- math.cos(theta1)),-self.length *(math.sin(theta2)- math.sin(theta1)))
    
  #Description: Moves to a location descpried in (x,y)
  #Args:    X location
  #         Y location
  #Returns: None 
  def move_to(self, x, y):
    p2 = x,y
    
    if self.safe_position(x,y) == False:
      print("UNSAFE POSITION")
      return 
    
    # p1 is current and p2 to end goal
    while (True):
      p1 = self.get_x_y()
      
      theta1,theta2 = self.get_angle()
      if ( abs_distance((p2[0]-p1[0],(p2[1]-p1[1]))) < self.precision):
        self.bottom.brake()
        self.top.brake()
        break
      distance_x = (p1[0]-p2[0])/self.steps
      distance_y = (p1[1]-p2[1])/self.steps

      velocity_1 = -(distance_x * math.cos(theta1) +  distance_y*math.sin(theta1))/self.length
      velocity_2 = -(distance_x * math.cos(theta2) +  distance_y*math.sin(theta2))/self.length

      move_top_motor  = rad_to_deg(velocity_1) * self.SPEED
      move_bottom_motor = rad_to_deg(velocity_2) * self.SPEED



      #print("vel1:", move_bottom_motor)
      #print("vel2:", move_top_motor)


      if(move_bottom_motor > self.limit or move_top_motor > self.limit):
        print("TO FAST")
        return

      #self.bottom.run_time((velocity_1),self.time_per_move, Stop.COAST,False)
      #self.top.run_time((velocity_2),self.time_per_move, Stop.COAST,False)
      self.bottom.run(move_bottom_motor)
      self.top.run(move_top_motor) 

      #time.sleep(self.time_per_move)
    #print(self.get_x_y())
 
  #Description: Makes motor wait
  #Args:    stop_fn take in a function
  #         params is the parameters for the function given
  #Returns: None
  def wait_motor(self, stop_fn=lambda:False, params=()):
    while not self.top.control.done() or not self.bottom.control.done():
      if stop_fn(*params):
        self.top.brake()
        self.bottom.brake()
    return
  
  def ready_pen(self,ready):
    if ready:
      print("ready")
      self.pen.run_target(self.SPEED, self.pen_offset, then=Stop.BRAKE, wait=True)
    else:
      self.pen.run_target(self.SPEED, self.pen_offset*2, then=Stop.BRAKE, wait=True)





#Description: Class for the problem we are solving for.
#Args:    Self
#         origin: Where the middle of the circle will be
#         radius: how big of a circle
#         number_of_points: how many vertex should we have, we need 3 atleast
#Returns: None
class Problem:
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
    robot.move_to(end[0],end[1])
    robot.ready_pen(True)
    for point in problem.get_points():
      robot.move_to(point[0],point[1])
      robot.wait_motor()
    #robot.move_to(start[0],start[1])
    #robot.wait_motor()
  
  def solve_problem_outside_points(self,robot,points):
    end = (points)[-1]
    robot.ready_pen(False)
    robot.move_to(end[0],end[1])
    robot.ready_pen(True)
    for point in points():
      robot.move_to(point[0],point[1])
      robot.wait_motor()
    #robot.move_to(start[0],start[1]) 
    #robot.wait_motor()

  
    
# ===============
#Sets speed
robot = Robot(90)
robot.ready_pen(False)

#Reset Position
robot.resetPosition()
print(robot.get_x_y())
print(robot.get_angle())

#Moving

#Origin of Paper
#robot.move_to(-60,60)
#Middle of Paper
#robot.move_to(80,100)

#  def __init__(self, origin, radius, number_of_points):

#problem = Problem((80,100),30, 4)

#print(problem.get_points())
#points = []
#robot.ready_pen(True)
#problem.solve_problem(robot)
#robot.ready_pen(False)


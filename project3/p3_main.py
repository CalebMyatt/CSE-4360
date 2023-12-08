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
    self.length = 110
    #self.safety_distance is in mm(Millimeters) 
    self.safety_distance = 0
    # How close do we move per second(Higher number equals slower movement)
    self.steps = 20
    #How close do we have to be in mm(Millimeters) 
    self.precision = 2
    #How fast we change the velocity
    self.time_per_move = .01
    # limit how fast we can move
    self.limit = 360
  
    #Sets motor
    self.top = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    self.bottom = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    
  #Description: Return current angle as a tuple (Radians, Module to 2*PI )
  #Args:    None
  #Returns: Top angle     (Theta 2)
  #         Bottom angle  (Theta 1)
  def get_angle(self):
    return deg_to_rad(self.bottom.angle())%(2*PI), deg_to_rad(self.top.angle())%(2*PI)


  #Description: Resets position to (0,0)
  #Args:    None
  #Returns: None
  def resetPosition(self):
    self.bottom.run_until_stalled(self.SPEED, Stop.BRAKE, None)
    self.top.hold()
    self.bottom.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.bottom.hold()
    self.top.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.rotate_all(57.5)
    self.top.reset_angle(180)
    self.bottom.reset_angle(178)
   
    
  #Description: Rotate the entire robot
  #Args:    None
  #Returns: None
  def rotate_all(self,deg):
    self.top.run_target(self.SPEED, (self.top.angle() + deg), Stop.HOLD, False)  
    self.bottom.run_target(self.SPEED, (self.bottom.angle() + deg), Stop.HOLD, True)
    
  
  #Description: Moves the motors to an X and Y location(Old way)
  #Args:    Go to X location
  #         Go to Y location
  #Returns: None
  def move_to_old(self, x, y):
    #print("Start Location: ",xPrime, yPrime)
    #print("Start angle:",self.top.angle(), self.bottom.angle())
    #print("b:",b)
    #print("gamma",gamma)
    #beta =  rad_to_deg(math.acos(1-((b**2)/(2*self.length**2))))
    #Gets angle of Beta and Alpha
    #print(1-(b**2/(2*self.length**2)))
    #print("beta",beta,"alpha",alpha)
    #print("theta1", theta1, "\ntheta2", theta2)
    #self.wait_motor()
    #print("end location",self.get_x_y())
    #self.bottom.run_target(self.SPEED, theta1, Stop.HOLD, True)
    
    xPrime,yPrime = self.get_x_y()
    b = math.sqrt(((x - xPrime)**2) + ((y-yPrime)**2))
    gamma = rad_to_deg(math.asin(((y-yPrime)/b)))
    beta = rad_to_deg(math.acos(1-(b**2/(2*self.length**2))))
    alpha = (180 - beta)/2
    theta1 = alpha + gamma
    theta2 = theta1 + beta
    self.bottom.run_target(self.SPEED, theta1, Stop.HOLD,False ) 
    self.top.run_target(self.SPEED, theta2, Stop.HOLD,True ) 
    
    
  #Description: Checks if given (x,y) is safe to move to
  #Args:    X location
  #         Y location
  #Returns: Boolean value
  def safe_position(self,x,y):
    # Checks if (x,y) is out of range
    if abs_distance((x,y)) >= (2*self.length - self.safety_distance):
      self.bottom.brake()
      self.top.brake()
      return False
    # Checks if (x,y) leaves our painting zone
    elif x < 0 or y < 0:
      print("Below 0")
      return False
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
      print(p1)

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
      time.sleep(self.time_per_move)
    
 
  #Description: Makes motor wait
  #Args:    (Ask Landon :3)
  #Returns: None
  def wait_motor(self, stop_fn=lambda:False, params=()):
    while not self.top.control.done() or not self.bottom.control.done():
      if stop_fn(*params):
        self.top.brake()
        self.bottom.brake()
    return
  


#Description: Class for the problem we are solving for.
#Args:    Self
#         origin: Where the middle of the circle will be
#         radius: how big of a circle
#         number_of_points: howmany vertex should we have
#Returns: None
class Problem:
  def __init__(self, origin, radius, number_of_points):
    self.origin = origin # (x, y, rads from +x)
    self.radius = radius # (x,y)
    self.number_of_points = number_of_points # (x_size, y_size)
    self.points = []
    for x in range(number_of_points):
      percentage_of_circle = x/number_of_points
      new_point = math.cos(2*PI*percentage_of_circle) + self.origin[0] , math.sin(2*PI*percentage_of_circle)+ self.origin[1]
      self.points.append()
  def get_points(self):
    return self.points
  

    
# ===============
#Sets speed
robot = Robot(90)

#Reset Position
robot.resetPosition()
print(robot.get_x_y())
print(robot.get_angle())

#Moving
robot.move_to(0, 200)
time.sleep(5)

robot.move_to(0, 100)

time.sleep(5)


robot.move_to(50, 50)



#robot.move_to(100, 100)



#print(robot.reset_position())

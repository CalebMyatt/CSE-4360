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
PI = 3.1415926535897932384626

def deg_to_rad(deg):
  return deg*PI/180
def rad_to_deg(rad):
  return rad*180/PI

# ENUMS
class Robot:
  #Create self plus gets a speed
  def __init__(self, speed):
    #mm 
    self.length = 112
    self.SPEED = speed # 1.0 is the standard speed
    self.top = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    self.bottom = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    
  
  #Resets position to (0,0)
  def resetPosition(self):
    self.bottom.run_until_stalled(self.SPEED, Stop.HOLD, None)
    self.top.hold()
    self.bottom.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.bottom.hold()
    self.top.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.rotate_all(57.5)
    self.top.reset_angle(180)
    self.bottom.reset_angle(180)
    self.bottom.run_target(self.SPEED, 180-45 ,Stop.HOLD, True)
    #   time ms
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
  # Rotate the entire robot
  def rotate_all(self,deg):
    self.top.run_target(self.SPEED, (self.top.angle() + deg), Stop.HOLD, False)  
    self.bottom.run_target(self.SPEED, (self.bottom.angle() + deg), Stop.HOLD, True)
     
  #Move to an X and Y location   
  def move_to(self, x, y):
    xPrime,yPrime = self.get_x_y()
    print("Start Location: ",xPrime, yPrime)
    print("Start angle:",self.top.angle(), self.bottom.angle())

    b = math.sqrt(((x - xPrime)**2) + ((y-yPrime)**2))
    print("b:",b)
    gamma = rad_to_deg(math.asin(((y-yPrime)/b)))
    print("gamma",gamma)
    #beta =  rad_to_deg(math.acos(1-((b**2)/(2*self.length**2))))

    #Gets angle of Beta and Alpha
    #print(1-(b**2/(2*self.length**2)))
    beta = rad_to_deg(math.acos(1-(b**2/(2*self.length**2))))
    alpha = (180 - beta)/2
    print("beta",beta,"alpha",alpha)

    theta1 = alpha + gamma
    theta2 = theta1 + beta
    print("theta1", theta1, "\ntheta2", theta2)
    #self.wait_motor()

    self.bottom.run_target(self.SPEED, theta1, Stop.HOLD,False ) 
    self.top.run_target(self.SPEED, theta2, Stop.HOLD,True ) 
    print("end location",self.get_x_y())
    #self.bottom.run_target(self.SPEED, theta1, Stop.HOLD, True) 
  
  def wait_motor(self, stop_fn=lambda:False, params=()):
    while not self.top.control.done() or not self.bottom.control.done():
      if stop_fn(*params):
        self.top.brake()
        self.bottom.brake()
    return
  
  def get_x_y(self):
    #Top is theta2 and bot is theta1
    theta2 , theta1 = deg_to_rad((self.top.angle(), self.bottom.angle()))
    return (-self.length *(math.cos(theta2)- math.cos(theta1)),-self.length *(math.sin(theta2)- math.sin(theta1)))
  
  def get_angle(self):
    return self.top.angle(), self.bottom.angle()

class Problem:
  def __init__(self, start, goal, size, obstacles, obstacle_size=2):
    self.start = start # (x, y, rads from +x)
    self.goal = goal # (x,y)
    self.size = size # (x_size, y_size)

# get angle from x, 
    
# ===============
robot = Robot(90)
robot.resetPosition()
print(robot.get_x_y())
print(robot.get_angle())
#robot.move_to(0, 100)
#robot.move_to(100, 100)

#print(robot.reset_position())

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
  def __init__(self, speed):
    self.length = 112
    self.SPEED = speed # 1.0 is the standard speed
    self.top = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    self.bottom = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    
  def resetPosition(self):
    final_angle = self.bottom.run_until_stalled(self.SPEED, Stop.HOLD, None)
    self.top.hold()
    final_angle = self.bottom.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.bottom.hold()
    final_angle = self.top.run_until_stalled(-self.SPEED, Stop.HOLD, None)
    self.rotate_all(57.5)
    self.top.reset_angle(180)
    self.bottom.reset_angle(180)
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
  def rotate_all(self,deg):
    self.top.run_target(self.SPEED, (self.top.angle() + deg), Stop.HOLD, False)  
    self.bottom.run_target(self.SPEED, (self.bottom.angle() + deg), Stop.HOLD, True)
     
    
  def move_to(self, x, y):
    xPrime,yPrime = self.get_x_y()
    print("primes",xPrime, yPrime)
    b = math.sqrt(((x-xPrime)**2) + ((y-yPrime)**2))
    gamma = rad_to_deg(math.asin(((y-yPrime)/b)))
    print("b",b, "y", y)
    beta =  rad_to_deg(math.acos((1-((b**2)/(2*self.length**2)))))
    alpha = (180 - beta)/2
    print("gamma", gamma, "\nalpha",alpha,"\nbeta",beta)

    theta1 = alpha + gamma
    theta2 = theta1 + beta
    print(self.top.angle(), self.bottom.angle())
    print("theta1", theta1, "\ntheta2", theta2)
    self.wait_motor()
    self.top.run_target(self.SPEED, theta2, Stop.HOLD, False)  
    self.bottom.run_target(self.SPEED, theta1, Stop.HOLD, True) 
  
  def wait_motor(self, stop_fn=lambda:False, params=()):
    while not self.top.control.done() or not self.bottom.control.done():
      if stop_fn(*params):
        self.top.brake()
        self.bottom.brake()
    return


    
  def get_x_y(self):
    #Top is theta2 and bot is theta1
    theta2 , theta1 = (self.top.angle(), self.bottom.angle()) 
    theta0 = ((theta2 + theta1)/2) - 90
    r = 2*self.length *math.sin((theta2 + theta1)/2 - theta1)
    return (r*rad_to_deg(math.cos(theta0)),r*rad_to_deg(math.sin(theta0)))

class Problem:
  def __init__(self, start, goal, size, obstacles, obstacle_size=2):
    self.start = start # (x, y, rads from +x)
    self.goal = goal # (x,y)
    self.size = size # (x_size, y_size)

# get angle from x, 
    
# ===============
robot = Robot(80)
robot.resetPosition()
robot.move_to(0, 50)
#print(robot.reset_position())
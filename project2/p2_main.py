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

class Robot:
  def __init__(self, left_wheel_radius, left_friction_bias, left_turn_bias, axle_radius, wheel_wiggle_bias, speed, look_limit):
    self.gear_ratio = 5
    self.speed = self.gear_ratio*speed
    self.left_turn_bias = left_turn_bias
    self.left_friction_bias = left_friction_bias
    self.wheel_wiggle_bias = wheel_wiggle_bias

    # left wheel
    self.lwr = left_wheel_radius/self.gear_ratio
    self.lwc = 2*PI*self.lwr
    self.lm = Motor(Port.B, Direction.COUNTERCLOCKWISE) # left motor

    # right wheel
    self.rwr = left_wheel_radius/(self.gear_ratio*self.left_turn_bias)
    self.rwc = 2*PI*self.rwr
    self.rm = Motor(Port.C, Direction.COUNTERCLOCKWISE) # right motor

    # axle
    self.xr = axle_radius
    self.xc = 2*PI*self.xr

    # looking
    self.look_limit = look_limit
    self.cm = Motor(Port.D, Direction.COUNTERCLOCKWISE) # center motor
    self.ultrasonic = UltrasonicSensor(Port.S2)
    self.color = ColorSensor(Port.S1)

  # units: radians and meters
  def turn_left(self, rad, arc_radius=0):
    req_l_axle_rot = rad*(arc_radius-self.xr) # arclength of inner circle
    req_r_axle_rot = rad*(arc_radius+self.xr) # arclength of outer circle
    req_l_wheel_rot = 2*PI*(req_l_axle_rot/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*(req_r_axle_rot/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    arc_left_bias = req_l_wheel_rot/req_r_wheel_rot
    if arc_left_bias <= 1.0:  # left motor runs slower
      self.lm.run_target(self.left_friction_bias*self.speed*arc_left_bias, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot))
    else: # right motor runs slower
      self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.left_turn_bias*self.speed/arc_left_bias, rad_to_deg(req_r_wheel_rot))
  def turn_right(self, rad, arc_radius=0):
    self.turn_left(-rad, -arc_radius)
  def move_forward(self, dist):
    if dist == 0: return
    
    req_l_wheel_rot = 2*PI*( dist/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*( dist/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
    self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot))

  def look(self, rad):
    if rad != self.look_limit: # edgecase
      rad = (rad+self.look_limit) % (2*self.look_limit) - self.look_limit # safety equation
    self.cm.run_target(self.speed,rad_to_deg(rad*self.gear_ratio))
  def look_left(self):
    self.look(-self.look_limit)
  def look_forward(self):
    self.look(0)
  def look_right(self):
    self.look(self.look_limit)

  # utility functions

  # test functions
  def test(self):
    self.move_test()
    self.look_test()
  def move_test(self):
    ev3 = EV3Brick()
    ev3.speaker.say("Initializing movement test")
    ev3.speaker.say("Make sure the robot is aligned")
    time.sleep(3)
    ev3.speaker.say("Starting")

    for i in range(4):
      for j in range(4):
        self.move_forward(1*FT)
        self.turn_left(PI/2)
      for j in range(4):
        self.move_forward(1*FT)
        self.turn_right(PI/2)

    ev3.speaker.say("Complete")
  def look_test(self):
    ev3 = EV3Brick()
    ev3.speaker.say("Initializing viewing test")
    ev3.speaker.say("Make sure the sensor assembaly is centered")
    time.sleep(3)
    ev3.speaker.say("Starting")

    for i in range(4):
      self.look_right()
      print(robot.ultrasonic.distance())
      time.sleep(.2)

      self.look_forward()
      print(robot.ultrasonic.distance())
      time.sleep(.2)

      self.look_left()
      print(robot.ultrasonic.distance())
      time.sleep(.2)
    self.look_forward()
    
    ev3.speaker.say("Complete")
# ===============


left_wheel_radius = 0.040095
left_friction_bias = 1.0 # inteded to calibrate differences between L/R motor friction
left_turn_bias = 1.0037 # intended to calibrate to move straight
axle_radius = 0.062
wheel_wiggle_bias = 0.0005 # inteded to calibrate for the lack of movement between forward and backward motions (TODO NOT IMPLEMENTED)
speed = 90.0
look_limit = PI/2
robot = Robot(left_wheel_radius, 
              left_friction_bias, 
              left_turn_bias, 
              axle_radius, 
              wheel_wiggle_bias, 
              speed, 
              look_limit) # calibrated for speed=90

robot.look_test()


  

  


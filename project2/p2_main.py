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
  def __init__(self, left_wheel_radius, left_friction_bias, left_turn_bias, axle_radius, wheel_wiggle_bias, speed):
    gear_ratio = 5

    self.speed = gear_ratio*speed
    self.left_turn_bias = left_turn_bias
    self.left_friction_bias = left_friction_bias
    self.wheel_wiggle_bias = wheel_wiggle_bias

    # left wheel
    self.lwr = left_wheel_radius/gear_ratio
    self.lwc = 2*PI*self.lwr

    # right wheel
    self.rwr = left_wheel_radius/(gear_ratio*self.left_turn_bias)
    self.rwc = 2*PI*self.rwr

    # axle
    self.xr = axle_radius
    self.xc = 2*PI*self.xr

    # motors
    self.lm = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    self.rm = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# units: radians and meters
  def turn_left(self, rad):
    req_axle_rot = rad*self.xr # arclength of axle circumfrence
    req_l_wheel_rot = -2*PI*( (req_axle_rot-self.wheel_wiggle_bias)/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot =  2*PI*( (req_axle_rot-self.wheel_wiggle_bias)/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
    self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot))
  def turn_right(self, rad):
    self.turn_left(-rad)
  def move_forward(self, dist):
    if dist == 0: return
    
    req_l_wheel_rot = 2*PI*( dist/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*( dist/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
    self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot))

# ===============
ev3 = EV3Brick()

left_wheel_radius = 0.040095
left_friction_bias = 1.0 # inteded to calibrate differences between L/R motor friction
left_turn_bias = 1.0037 # intended to calibrate to move straight
axle_radius = 0.062
wheel_wiggle_bias = 0.0005
speed = 90.0
robot = Robot(left_wheel_radius, left_friction_bias, left_turn_bias, axle_radius, wheel_wiggle_bias, speed) # calibrated for speed=90

ev3.speaker.beep()
robot.move_forward(1*FT)
ev3.speaker.beep()

time.sleep(5)

ev3.speaker.beep()
robot.turn_right(15*PI)
ev3.speaker.beep()

time.sleep(5)

ev3.speaker.beep()
robot.move_forward(1*FT)
ev3.speaker.beep()

time.sleep(5)

ev3.speaker.beep()
robot.turn_left(15*PI)
ev3.speaker.beep()


  

  


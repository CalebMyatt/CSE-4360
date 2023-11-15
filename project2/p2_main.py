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
M = 1.0
# =============================================================================
def deg_to_rad(deg):
  return deg*PI/180
def rad_to_deg(rad):
  return rad*180/PI
def feet_to_meters(feet):
  return feet*FT
def meters_to_feet(meters):
  return meters/FT


class Robot:
  def __init__(self, left_wheel_radius, left_friction_bias, left_turn_bias, axle_radius, wheel_wiggle_bias, speed, look_limit):
    self.speed = speed
    self.left_turn_bias = left_turn_bias
    self.left_friction_bias = left_friction_bias
    self.wheel_wiggle_bias = wheel_wiggle_bias

    # left wheel
    self.lwr = left_wheel_radius
    self.lwc = 2*PI*self.lwr
    self.lm = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8,24,40]) # left motor

    # right wheel
    self.rwr = left_wheel_radius/(self.left_turn_bias)
    self.rwc = 2*PI*self.rwr
    self.rm = Motor(Port.C, Direction.COUNTERCLOCKWISE, [8,24,40]) # right motor

    # axle
    self.xr = axle_radius
    self.xc = 2*PI*self.xr

    # looking
    self.look_limit = look_limit
    self.cm = Motor(Port.D, Direction.CLOCKWISE, [8,40]) # center motor
    self.ultrasonic_sensor = UltrasonicSensor(Port.S2)
    # self.color_sensor = ColorSensor(Port.S1)

  # motor functions. Units: rad, meter
  def turn_left(self, rad=PI/2, arc_radius=0):
    req_l_axle_rot = rad*(arc_radius-self.xr) # arclength of inner circle
    req_r_axle_rot = rad*(arc_radius+self.xr) # arclength of outer circle
    req_l_wheel_rot = 2*PI*(req_l_axle_rot/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*(req_r_axle_rot/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    arc_left_bias = req_l_wheel_rot/req_r_wheel_rot
    if arc_left_bias <= 1.0:  # left motor runs slower
      self.lm.run_target(self.left_friction_bias*self.speed*arc_left_bias, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot), wait=False)
    else: # right motor runs slower
      self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.left_turn_bias*self.speed/arc_left_bias, rad_to_deg(req_r_wheel_rot), wait=False)
  def turn_right(self, rad=PI/2, arc_radius=0):
    self.turn_left(-rad, -arc_radius)
  def move_forward(self, dist=1*FT):
    if dist == 0: return
    
    req_l_wheel_rot = 2*PI*( dist/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*( dist/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.left_friction_bias*self.speed, rad_to_deg(req_l_wheel_rot), wait=False)
    self.rm.run_target(self.left_turn_bias*self.speed, rad_to_deg(req_r_wheel_rot), wait=False)

  def look_left(self, rad=PI/2, sensing=False):
    speed = self.speed
    if sensing:
      speed /= 5

    if rad != self.look_limit: # edgecase
      rad = (rad+self.look_limit) % (2*self.look_limit) - self.look_limit # safety equation
    self.cm.run_target(speed, rad_to_deg(rad), wait=False)
  def look_right(self, rad=PI/2, sensing=False):
    self.look_left(-rad, sensing)
  def look_forward(self):
    self.look_left(0)

  def wait_for_motors(self):
    while not self.lm.control.done() or not self.rm.control.done() or not self.cm.control.done():
      pass
  # utility functions
  def look_sweep(self):
    # this is a simple implementation, but this function is very important.
    # TODO: do multiple sweaps and get the average of multiple. Left->right and right->left.
    # TODO: check for outlying data.
    self.look_left()
    self.wait_for_motors()
    self.look_right(sensing=True)

    thetas = [self.cm.angle()]
    dists = [self.ultrasonic_sensor.distance()]
    
    while not self.cm.control.done():
      prev_dist = dists[-1]
      new_dist = self.ultrasonic_sensor.distance()
      if prev_dist != new_dist:
        new_theta = deg_to_rad(self.cm.angle())
        thetas.append(new_theta)
        dists.append(new_dist)
        print(new_theta, new_dist)

    return (thetas, dists)
  def recenter(self):
    self.look_forward()
    self.wait_for_motors()
  
  def turn_to_closest(self):
    (thetas, dists) = self.look_sweep()
    min_dist = min(dists)
    theta = thetas[dists.index(min_dist)]
    print(theta, min_dist)

    self.turn_left(theta)
    self.wait_for_motors()
  def turn_to_furthest(self):
    (thetas, dists) = self.look_sweep()
    max_dist = max(dists)
    theta = thetas[dists.index(max_dist)]
    print(theta, max_dist)

    self.turn_left(theta)
    self.wait_for_motors()


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
      self.look_forward()
      self.look_left()
      self.look_forward()
    
    ev3.speaker.say("Complete")
# ===============
def main():
  left_wheel_radius = 0.040095
  left_friction_bias = 1.0 # inteded to calibrate differences between L/R motor friction
  left_turn_bias = 1.0037 # intended to calibrate to move straight
  axle_radius = 0.062
  wheel_wiggle_bias = 0.0005 # inteded to calibrate for the lack of movement between forward and backward motions (TODO NOT IMPLEMENTED)
  speed = 90.0
  look_limit = PI/2 # don't change this unless the default values for look functions are set to this value somehow
  robot = Robot(left_wheel_radius, 
                left_friction_bias, 
                left_turn_bias, 
                axle_radius, 
                wheel_wiggle_bias, 
                speed, 
                look_limit) # calibrated for speed=90

  robot.turn_to_closest()
  robot.recenter()

if __name__ == '__main__':
  main()

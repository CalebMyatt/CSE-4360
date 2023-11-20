#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

PI = 3.1415926535897932384626
FT = 0.3048
M = 1.0
# TODO: 
# Refactor (Combine all movement with wait_for_motors function as a bollean)
# Landon create and interface and Caleb and Antohony create the solver

# =============================================================================
def deg_to_rad(deg):
  return deg*PI/180
def rad_to_deg(rad):
  return rad*180/PI

def feet_to_meters(feet):
  return feet*FT
def meters_to_feet(meters):
  return meters/FT
def millimeters_to_meters(millimeters):
  return millimeters/1000

def direction_to_cartesion(direction):
  if direction%4==0:
    return (0,1)
  if direction%4==1:
    return (1,0)
  if direction%4==2:
    return (0,-1)
  if direction%4==3:
    return (-1,0)

class Tile:
  def __init__(self, north=False, west=False, south=False, east=False):
    self.walls = [north, west, south, east]

    self.visited = True
  
  def __str__(self):
    north = self.walls[0]
    west = self.walls[1]
    south = self.walls[2]
    east = self.walls[3]
    return 'n:'+str(north)+' w:'+str(west)+' s:'+str(south)+' e:'+str(east)
  def __repr__(self):
    return self.__str__()

class Robot:
  def __init__(self, left_wheel_radius, left_friction_bias, left_turn_bias, axle_radius, wheel_wiggle_bias, speed, look_limit):
    self.left_speed = left_friction_bias*speed
    self.right_speed = left_turn_bias*speed
    self.center_speed = speed
    # self.wheel_wiggle_bias = wheel_wiggle_bias

    # left side
    self.lwr = left_wheel_radius
    self.lwc = 2*PI*self.lwr
    self.lm = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8,24,40]) # left motor
    self.ls = TouchSensor(Port.S1)

    # right side
    self.rwr = left_wheel_radius/(left_turn_bias)
    self.rwc = 2*PI*self.rwr
    self.rm = Motor(Port.C, Direction.COUNTERCLOCKWISE, [8,24,40]) # right motor
    self.rs = TouchSensor(Port.S2)

    # axle
    self.xr = axle_radius
    self.xc = 2*PI*self.xr

    # looking
    self.look_limit = look_limit
    self.cm = Motor(Port.D, Direction.CLOCKWISE, [8,40]) # center motor
    self.ultrasonic_sensor = UltrasonicSensor(Port.S4)
    self.color_sensor = ColorSensor(Port.S3)

  # motor functions. Units: rad, meter ========================================
  def turn_left(self, rad=PI/2, arc_radius=0):
    req_l_axle_rot = rad*(arc_radius-self.xr) # arclength of inner circle
    req_r_axle_rot = rad*(arc_radius+self.xr) # arclength of outer circle
    req_l_wheel_rot = 2*PI*(req_l_axle_rot/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*(req_r_axle_rot/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    arc_left_bias = req_l_wheel_rot/req_r_wheel_rot
    if arc_left_bias <= 1.0:  # left motor runs slower
      self.lm.run_target(self.left_speed*arc_left_bias, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.right_speed, rad_to_deg(req_r_wheel_rot), wait=False)
    else: # right motor runs slower
      self.lm.run_target(self.left_speed, rad_to_deg(req_l_wheel_rot), wait=False)
      self.rm.run_target(self.right_speed/arc_left_bias, rad_to_deg(req_r_wheel_rot), wait=False)
  def turn_right(self, rad=PI/2, arc_radius=0):
    self.turn_left(-rad, -arc_radius)
  def move_forward(self, dist=1*FT):
    if dist == 0: return
    
    req_l_wheel_rot = 2*PI*( dist/self.lwc ) # required rotation of left wheel
    req_r_wheel_rot = 2*PI*( dist/self.rwc ) # required rotation of right wheel

    self.lm.reset_angle(0)
    self.rm.reset_angle(0)
    self.lm.run_target(self.left_speed, rad_to_deg(req_l_wheel_rot), wait=False)
    self.rm.run_target(self.right_speed, rad_to_deg(req_r_wheel_rot), wait=False)

  def look_left(self, rad=PI/2, sensing=False):
    speed = self.center_speed
    if sensing:
      speed /= 5

    if rad != self.look_limit: # edgecase
      rad = (rad+self.look_limit) % (2*self.look_limit) - self.look_limit # safety equation
    self.cm.run_target(speed, rad_to_deg(rad), wait=False)
  def look_right(self, rad=PI/2, sensing=False):
    self.look_left(-rad, sensing)
  def look_forward(self):
    self.look_left(0)

  def wait_for_motors(self, stop_fn=lambda:False, params=()):
    while not self.lm.control.done() or not self.rm.control.done() or not self.cm.control.done():
      if stop_fn(*params):
        self.lm.brake()
        self.rm.brake()
        self.cm.brake()
        return
  # sensors ===================================================================
  def on_fire(self):
    if self.color_sensor.color() == Color.WHITE:
      return True
    return False
  def no_fire(self):
    return not self.on_fire()
  # speaking ==================================================================
  def alert(self, message="Alert"):
    ev3 = EV3Brick()
    ev3.speaker.say(message)
  # utility functions =========================================================
  def align_all(self):
    self.align_back()

    self.turn_left()
    self.wait_for_motors()

    self.align_back()

    self.turn_right()
    self.wait_for_motors()

    self.align_back()
  def align_back(self):
    self.look_right()

    while not self.ls.pressed() or not self.rs.pressed():
      if self.ls.pressed():
        self.lm.hold()
      else:
        self.lm.run(-self.left_speed)

      if self.rs.pressed():
        self.rm.stop()
      else:
        self.rm.run(-self.right_speed)
      wait(100)
    self.rm.hold()
    self.lm.hold()

    self.look_forward()
    self.move_forward(.5*FT-.075*M)
    self.wait_for_motors()
  
  def look_around(self, realign_threshold=.3*FT):
    self.look_left()
    self.wait_for_motors()
    l_dist = millimeters_to_meters(self.ultrasonic_sensor.distance())
    if l_dist <= realign_threshold:
      self.turn_right()
      self.wait_for_motors()
      self.align_back()
      self.turn_left()
      self.wait_for_motors()
      return self.look_around()

    self.look_right()
    self.wait_for_motors()
    r_dist = millimeters_to_meters(self.ultrasonic_sensor.distance())
    if r_dist <= realign_threshold:
      self.turn_left()
      self.wait_for_motors()
      self.align_back()
      self.turn_right()
      self.wait_for_motors()
      return self.look_around()

    self.look_forward()
    self.wait_for_motors()
    f_dist = millimeters_to_meters(self.ultrasonic_sensor.distance()) 
    if f_dist <= realign_threshold-millimeters_to_meters(67):
      self.turn_left(PI)
      self.wait_for_motors()
      self.align_back()
      self.turn_right(PI)
      self.wait_for_motors()
      return self.look_around()

    return (l_dist, f_dist, r_dist)
  def get_walls(self, facing, behind=True, threshold=.7*FT):
    l = []
    for dist in self.look_around():
      if dist < threshold:
        l.append(True)
      else:
        l.append(False)

    l += [behind] # left, forward, right, back
    n,e,s,w = l[(facing+1)%4:] + l[:(facing+1)%4]  # north, east, south, west
    return Tile(n,w,s,e)
  
  def go_to_nearest_unvisited_tile(self, map, location, facing, limit=10):
    depth = 1
    result = None
    while result == None:
      if depth == limit:
        map.clear()
        self.alert("I didn't find a fire")
        self.align_all()
        map[location] = self.get_walls(facing, True)
        depth = 0

      result = self.find_nearest_unvisited_tile(map, location, facing, depth, [])

      depth += 1

    for action in result[0]:
      action(self)
      self.wait_for_motors(lambda: not self.no_fire())
    self.turn_left(PI)
    self.wait_for_motors(lambda: not self.no_fire())
    self.turn_right(PI)
    self.wait_for_motors(lambda: not self.no_fire())

    return (result[1], result[2])
  def find_nearest_unvisited_tile(self, map, location, facing, depth, tested):
    if location in tested: # if visited before
      return None
    elif depth <= 0: # return no result
      return None
    
    if not location in map.keys(): # found unknown
      return ([],location,facing)
    elif not map[location].visited: # found unvisited tile
      return ([],location,facing)
    tested += [location]

    x = location[0]
    y = location[1]

    # forward
    rotation = facing
    if not map[location].walls[rotation%4]:
      x_diff, y_diff = direction_to_cartesion(rotation)
      result = self.find_nearest_unvisited_tile(map, (x+x_diff, y+y_diff), rotation, depth-abs(facing-rotation), tested)
      if result != None:
        return ([Robot.move_forward] + result[0], result[1], result[2])
      
    # left
    rotation = facing+1
    if not map[location].walls[rotation%4]:
      x_diff, y_diff = direction_to_cartesion(rotation)
      result = self.find_nearest_unvisited_tile(map, (x+x_diff, y+y_diff), rotation, depth-abs(facing-rotation), tested)
      if result != None:
        return ([Robot.turn_left, Robot.move_forward] + result[0], result[1], result[2])

    # right
    rotation = facing-1
    if not map[location].walls[rotation%4]:
      x_diff, y_diff = direction_to_cartesion(rotation)
      result = self.find_nearest_unvisited_tile(map, (x+x_diff, y+y_diff), rotation, depth-abs(facing-rotation), tested)
      if result != None:
        return ([Robot.turn_right, Robot.move_forward] + result[0], result[1], result[2])

    # back
    rotation = facing+2
    if not map[location].walls[rotation%4]:
      x_diff, y_diff = direction_to_cartesion(rotation)
      result = self.find_nearest_unvisited_tile(map, (x+x_diff, y+y_diff), rotation, depth-abs(facing-rotation), tested)
      if result != None:
        return ([Robot.turn_left, Robot.turn_left, Robot.move_forward] + result[0], result[1], result[2])

    return None

  def find_fire(self):
    location = (0,0)
    facing = 0
    map = {}

    behind = True # there is a wall behind us initally
    while self.no_fire():
      map[location] = self.get_walls(facing, behind)
      behind = False

      location, facing = self.go_to_nearest_unvisited_tile(map, location, facing)

# ===============
def main():
  left_wheel_radius = 0.040095
  left_friction_bias = 1.0000 # inteded to calibrate differences between L/R motor friction. Higher value => left wheel turns faster over same distance
  left_turn_bias = 0.995 # intended to calibrate to move straight. Higher value => turns left more
  axle_radius = 0.0635
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
  
  robot.align_all()
  robot.find_fire()
  robot.alert("There is a fire. Please put it out.")


if __name__ == '__main__':
  main()

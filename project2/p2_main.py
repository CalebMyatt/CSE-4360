#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

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
  def turn_optimal(self, rad, arc_radius=0):
    rad = (rad+PI)%(2*PI)-PI
    if rad > 0:
      self.turn_left(rad, arc_radius)
    else:
      self.turn_right(-rad, arc_radius)
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

  def wait_for_motors(self, motors=[], stop_fn=lambda:False, params=()):
    if len(motors) == 0:
      motors = [self.lm, self.cm, self.rm]

    while any(map(lambda x: not x.control.done(), motors)): # while any motor is running
      if stop_fn(*params): # if stopping condition, stop
        for motor in motors:
          motor.brake()
        return
      wait(20) # aka 50Hz. not needed much
      
  # sensors ===================================================================
  def sees_color(self, color):
    if self.color_sensor.color() == color:
      return True
    return False
  
  def observe_left(self, rad=PI/2):
    self.look_left(rad)
    self.wait_for_motors()
    return millimeters_to_meters(self.ultrasonic_sensor.distance())
  def observe_right(self, rad=PI/2):
    self.look_right(rad)
    self.wait_for_motors()
    return millimeters_to_meters(self.ultrasonic_sensor.distance())
  def observe_forward(self):
    self.look_forward()
    self.wait_for_motors()
    return millimeters_to_meters(self.ultrasonic_sensor.distance())
  def observe_around(self):
    r_dist = self.observe_right()
    self.turn_left()
    l_dist = self.observe_forward()
    b_dist = self.observe_left()
    self.turn_right()
    f_dist = self.observe_forward()

    self.look_right() # prepare for next call

    return (f_dist, l_dist, b_dist, r_dist)
  # speaking ==================================================================
  def alert(self, message="Alert"):
    ev3 = EV3Brick()
    ev3.speaker.say(message)
  # utility functions =========================================================
  def align_back(self, rear_calibration=.075*M):
    self.look_right()

    while not self.ls.pressed() or not self.rs.pressed():
      if self.ls.pressed():
        self.lm.stop()
      else:
        self.lm.run(-self.left_speed)

      if self.rs.pressed():
        self.rm.stop()
      else:
        self.rm.run(-self.right_speed)

    self.look_forward()
    self.move_forward(.5*FT-rear_calibration)
    self.wait_for_motors()

class Solver:
  def solve(robot):
    Solver.align_all(robot)
    Solver.find_fire(robot)
    robot.alert("There is a fire. Please put it out.")

  def align_all(robot: Robot, rotations=3):
    robot.align_back()

    for i in range(rotations):
      right_left = (2*i)%2-1

      robot.turn_optimal(right_left * PI/2)
      robot.wait_for_motors()
      robot.align_back()
  def realign(robot: Robot, distances, threshold = .4*FT):
    misalignments = map(lambda dist: dist<threshold, distances)
    for (i, too_close) in enumerate(misalignments):
      if too_close:
        robot.turn_optimal(i * PI/2 + PI) # turn away
        robot.align_back()
        robot.turn_optimal(i * PI/2) # turn back
    
    if any(misalignments):
      return Solver.realign(robot, robot.observe_around())
    else:
      return distances
  def get_tile(robot: Robot, facing: float, threshold=.7*FT):
    distances = robot.observe_around()
    distances = Solver.realign(robot, distances) # realigns if needed

    relative_walls = list(map(lambda dist: dist < threshold, distances))

    shift = int(-facing/(PI/2)) % 4
    walls = relative_walls[shift:] + relative_walls[:shift]  # north, east, south, west
    return Tile(*walls)
  
  def visit_next_tile(robot: Robot, map, location, facing, depth_limit=25):
    # iterative deepening search
    depth = 1
    actions = None
    while actions == None:
      if depth == depth_limit: # if no solution found
        map.clear() # delete map
        depth = 0
        robot.alert("No fire found, looking again.")

      actions, location, facing = Solver.find_next_tile(map, location, facing, depth, {})
      depth += 1

    # executing route
    for (action, params) in actions:
      action(robot, *params)
      robot.wait_for_motors(lambda: not robot.sees_color(Color.BLACK))

    return (location, facing)
  def find_next_tile_direction(rotation, facing, map, location, depth, tested):
    x = location[0]
    y = location[1]
  
    if not map[location].walls[rotation%4]: # if no wall in the way
      x_diff, y_diff = direction_to_cartesion(rotation)
      depth = depth-abs(int((facing-rotation)/(PI/2)))

      actions, location, facing = Solver.find_next_tile(map, (x+x_diff, y+y_diff), facing+rotation, depth, tested)
      if actions != None: # if solution found
        return ([(Robot.turn_optimal,(rotation)), (Robot.move_forward,())] + actions, location, facing)
    return (None, None, None)
  def find_next_tile(map, location, facing, depth, tested):
    if depth <= 0: # return no result
      return (None, None, None)
    
    if location in tested.keys(): # if visited...
      prev_depth = tested[location]
      if prev_depth < depth: # ...with lower depth before
        return (None, None, None)
      else:
        tested[location] = depth
    tested.update({location: depth})
    
    if not location in map.keys(): # found undiscovered node. return route
      return ([],location,facing)
    elif not map[location].visited: # found unvisited tile. return route
      return ([],location,facing)

    # test all four directions
    for i in range(4):
      actions, location, facing = Solver.find_next_tile_direction(i*PI/2, facing, map, location, depth, tested)
      if actions != None:
        return (actions, location, facing)

    return (None, None, None)

  def find_fire(robot: Robot):
    location = (0,0)
    facing = 0.0
    map = {}

    while robot.sees_color(Color.BLACK):
      map[location] = Solver.get_tile(robot, facing)
      location, facing = Solver.visit_next_tile(map, location, facing)
  
# ===============
def main():
  # TODO change all units to degrees and milimeters

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
  
  Solver.solve(robot)

if __name__ == '__main__':
  main()

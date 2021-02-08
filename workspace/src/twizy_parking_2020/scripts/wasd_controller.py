#!/usr/bin/env python

# This node reads the WASD keys, increments/decrements the speed/angle and publishes this to the topic "controls". It is used to manually drive the car.

import rospy
from twizy.msg import car_control

# A class for reading the keybard, i copied this from the internet and have no idea how it works.
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

# main loop, creates node and publishes the speed and angle to "controls"
if __name__ == '__main__':
  # initializes the node and sets up the publishing settings. what everything means can be found in the ros tutorials
  rospy.init_node('user_listener', anonymous=True)
  pub = rospy.Publisher('controls', car_control, queue_size=5)
  rate = rospy.Rate(60) # 10hz
  # initializes the custom message and makes sure the starting values are 0
  msg = car_control()
  angle = 0
  speed = 0

  print("Press 'c' to exit this program")

  while not rospy.is_shutdown():

	# reads keyboard and increments/decrements the angle or speed value depending on the input
      c = getch()
      if c == 'c':
        rospy.signal_shutdown("byebye")
      elif c == 'a':
        angle -= 5
      elif c == 'd':
        angle += 5
      elif c == 'w':
        speed += 1
      elif c == 's':
        speed -= 1
      
	# limits the values as the control_listener cannot handle improper values in the message
      if angle < -40:
        angle = -40
      if angle > 40:
        angle = 40
      if speed > 5:
        speed = 5
      if speed < -5:
        speed = -5

      print("speed:", speed, "angle", angle)

      msg.angle = angle
      msg.speed = speed
      pub.publish(msg)
      rate.sleep()

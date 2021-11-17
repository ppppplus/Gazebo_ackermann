#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
u/j : increase/decrease speed by 10% 
i/k : increase/decrease steering angle by 10%
anything else : stop
CTRL-C to quit
---------------------------
"""

keyBindings = {
  'q':(1,1),
  'w':(1,0),
  'e':(1,-1),
  'a':(0,1),
  's':(0,0),
  'd':(0,-1),
  'z':(-1,1),
  'x':(-1,0),
  'c':(-1,-1)
}

velcontrolBindings = {
  'u':(1.1,1),
  'j':(0.9,1),
  'i':(1,1.1),
  'k':(1,0.9)
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 1
turn = 0.4
MAXTURN = 0.58

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
  rospy.init_node('keyop')
  print(banner)
  x = 0
  th = 0
  status = 0

  try:
    while(1):
      key = getKey()
      if key in keyBindings.keys():
        x = keyBindings[key][0]
        th = keyBindings[key][1]
      elif key in velcontrolBindings.keys():
        speed_mul = velcontrolBindings[key][0]
        turn_mul = velcontrolBindings[key][1]
        speed = speed*speed_mul
        turn = turn*turn_mul
        if turn > MAXTURN:
          turn = MAXTURN
          print("The set value of steering angle has reached the maximum!")
        print(vels(speed,turn))
      else:
        x = 0
        th = 0
        if (key == '\x03'):
          break
      msg = AckermannDriveStamped()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = "base_link"
      msg.drive.speed = x*speed
      msg.drive.acceleration = 1
      msg.drive.jerk = 1
      msg.drive.steering_angle = th*turn
      msg.drive.steering_angle_velocity = 1

      pub.publish(msg)

  except:
    print("error")

  finally:
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.speed = 0
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#!/usr/bin/env python
import rospy
import os
import sys
import math
class ModelEditer:
    def __init__(self, name, x, y, theta):
        self.name = name
        self.x = str(x)
        self.y = str(y)
        self.theta = float(theta)
        self.ox = 0
        self.oy = 0
        self.oy = 0
        self.ow = 0
        self.RPY2Quar()
        self.set_model_state()
    def set_model_state(self):
        os.system("rosservice call /gazebo/set_model_state '{model_state: {model_name: " + self.name +
        ", pose: {position: {x: " + x + ", y: " + y+ " ,z: 0.5"
        "}, orientation: {x: " + str(self.ox) + ", y: " + str(self.oy) + ", z: " + str(self.oz) + ", w: " + str(self.ow) + "} } , reference_frame: world } }'")
    def RPY2Quar(self):
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        cp = math.cos(0)
        sp = math.sin(0)
        cr = math.cos(0)
        sr = math.sin(0)
        
        self.ow = cy * cp * cr + sy * sp * sr
        self.ox = cy * cp * sr - sy * sp * cr
        self.oy = sy * cp * sr + cy * sp * cr
        self.oz = sy * cp * cr - cy * sp * sr
        
if __name__ == '__main__':
    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    theta = sys.argv[4]
    ModelEditer(robot_name, x, y, theta)

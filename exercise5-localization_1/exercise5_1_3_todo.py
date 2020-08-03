#!/usr/bin/env python
# _*_ coding: utf-8 _*_
import sys
import math
import time
import signal

import numpy as np

from cyber_py3 import cyber
from modules.localization.proto.localization_pb2 import localization
from modules.localization.proto.localization_pb2 import pos
from modules.sensors.proto.nooploop_pb2 import TagFrame
from modules.sensors.proto.sensors_pb2 import Gyro 

from modules.control.proto.control_pb2 import Control_Command
from modules.control.proto.chassis_pb2 import Chassis

def eular_iteration_func(x_last, u, delta_t):
    delta_x = 0.1 * x_last + u
    x_now = x_last + delta_t * delta_x
    return x_now


class Exercise5_2(object):
    def __init__(self, node):
        self.node = node
        self.pos = pos()

        self.x_pos = 1.1
        self.theta = 0
        self.y_pos = 2.8
        self.offset = 0.1
        self.steer = 0
        self.L = 0.35
        self.gyroz = 0
        self.node.create_reader("/geek/gyro", Gyro, self.gyrocallback)
        self.writer = self.node.create_writer("/geek/uwb/localization", pos)
        self.node.create_reader("/chassis", Chassis, self.chassiscallback)
        self.node.create_reader("/control", Control_Command, self.commandcallback)
        signal.signal(signal.SIGINT, self.sigint_handler)
        signal.signal(signal.SIGHUP, self.sigint_handler)
        signal.signal(signal.SIGTERM, self.sigint_handler)
        self.is_sigint_up = False
        while True:
            time.sleep(0.05)
            if self.is_sigint_up:
                print("Exit")
                self.is_sigint_up = False
                sys.exit()

    def sigint_handler(self, signum, frame):
            #global is_sigint_up
        self.is_sigint_up = True
        print("catched interrupt signal!")

    def gyrocallback(self, gyro):
        self.gyroz = -gyro.gyro.z * 1.0 - 0.01
        self.theta += 0.02 * self.gyroz

    def commandcallback(self, command):
        self.steer = command.steer_angle * 3.14 / 180

    def chassiscallback(self, chassis):
        velocity = chassis.speed
        #steer_angle = self.offset + chassis.steer_angle * 3.14 / 180
        #print(steer_angle) 
        #delta_theta = self.gyroz 
        #self.theta += 0.05 * delta_theta
        
        delta_x = velocity * math.cos(self.theta)
        delta_y = velocity * math.sin(self.theta)
        
        self.x_pos += 0.05 * delta_x
        self.y_pos += 0.05 * delta_y
        self.pos.x = self.x_pos
        self.pos.y = self.y_pos
        self.pos.z = 0
        self.pos.yaw = self.theta + 2 * 3.14159 / 2
        self.writer.write(self.pos)

if __name__ == '__main__':
    cyber.init()

    exercise_node = cyber.Node("localization_node")
    
    exercise = Exercise5_2(exercise_node)
    exercise_node.spin()
    cyber.shutdown()



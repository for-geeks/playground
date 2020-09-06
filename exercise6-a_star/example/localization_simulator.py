#!/usr/bin/env python
# _*_ coding: utf-8 _*_
import sys
import math
import time
import numpy as np
import signal

from cyber_py3 import cyber
from modules.control.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import localization
from modules.localization.proto.localization_pb2 import pos
from modules.control.proto.control_pb2 import Control_Command


class Simulator(object):
    def __init__(self, node):
        self.node = node
        self.alpha = 0.01
        self.maxTimes = 100
        self.x = np.array([0.725, 0.7])
        # base station matrix

        self.xi = np.array([])
        self.yi = np.array([])
        # distance
        self.di = np.array([])
        #self.di = np.array([8, 9.303, 0, 4.75])

        self.localization = localization()
        self.pos = pos()
        self.vel_head = 0
        self.yaw = -3.1415926
        self.direction = 0
        self.x_now = 0.5
        self.y_now = 2.2
        self.velocity = 0
        self.accelorator = 0
        self.chassis = Chassis()
        self.node.create_reader("/control", Control_Command,
                                self.controlcallback)

        self.writer_pos = self.node.create_writer("/geek/uwb/localization",
                                                  pos)
        self.writer_chassis = self.node.create_writer("/chassis", Chassis)

        signal.signal(signal.SIGINT, self.sigint_handler)
        signal.signal(signal.SIGHUP, self.sigint_handler)
        signal.signal(signal.SIGTERM, self.sigint_handler)
        self.is_sigint_up = False
        self.pos.x = 0.5
        self.pos.y = 2.4
        self.pos.z = 0
        self.pos.yaw = self.yaw

        while True:
            time.sleep(0.05)
            self.writer_chassis.write(self.chassis)
            self.writer_pos.write(self.pos)
            if self.is_sigint_up:
                print("Exit")
                self.is_sigint_up = False
                sys.exit()

    def sigint_handler(self, signum, frame):
        #global is_sigint_up
        self.is_sigint_up = True
        print("catched interrupt signal!")

    def controlcallback(self, control):
        if (control.throttle > 10):
            self.accelorator = (control.throttle - 10) * 0.03
        elif control.throttle > 0:
            self.accelorator = 0
        elif control.throttle < 0:
            self.accelorator = control.throttle * 0.03

        dt = 0.05
        self.velocity += (self.accelorator - 2 * self.velocity) * dt
        self.yaw += self.velocity * (control.steer_angle * 2 *
                                     3.14159265358979 / 180) * dt
        self.x_now -= self.velocity * math.cos(self.yaw) * dt
        self.y_now -= self.velocity * math.sin(self.yaw) * dt
        self.pos.x = self.x_now
        self.pos.y = self.y_now
        self.pos.z = 0
        self.pos.yaw = self.yaw
        self.chassis.speed = self.velocity


if __name__ == '__main__':
    cyber.init()

    exercise_node = cyber.Node("simulation_node")

    exercise = Simulator(exercise_node)
    exercise_node.spin()
    cyber.shutdown()

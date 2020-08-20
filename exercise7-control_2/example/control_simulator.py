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
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point
from modules.control.proto.control_pb2 import Control_Command
import random


class Simulator(object):
    def __init__(self, node):
        self.node = node
        self.a = []
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
        self.writer_global = self.node.create_writer("/planning/global_trajectory",
                                              Trajectory)
        self.writer_dwa = self.node.create_writer("/planning/dwa_trajectory",
                                              Trajectory)

        signal.signal(signal.SIGINT, self.sigint_handler)
        signal.signal(signal.SIGHUP, self.sigint_handler)
        signal.signal(signal.SIGTERM, self.sigint_handler)
        self.is_sigint_up = False
        self.pos.x = 0.5
        self.pos.y = 2.4
        self.pos.z = 0
        self.offset = random.random() * 3 + 4
        self.pos.yaw = self.yaw
        self.global_path = Trajectory()
        self.local_path = Trajectory()
        point_xy = Point()
        i = 48
        point_xy.y = 48
        point_xy.x = 122
        self.global_path.point.append(point_xy)
        for j in range(122, 1130):
            self.a.append([i, j])
        point_xy.y = 48
        point_xy.x = 1130
        self.global_path.point.append(point_xy)
		

        for i in range(1132, 1231):
            self.a.append([i - 1132 + 48,i])

        point_xy.y = i - 1132 + 48
        point_xy.x = i
        self.global_path.point.append(point_xy)

        for i in range(145, 643):
            self.a.append([i,1228])
        point_xy.y = i
        point_xy.x = 1228
        self.global_path.point.append(point_xy)

        for i in range(1228, 85, -1):
            self.a.append([640,i])
	
        point_xy.y = 640
        point_xy.x = i
        self.global_path.point.append(point_xy)

        for i in range(653, 87, -1):
            self.a.append([i,87])
		
        point_xy.y = i
        point_xy.x = 87
        self.global_path.point.append(point_xy)
        
        for i in range(85, 122):
            self.a.append([85 - (i - 85), i])	
        point_xy.y = 85 - (i - 85)
        point_xy.x = i
        self.global_path.point.append(point_xy)

        self.writer_global.write(self.global_path)
        while True:
            time.sleep(0.05)
            self.writer_global.write(self.global_path)
            self.writer_chassis.write(self.chassis)
            self.writer_pos.write(self.pos)
            near_index = self.findneaest_point()
            self.local_path = Trajectory()
            self.generate_local_path(self.local_path, near_index)
            self.writer_dwa.write(self.local_path)
            if self.is_sigint_up:
                print("Exit")
                self.is_sigint_up = False
                sys.exit()
    def rotate(self, x, y, yaw):
        out_x = x * math.cos(yaw) + y * math.sin(yaw)
        out_y = y * math.cos(yaw) - x * math.sin(yaw)
        return [out_x, out_y]

    def index_calcu(self, index, a):
        if (index >= len(a)):
            return index % len(a)
        else:
            return index

    def find_dist(self, ptr, pos):
        return (ptr[1] / 144.9 - pos.x)**2 + (ptr[0] / 144.9 - pos.y)**2

    def generate_local_path(self, local_path, start_index):  
        point_xy = Point()
        for i in range(0, 100, 5):
            p = self.a[self.index_calcu(start_index + i, self.a)]    
            rot_p = self.rotate(p[1] / 144.9 - self.pos.x, p[0]/144.9 - self.pos.y, self.pos.yaw)
            point_xy.x = -rot_p[0] + 0.07
            point_xy.y = rot_p[1]
            self.local_path.point.append(point_xy)

    def findneaest_point(self):
        shortest_dist = self.find_dist(self.a[0], self.pos)
        index_short = 0
        index_now = 0
        npxy = []
        for pt in self.a:
            dist = self.find_dist(pt, self.pos)
            index_now = index_now + 1
            if (dist < shortest_dist):
                shortest_dist = dist
                index_short = index_now
                npxy = pt 
        #print (npxy)
        return index_short


    def sigint_handler(self, signum, frame):
        #global is_sigint_up
        self.is_sigint_up = True
        print("catched interrupt signal!")

    def controlcallback(self, control):
        
        self.accelorator = control.throttle
        if (control.throttle > self.offset):
            self.accelorator = (control.throttle) * 0.33 * 0.5 + self.accelorator * 0.5
        elif control.throttle > 0:
            self.accelorator = self.accelorator * 0.5 
            if self.velocity > 0.05:
                self.accelorator = self.accelorator * 0.5 + 0.5 * control.throttle * 0.33 * (control.throttle / self.offset)
        elif control.throttle < 0:
            self.accelorator = self.accelorator * 0.5 + 0.5 * control.throttle * 0.33

        dt = 0.05
        self.velocity += (self.accelorator * 0.25 - 2 * self.velocity) * dt
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

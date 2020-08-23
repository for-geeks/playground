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
from modules.sensors.proto.nooploop_pb2 import TagFrame
from modules.sensors.proto.sensors_pb2 import Gyro 

class Exercise(object):
    def __init__(self, node):
        self.node = node
        self.alpha = 0.01
        self.maxTimes = 100
        self.x = 2.5;
        self.y = 4.5
        # base station matrix
        self.base_x = np.array([3.3, 3.3, 5.5, 5.5])
        self.base_y = np.array([1.1, 3.38, 1.1, 3.38])
        
        self.pos = pos()
        
        self.node.create_reader("/geek/uwb/pose", TagFrame, self.tagcallback)

        self.writer = self.node.create_writer("/geek/uwb/localization", pos)

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

    def tagcallback(self, tag):
        distances_now = []
        for k in range (0, 4):
            if tag.dis[k].distance == 1:
                continue
            distances_now.append(tag.dis[k].distance)

        self.calcu_loss_fun(self.x, self.y, self.maxTimes, self.alpha, self.base_x, self.base_y, distances_now)
        #print self.vel_head
            
        
    # 定义函数f(x)
     # 定义函数f(x)
    def problem(self, x, y, base_x, base_y, distance):
        '''
	学员需要在此实现problem函数，使得能够计算给定xy时，单个方程的误差。
	'''
        return (x - base_x) ** 2 +  (y - base_y)**2 - distance**2  
    
    #定义损失函数
    def loss_fun(self, x, y, base_x, base_y, distances):
        '''
        学员需要在此实现loss_fun函数, 此方程调用了problem函数，用于求解四个方程的误差之和
        '''
        sum_err = 0;
        for i in range(0, len(base_x)):
            sum_err += math.fabs((self.problem(x, y, base_x[i], base_y[i], distances[i]) - 0))
        return sum_err


    def slope_fx(self, x, y, base_x, base_y, distances):
        d = 0.01;
        J1 = (self.loss_fun(x + d, y, base_x, base_y, distances) - self.loss_fun(x - d, y, base_x, base_y, distances)) / (2.0*d)
        J2 = (self.loss_fun(x, y + d, base_x, base_y, distances) - self.loss_fun(x, y - d, base_x, base_y, distances)) / (2.0*d)
        return [J1, J2]
    
    def calcu_loss_fun(self, x, y, maxTimes, alpha, base_x, base_y, distances):
        
        '''
        学员需要在此实现calcu_loss_fun函数, 完成maxtimes次迭代。
        '''
        for i in range(maxTimes):
            ret = self.slope_fx(x, y, base_x, base_y, distances)
            x = x - ret[0]*alpha;
            y = y - ret[1]*alpha;	    
        if self.loss_fun(x, y, base_x, base_y, distances) > 1000 :
            return
        self.pos.x = x
        self.pos.y = y
        #self.pos.x = self.head_x
        #self.pos.y = self.head_y
        self.pos.z = 0
        self.pos.yaw = 0
        self.writer.write(self.pos)


if __name__ == '__main__':
    cyber.init()

    exercise_node = cyber.Node("localization_node")
    
    exercise = Exercise(exercise_node)
    exercise_node.spin()
    cyber.shutdown()




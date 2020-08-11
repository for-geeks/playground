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
        self.alpha = 0.02
        self.uwb_result = [0, 0]
        self.maxTimes = 200
        self.x = np.array([0.725, 0.7]);
        # base station matrix
        self.base_x = np.array([3.3, 3.3, 5.5, 5.5])
        self.base_y = np.array([1.1, 3.38, 1.1, 3.38])
        
        self.xi = np.array([])
        self.yi = np.array([])
        # distance 
        self.di = np.array([])
        #self.di = np.array([8, 9.303, 0, 4.75])
        self.states = np.array([1, 0, 2.8, 0]).T
        self.states_2 = np.array([1, 0, 2.8, 0]).T
        self.localization = localization()
        self.pos = pos()
        self.gyroz = 0
        self.vel_head = 0
        self.yaw = 3.14
        self.direction = 0
        self.kalman_x = 1
        self.kalman_y = 2.8
        self.node.create_reader("/geek/gyro", Gyro, self.gyrocallback)
        self.node.create_reader("/geek/uwb/pose", TagFrame, self.tagcallback)
        self.node.create_reader("/chassis", Chassis, self.chassiscallback)
        self.writer = self.node.create_writer("/geek/uwb/localization", pos)
        self.fuseflag = 0
        self.v_x = 0
        self.head_x = 1
        self.head_y = 0
        self.v_y = 0
        self.direction_lag = 0
        self.x_tag = 0
        self.y_tag = 0
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
        if (1):
            self.gyroz = -gyro.gyro.z
            

    def prediction_func(self, states, deltat):
        states_new = np.array([0.0, 0.0, 0.0, 0.0]).T
        states_new[0] = states[0] + deltat * states[1]
        #states_new[1] = 
        states_new[2] = states[2] + deltat * states[3]
        #states_new[3] = velocity * math.sin(states[4])

        return states_new
        

    def mesurement_func(self, states, uwb_result):
        L = np.array([[0.1, 0],[ 0.2, 0],[0.0,0.1],[0.0,0.2]])
        error = np.array([uwb_result[0] - states[0], uwb_result[1] - states[2]]).T
        dstate = np.matmul(L, error)
        newstate = states + dstate
        return newstate
        pass


    def calculate_yaw(self, v_x, v_y, speed, last_yaw):
        delta_x = 1
        delta_y = math.tan(last_yaw)
        lenth_xy = math.sqrt(1 + delta_y ** 2)
        #head_x_new = delta_x / lenth_xy
        #head_y_new = delta_y / lenth_xy
        if math.fabs(speed) > 0.001:
            head_x_new = (self.head_x * math.cos(self.gyroz * 0.05) - self.head_y * math.sin(0.05 * self.gyroz))

            head_y_new = (self.head_x * math.sin(self.gyroz * 0.05) + self.head_y * math.cos(self.gyroz * 0.05))
            self.head_x = head_x_new
            self.head_y = head_y_new
        else:
            return last_yaw
        if (math.sqrt(v_x**2 + v_y**2) > 0.05):
            lenth_v = math.sqrt(v_x ** 2 + v_y ** 2)
            if speed < 0:
                pass
                #head_x_new += 0.2 * (-v_x / lenth_v - head_x_new)
                #head_y_new += 0.2 * (-v_y / lenth_v - head_y_new)
            else :
                self.head_x += 0.2 * (v_x/ lenth_v - self.head_x)
                self.head_y += 0.2 * (v_y / lenth_v - self.head_y)
        return math.atan2(self.head_y, self.head_x)
        

    def second_prediction_func(self, states_2, velocity, yaw, deltat):
        states_new = np.array([0.0, 0.0, 0.0, 0.0]).T
        states_new[1] = velocity * math.cos(yaw)
        states_new[0] = states_2[0] + deltat * states_2[1]
        states_new[3] = velocity * math.sin(yaw)
        states_new[2] = states_2[2] + deltat * states_2[3]
        return states_new

    def second_mesurement_func(self, states_2, uwb_result):
        L = np.array([[0.05, 0],[ 0.01, 0],[0.0,0.05],[0.0,0.01]])
        error = np.array([uwb_result[0] - states_2[0], uwb_result[1] - states_2[2]]).T
        dstate = np.matmul(L, error)
        newstate = states_2 + dstate
        return newstate
	
    def chassiscallback(self, chassis):
        if (chassis.speed > 0):
            self.direction = 1
        else:
            self.direction = -1


        if (abs(chassis.speed) < 0.1):
            self.direction = 0
        self.direction_lag = self.direction_lag * 0.5 + 0.5 * self.direction

        velocity = chassis.speed

        self.states = self.prediction_func(self.states, 0.05)

        self.states = self.mesurement_func(self.states, self.uwb_result)
       
        self.yaw = self.calculate_yaw(self.states[1], self.states[3], velocity, self.yaw)

	#second observer was called here
        
        self.states_2 = self.second_prediction_func(self.states_2, velocity, self.yaw, 0.05)

        self.states_2 = self.second_mesurement_func(self.states_2, self.uwb_result)

        self.pos.x = self.states_2[0]
        self.pos.y = self.states_2[2]
        self.pos.yaw = self.yaw + 3.14
	
        self.writer.write(self.pos)

    def tagcallback(self, tag):
        self.di = np.array([])
        self.xi = np.array([])
        self.yi = np.array([])
        for k in range (0, 4):
            if tag.dis[k].distance == 1:
                continue
            self.di = np.append(self.di, tag.dis[k].distance)
            self.xi = np.append(self.xi, self.base_x[k])
            self.yi = np.append(self.yi, self.base_y[k])
            
        datas = np.array([[self.xi], [self.yi], [self.di]])
        datas = datas.T
        self.calcu_loss_fun(self.x, self.maxTimes, self.alpha, datas)

    def problem(self, x, data):
        return (x[0] - data[0,0]) ** 2 +  (x[1] - data[0,1])**2 - data[0,2]**2  

    def loss_fun(self, x, datas):
        sum_err = 0;
        for data in datas:
          sum_err += math.fabs((self.problem(x, data) - 0))
        return sum_err

    def slope_fx(self, x, datas):
        d = 0.01;
        delta1  = np.array([d, 0]);
        delta2  = np.array([0, d]);
        J1 = (self.loss_fun(x+delta1, datas) - self.loss_fun(x-delta1, datas)) / (2.0*d)
        J2 = (self.loss_fun(x+delta2, datas) - self.loss_fun(x-delta2, datas)) / (2.0*d)
        return [J1, J2]

    def calcu_loss_fun(self, x, maxTimes, alpha, datas):
        for i in range(maxTimes):
                ret = self.slope_fx(x, datas)
                x1 = x[0] - ret[0]*alpha;
                x2 = x[1] - ret[1]*alpha;	
                x = np.array([x1, x2])
        if self.loss_fun(x, datas) > 1000:
            return
    
        self.uwb_result = x



if __name__ == '__main__':
    cyber.init()

    exercise_node = cyber.Node("localization_node")
    
    exercise = Exercise(exercise_node)
    exercise_node.spin()
    cyber.shutdown()




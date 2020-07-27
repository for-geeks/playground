#!/usr/bin/env python
# _*_ coding: utf-8 _*_
import sys
import math
import time
import numpy as np
import signal
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class Exercise(object):
    def __init__(self, node):
        self.node = node
        self.alpha = 0.01
        self.maxTimes = 200
        self.x = 9
        self.y = 9
        # base station matrix
        self.base_x = np.array([3.3, 3.3, 5.5, 5.5])
        self.base_y = np.array([1.1, 3.38, 1.1, 3.38])
        distances = [0, 2.28, 2.2, 3.168]

        self.calcu_loss_fun(self.x, self.y, self.maxTimes, self.alpha, self.base_x, self.base_y, distances)
           
    # 定义函数f(x)
    def problem(self, x, y, base_x, base_y, distance):
        '''
        fill this function here
        '''
        return 0
    #定义损失函数
    def loss_fun(self, x, y, base_x, base_y, distances):
        sum_err = 0;
        for i in range(0, len(base_x)):
          sum_err += math.fabs((self.problem(x, y, base_x[i], base_y[i], distances[i]) - 0))
        return sum_err


    def slope_fx(self, x, y, base_x, base_y, distances):
        d = 0.01;
        '''
        fill this function, calculate slope for iteration
        '''
        J1 = 0
        J2 = 0
        return [J1, J2]
    
    def calcu_loss_fun(self, x, y, maxTimes, alpha, base_x, base_y, distances):
        
        x_vec = np.linspace(0,9,50)
        y_vec = np.linspace(0,9,50)
        X, Y = np.meshgrid(x_vec, y_vec)
        Z = [[]]
        for x_this in x_vec:
            z_list = []
            for y_this in y_vec:
                z_list.append(self.loss_fun(x_this,y_this,base_x, base_y, distances))
            Z.append(z_list)
        Z.remove([])
	#Z.resize(100, 100)
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        #ax.contour3D(X, Y, Z, 50, cmap='binary')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.view_init(60, 35)
        #plt.contourf(X, Y, Z)
        Z = np.array(Z)
        ax.plot_surface(Y, X, Z, cmap = 'viridis')
	# 显示图表
	
        xline = np.array([x])
        yline = np.array([y])
        zline = np.array([self.loss_fun(x, y, base_x, base_y, distances)])

        for i in range(maxTimes):
            ret = self.slope_fx(x, y, base_x, base_y, distances)
            x = x - ret[0]*alpha;
            y = y - ret[1]*alpha;	
            #x = np.array([x1, x2])
            zline = np.append(zline, self.loss_fun(x, y, base_x, base_y, distances))
            xline = np.append(xline, x)
            yline = np.append(yline, y)
            ax.plot3D(xline, yline, zline, 'red')
            plt.pause(0.001)
            print(x, y)
        print("result is : ")
        print("[x: %f, y: %f]", x, y)
        plt.show()


if __name__ == '__main__':
    exercise = Exercise(0)





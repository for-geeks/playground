#!/usr/bin/env python
# _*_ coding: utf-8 _*_
import sys
import math
import time
import signal
import cv2
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

is_sigint_up = False
x0 = 0
u = 0
x = x0
t = 0

def eular_iteration_func(x_last, u, delta_t):
    '''
        delta_x = ????
        x_now = ???

        student should fullfill this function, system shall work
    '''
    x_now = 0
    return x_now

def on_key_press(event):
    if event.key in "ws":
        if (event.key == 'w'):
            global u
            u = u + 0.1
        else:
            global u
            u = u - 0.1

def simple_plot(delta_t):   
    fig = plt.figure(figsize=(8, 6), dpi=80)
    fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    plt.ion()
    plt.cla()
    plt.title("plot")
    plt.grid(True)
    t_vec = []
    x_vec = []
    u_vec = []
    while True:
        if is_sigint_up:
                print("Exit plot")
                break
        pass

        plt.cla()
        plt.xlabel("X")
        plt.xlim(-4 + t,  t)
        plt.xticks(np.linspace(-4 + t, +t, 9, endpoint=True))

        # 设置Y轴
        plt.ylim(-1.0, 1.0)
        plt.yticks(np.linspace(-1, 1, 9, endpoint=True))
        t_vec.append(t)
        x_vec.append(x)
        u_vec.append(u)
        if (len(t_vec) > 100):
            t_vec.pop(0)
            x_vec.pop(0)
            u_vec.pop(0)
        # 画两条曲线
        plt.plot(t_vec, x_vec, "b--", linewidth=2.0, label="x")
        plt.plot(t_vec, u_vec, "g-", linewidth=2.0, label="u")

        # 设置图例位置,loc可以为[upper, lower, left, right, center]
        plt.legend(loc="upper left", shadow=True)
        plt.pause(delta_t)
    plt.show()
    plt.ioff()
    plt.close()
    
def sigint_handler(signum, frame):
            #global is_sigint_up
        global is_sigint_up
        is_sigint_up = True
        print("catched interrupt signal!")

def iter_loop():
    delta_t = 0.01
    while True:
        global x
        global u
        global t
        x = eular_iteration_func(x, u, delta_t)
        t = t + delta_t
        time.sleep(delta_t)
        global is_sigint_up
        if is_sigint_up:            
                print("Exit")
                break
        pass

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGHUP, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)
    t1 = threading.Thread(target=iter_loop, args=())
    t1.start()
    simple_plot(0.01)
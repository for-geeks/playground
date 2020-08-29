import math
import time
import numpy as np
import matplotlib.pyplot as plt

class Config(object):
    """
    用来仿真的参数，
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.3  # [m/s]  # 最大速度
        #self.min_speed = 0  # [m/s]  # 最小速度，设置为可以倒车
        self.min_speed = 0.1  # [m/s]  # 最小速度，设置为不倒车
        self.max_yawrate = 120.0 * math.pi / 180.0  # [rad/s]  # 最大角速度
        self.max_accel = 0.8  # [m/ss]  # 最大加速度
        self.max_dyawrate = 120.0 * math.pi / 180.0  # [rad/ss]  # 最大角加速度
        self.v_reso = 0.03  # [m/s]，速度分辨率
        self.yawrate_reso = 1 * math.pi / 180.0  # [rad/s]，角速度分辨率
        self.dt = 0.1  # [s]  # 采样周期
        self.predict_time = 5  # [s]  # 向前预估三秒
        self.to_goal_cost_gain = 10 # 目标代价增益
        self.speed_cost_gain = 10  # 速度代价增益
        self.obstacle_cost_gain = 10
        self.robot_radius = 0.1  # [m]  # 机器人半径

def motion(x, u, dt):

    x[0] += u[0] * math.cos(x[2]) * dt  # x方向位移
    x[1] += u[0] * math.sin(x[2]) * dt  # y
    x[2] += u[1] * dt  # 航向角
    x[3] = u[0]  # 速度v
    x[4] = u[1]  # 角速度w

    return x

def calc_dynamic_window(x, config):
    """
    位置空间集合
    """

    # 车辆能够达到的最大最小速度
    vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # 一个采样周期能够变化的最大最小速度
    vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    # 求出两个速度集合的交集
    vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
          max(vs[2], vd[2]), min(vs[3], vd[3])]

    return vr


def calc_trajectory(x_init, v, w, config):
    """
    预测x秒内的轨迹
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        trajectory = np.vstack((trajectory, x))  # 垂直堆叠，vertical
        time += config.dt

    return trajectory


def calc_to_goal_cost(trajectory, goal, config):
    dx = # ?
    dy = # ? 
    goal_dis = # ? 
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def calc_obstacle_cost(traj, ob, config):
    """
    计算预测轨迹和障碍物的最小距离，dist(v,w)
    """

    skip_n = 2  # for speed up
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_final_input(x, u, vr, config, goal, ob):
    """
    计算采样空间的评价函数，选择最合适的那一个作为最终输入
    """
    x_init = x[:]
    min_cost = 10000.0
    min_u = u
    v = 0.5

    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    # v,生成一系列速度，w，生成一系列角速度

    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):

            trajectory = calc_trajectory(x_init, v, w, config)

            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            # 评价函数多种多样，看自己选择
            final_cost = to_goal_cost + speed_cost + ob_cost

            #print(to_goal_cost, speed_cost, ob_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
                best_trajectory = trajectory

    return min_u, best_trajectory


def dwa_control(x, u, config, goal, ob):
    """
    调用前面的几个函数，生成最合适的速度空间和轨迹搜索空间
    """
    # Dynamic Window control

    vr = calc_dynamic_window(x, config)

    u, trajectory = calc_final_input(x, u, vr, config, goal, ob)

    return u, trajectory


def plot_arrow(x, y, yaw, length=0.5, width=0.01):
    """
    arrow函数绘制箭头
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=0.15 * width, head_width=width)
    plt.plot(x, y)


def main():
    """
    主函数
    :return:
    """
    # print(__file__ + " start!!")
    # 初始化位置空间
    x = np.array([0.0, 0.0, math.pi / 2.0, 0.2, 0.0])

    goal = np.array([0, 1])

    ob = np.matrix([[0.0, 0.5]])

    u = np.array([0.3, 0.0])

    config = Config()
    trajectory = np.array(x)
    best_trajectory = np.array(x)

    num = 0

    time_start = time.time()


    vr = calc_dynamic_window(x, config)
    trajs = []
    min_cost = 10000.0
    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):
            print([v, w])
            this_traj = calc_trajectory(x, v, w, config)
            trajs.append(this_traj)
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(this_traj, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - this_traj[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(this_traj, ob, config)

            # 评价函数多种多样，看自己选择
            final_cost = to_goal_cost + speed_cost + ob_cost

            #print(to_goal_cost, speed_cost, ob_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_trajectory = this_traj

    
    time_end = time.time()

    print('time:', time_end - time_start)
    print(trajectory)

    print("Done", num)

    draw_path(trajs, goal, ob, x, best_trajectory)

def draw_path(trajectorys, goal, ob, x, best_trajectory):
    """
    画图函数
    """
    
    plt.cla()  # 清除上次绘制图像
    
    for trajectory in trajectorys:
        plt.plot(0, 0, "og")
        #plot_arrow(x[0], x[1], x[2])
        plt.axis("equal")
        plt.grid(True)
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-')
    plt.plot(goal[0], goal[1], "ro")
    plt.plot(ob[:, 0], ob[:, 1], "bs")
    plt.plot(x[0], x[1], "xr")
    plt.plot(best_trajectory[:, 0], best_trajectory[:, 1], 'g*')
    plt.show()


if __name__ == '__main__':
    main()

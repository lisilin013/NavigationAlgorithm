#!/usr/bin/python3
# coding:utf-8

'''
SIR粒子滤波的应用，算法流程参见博客http://blog.csdn.net/heyijia0327/article/detail
'''

import numpy as np
import math
import matplotlib.pyplot as plt


x = 0.1  # initial state
process_cov = 1  # 状态方差
meassure_cov = 1  # 测量方差
TestNum = 100  # 实验次数
N = 100  # particle数目

init_cov = 2  # 初始化分布方差
x_P = np.zeros(N)  # Particles
x_P_update = np.zeros(N)  # Particles
z_update = np.zeros(N)  # Particles
P_w = np.zeros(N)  # Particles weight

# 用一个高斯分布随机产生N个初始粒子
for i in range(0, N):
    x_P[i] = x + np.sqrt(init_cov) * np.random.normal(0, 1)

# plot显示的变量存储
z_out = np.zeros(TestNum)
x_true = np.zeros(TestNum)
x_est_out = np.zeros(TestNum)

for t in range(0, TestNum):
    # 系统状态方程和观测方程的真值
    x_tr = 0.5 * x + 25 * x / (1 + x ** 2) + 8 * np.cos(1.2 * (t - 1))
    z_tr = x_tr ** 2 / 20

    # 状态预测值
    x = x_tr + np.sqrt(process_cov) * np.random.normal(0, 1)
    # 传感器测量值
    z = z_tr + np.sqrt(meassure_cov) * np.random.normal(0, 1)

    # 更新粒子
    for i in range(0, N):
        # 从先验p(x(k)|x(k-1))中采样
        x_P_update[i] = 0.5 * x_P[i] + 25 * x_P[i] / (1 + x_P[i] ** 2) + 8 * np.cos(1.2 * (t - 1)) + np.sqrt(process_cov) * np.random.normal(0, 1)
        # 计算采样粒子的值，为后面根据似然去计算权重做铺垫
        z_update[i] = x_P_update[i] ** 2 / 20
        # 对每个粒子计算其权重，这里假设量测噪声是高斯分布。所以 w = p(y|x) 对应下面的计算公式，其实也即是观测方程的高斯函数形式
        P_w[i] = (1 / np.sqrt(2 * np.pi * meassure_cov)) * np.exp(-(z - z_update[i]) ** 2 / (2 * meassure_cov))

    # 权重归一化
    P_w = P_w / P_w.sum()

    # Resampling
    for i in range(0, N):
        ind = np.where(np.random.uniform(0, 1) <= np.cumsum(P_w))[0][0]  # 返回第一个符合条件的数的下标
        x_P[i] = x_P_update[ind]  # 粒子权重大的将多得到后代
    P_w = np.zeros(N) + 1.0 / N  # 权重重新初始化
    # 状态估计，resampling之后，粒子权重变为1/N
    x_est = np.mean(x_P)

    # save data
    x_true[t] = x_tr
    x_est_out[t] = x_est

    # z_out[t] = z

t = range(0, TestNum)
plt.plot(t, x_true, '.-b', t, x_est_out, '-.r')
plt.xlabel('time step')
plt.ylabel('x estimate')
plt.legend(['Ground True', 'Estimate'])
plt.show()

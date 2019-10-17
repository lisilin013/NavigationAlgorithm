"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt


# Estimation parameter of PF
Q = np.diag([0.1]) ** 2  # range error
R = np.diag([1.0, np.deg2rad(40.0)]) ** 2  # input error

#  Simulation parameter
Qsim = np.diag([0.2]) ** 2
Rsim = np.diag([1.0, np.deg2rad(30.0)]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range

# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

show_animation = True

'''
得到一个系统输入值u
'''


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


'''
观测方程
'''


def observation(xTrue, xd, u, RFID):
    '''
    :param xTrue: 状态真值
    :param xd: 航迹推算值
    :param u: 输入值
    :param RFID: landmark坐标
    :return: 状态真值，测量值，状态航迹推算值，输入u
    z(distance, rfid_x, rfid_y)
    '''
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx ** 2 + dy ** 2)
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            zi = np.array([[dn, RFID[i, 0], RFID[i, 1]]])
            z = np.vstack((z, zi))  # rfid_dim*3

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    '''
    根据input和当前状态推测下一时刻状态，其实是状态方程
    :param x: x,y,(2d pose) phi(orientation), v(velocity)
    :param u: v_t, w_t
    :return:
    '''
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x


'''
高斯函数，根据方差sigma得到在x=(\mu-mean)的概率
'''


def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(xEst, px, pw):
    cov = np.zeros((3, 3))

    # 遍历所有粒子
    for i in range(px.shape[1]):
        # 计算粒子状态和估计状态的前三维(x,y,phi)的error
        dx = (px[:, i] - xEst)[0:3]
        # weight*error^2作为var
        cov += pw[0, i] * dx.dot(dx.T)

    return cov


def pf_localization(px, pw, xEst, PEst, z, u):
    '''
    Localization with Particle filter
    :param px: particles
    :param pw: particles' weights
    :param xEst: 状态估计值
    :param PEst: 状态估计协方差矩阵
    :param z: 测量值
    :param u: input
    :return: xEst, PEst, px, pw
    '''

    for ip in range(NP):
        x = np.array([px[:, ip]]).T
        w = pw[0, ip]
        # ------------------------------------------
        # 【第1步 粒子采样 P(x_k|x_k-1)】
        # 这里其实是SIR filter，因为采样并没有用到测量值z
        # ------------------------------------------
        ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud)

        # ------------------------------------------
        # 【第2步 计算粒子的权值更新 】
        # ------------------------------------------
        for i in range(len(z[:, 0])):  # z(rfid_dim*3)
            dx = x[0, 0] - z[i, 1]
            dy = x[1, 0] - z[i, 2]
            prez = math.sqrt(dx ** 2 + dy ** 2)  # 机器人位置距离rfid的距离
            dz = prez - z[i, 0]  # 距离error
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))  # 将所有的rfid测量值逐个迭代完成得到该粒子的权重

        px[:, ip] = x[:, 0]  # 更新粒子状态
        pw[0, ip] = w  # 更新例子权值

    # ------------------------------------------
    # 【第3步 权值归一化 】
    # ------------------------------------------
    pw = pw / pw.sum()

    # ------------------------------------------
    # 【第4步 粒子滤波状态估计】
    # ------------------------------------------
    xEst = px.dot(pw.T)  # 估计粒子状态
    PEst = calc_covariance(xEst, px, pw)  # 估计粒子的协方差矩阵，只计算了状态的前三维(x,y,phi)的cov

    # ------------------------------------------
    # 【第5步 重采样】
    # ------------------------------------------
    px, pw = resampling(px, pw)

    return xEst, PEst, px, pw


def resampling(px, pw):
    """
    low variance re-sampling
    """

    Neff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
    # 有效粒子数目少的时候进行重采样
    if Neff < NTh:
        wcum = np.cumsum(pw)  # 维度 NP*1 轮盘采样
        base = np.cumsum(pw * 0.0 + 1 / NP) - 1 / NP  # 维度 NP*1, 假如NP=100，则base是一个按照0.01递增的[0,0.99]序列
        resampleid = base + np.random.rand(base.shape[0]) / NP  # 维度 NP*1，均匀分布采样

        # 得到重采样ids
        inds = []
        ind = 0
        for ip in range(NP):
            while resampleid[ip] > wcum[ind]:
                ind += 1
            inds.append(ind)

        px = px[:, inds]
        pw = np.zeros((1, NP)) + 1.0 / NP  # 权重重新初始化

    return px, pw


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # eigval[bigind] or eiqval[smallind] were occassionally negative numbers extremely
    # close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try:
        a = math.sqrt(eigval[bigind])
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eigval[smallind])
    except ValueError:
        b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, 0.0],
                     [10.0, 10.0],
                     [0.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))  # estimate state
    xTrue = np.zeros((4, 1))  # ground true
    xDR = np.zeros((4, 1))  # Dead reckoning
    PEst = np.eye(4)  # 估计的协方差矩阵

    px = np.zeros((4, NP))  # Particle store
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xDR

    # -------------------------------------------------
    # 主循环
    # -------------------------------------------------
    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:
            plt.cla()

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 1]], [xTrue[1, 0], z[i, 2]], "-k")
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(px[0, :], px[1, :], ".r")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()

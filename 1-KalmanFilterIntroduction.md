# 1-KalmanFilterIntroduction

标签（空格分隔）： navigation-algorithm

[toc]

---

# Reference
- [how-a-kalman-filter-works-in-pictures][1]
- [kalman filter 基本思想](https://www.zhihu.com/question/22422121)
- [robot_localization ros pkg][2]
- [Kalman Filter 通俗讲解][3]
- [ekf tutorial][4]
- [手把手教你写卡尔曼滤波器][5]

* HOPE TO READING Book: 
>**Poor Man's Explanation of Kalman Filtering: Or How I Stopped Worrying & Learned to Love Matrix Inversion**

# 1 Kalman Filter的来源
使用bayes filter来实现广泛使用的一项技术就是kalman filter(KF)，KF在1958年被Swerling和Kalman在研究linear gaussian system的滤波和预测时发明的。KF实现了对连续状态空间的belif computation。它不能用于离散或者混合状态空间。

# 2 KF理解

* $K_{k}$卡尔曼增益实际上是表示观测量$z_k$的权重,如果相信$z_K$更多一些,那么这个增益就越大.
* 关于自己的理解

> 用$k-1$时刻的状态量来预测,得到$k$时刻的先验状态量估计值和先验估计偏差,然后计算测量值的可靠程度(卡尔曼增益),最后结合$k$时刻的测量值,得到$k$时刻的后验最优估计值和其偏差

- 使用KF问题场景：
    - 一个sensor获取机器人state
    - 一个sensor可以predict机器人state，抽象出一个预测方程，预测state和该state对应的covariance
- KF假设：
变量是随机的并且是高斯分布的，对应均值$\mu$和方差$\sigma^2$,方差用来表示数据的不确定性

# 3 kalman在一维情况下的例子
- Model
$x_k = a x_{k-1} + b u_k + \omega_k$ 状态转移方程
$z_k = c x_k + v_k$ 观测方程
- predict
$\hat{x}_k = a\hat{x}_{k-1}+ b u_k$
$p_k = ap_{k-1}a+q$
- update
$g_k=p_kc/(cp_kc+r)$
$\hat{x}_k \leftarrow \hat{x}_k + g_k(z_k -c\hat{x}_k)$
$p_k \leftarrow (1 - g_kc)p_k$

$\omega_k \sim N(0, q)$表示模型控制的噪声方差
$v_k \sim N(0,r)$表示观测量(传感器)的噪声方差
$g_k$表示卡尔曼增益
$p_k$表示预测的方差

# 4 扩展至高维度

- Model
$x_k = A x_{k-1} + B u_k + \omega_k$ 状态转移型方程
$z_k = H x_k + v_k$ 观测方程
- predict
$\hat{x}_k = A\hat{x}_{k-1}+ B u_k$
$P_k = AP_{k-1}A^T+Q$
- update
$G_k=P_kH^T(HP_kH^T+R)^{-1}$
$\hat{x}_k \leftarrow \hat{x}_k + G_k(z_k -H\hat{x}_k)$
$P_k \leftarrow (1 - G_kH)P_k$

$\omega_k \sim N(0, Q)$表示模型控制的噪声方差
$v_k \sim N(0,R)$表示观测量(传感器)的噪声方差
$G_k$表示卡尔曼增益
$P_k$表示预测的方差

$H$相当于一维里面的$c$

# 5 使用非线性函数来扩展KF成为EKF
- Model
$x_k = f(x_{k-1}, u_k)  + w_k$
$z_k = h(x_{k})  + v_k$
- predict
$\hat{x}_k = f(\hat{x}_{k-1}, u_k)$
$P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}$
- update
$G_k = P_k H_k^T (H_k P_k H_k^T + R)^{-1}$
$\hat{x}_k \leftarrow \hat{x}_{k} + G_k(z_k - h(\hat{x}_{k}))$
$P_k \leftarrow (I - G_k H_k) P_k$

其中，$F_k$是$f(x_{k-1}, u_k)$的Jacobian矩阵，$H_k$是$h(x_{k})$的Jacobian矩阵

# 6 调参经验之谈
- P初始化不能给0，一般设置为1
- R是根据传感器的数据方差设置的，两个传感器的协方差一般设置为0
- Q对整个系统存在影响，但又不能太确定对系统的影响有多大。工程上，我们一般将Q设置为单位矩阵参与运算

# 7 KF内矩阵的认识理解
- $A$或者$F$ 状态转移矩阵（state transistion matrix）有模型的状态转移方程决定
- $B$ 
- $P$ 状态协方差矩阵（state covariance matrix）不可以初始化为0
- $Q$ 过程噪声（process covariance matrix）工程上一般设置为I
- $H$ 测量矩阵（Measurement Matrix）由模型的观测方程模型决定
- $R$ 测量噪声矩阵（measurement covariance matrix），由传感器的噪声决定
- $K$ 卡尔曼增益（Kalman Gain），用人话讲就是求y值的权值



  [1]: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
  [2]: http://docs.ros.org/melodic/api/robot_localization/html/index.html
  [3]: https://blog.csdn.net/u010665216/article/details/80556000
  [4]: https://home.wlu.edu/~levys/kalman_tutorial/
  [5]: https://zhuanlan.zhihu.com/p/45238681
  
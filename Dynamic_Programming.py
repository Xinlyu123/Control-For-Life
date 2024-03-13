#Xin Lyu
#Dynamic Programming (Drone Case)
# Initial Condtion: h(0) = 0  target goal; h(tf)=10
#                   h_dot(0) = 0      h_dot(tf) = 0
#                   Cost function = tf (Minimize the flying time)
# Constraints       -3<= alpha <= 2
#                   0 <= V <= 3
# System dynamics   m*h_ddot = f(t) - mg

import numpy as np

def drone_case():
    # 代价方程最小化 时间 6x4 的矩阵
    t = np.zeros([6, 4])
    #t_temp = np.zeros([4,4])
    a = np.zeros([6, 4])
    dx = 2


    # 计算 10 m 到 8 m
    # 计算平均速度
    vel_init = np.zeros([1,4])
    vel_final = 0
    vel = np.array([0,1,2,3])
    avg_vel = np.zeros([1,4])

    acc = np.zeros([4,4])
    for i in range(4):
        # 计算平均速度
        avg_vel[0,i] = 0.5*(vel_init[0,i]+vel[i])
        # 速度和为0时 t-> infinity
        if avg_vel[0,i] == 0:
            t[1,i] = np.inf
        else:
            t[1,i] = dx/avg_vel[0,i]
        # 计算加速度
        if t[1,i] == 0:
            a[1,i] = 0
        else:
            a[1,i] = (t[0,0]-vel[i])/t[1,i]
    # 找出加速度超出边界的点位 并将cost记为无穷
    acc_x, acc_y = np.where(np.logical_or(a < -3, a > 2))

    # 计算 node 与 node 之间的cost 一共 16 （4*4）
    t[acc_x, acc_y] = np.inf


    # 计算 8 m - 2 m
    for h in range (2,5,1): # 2 3 4
        arr = np.array([0, 1, 2, 3])

        V_x = np.tile(arr, (4, 1))
        V_y = np.reshape(np.repeat(arr.reshape(1, -1), 4, axis=1), (4, 4))

        V_avg = 0.5 * (V_x + V_y)
        delta = dx/V_avg
        acc = (V_y - V_x)/delta
        J_temp = delta

        #遍历 加速度矩阵 找到不符合限制条件的
        acc_x,acc_y = np.where(np.logical_or(acc < -3, acc > 2))

        # 计算 node 与 node 之间的cost 一共 16 （4*4）
        J_temp[acc_x,acc_y] = np.inf

        # 计算cost function
        res = np.reshape(np.repeat(np.reshape(t[h - 1, :],(4,1)),4),(4,4))
        J_temp += res
        t[h,:] = np.min(J_temp,axis=0)
        indices = np.argmin(J_temp,axis= 0)

        # 找到对应的加速度
        for i in range(4):
            a[h,i] = acc[indices[i],i]


    # 计算 8 m - 2 m
    J_temp = np.zeros([1, 4])
    a_temp = np.zeros([1, 4])
    for i in range(4):
        #计算平均速度
        avg_vel[0,i] = 0.5*(vel_final+vel[i])
        # 速度和为0时 t-> infinity

        J_temp[0,i] = dx / avg_vel[0, i]
        # 计算加速度
        if J_temp[0,i] == 0:
            a_temp[0,i] = np.inf
        else:
            a_temp[0,i] = ( vel[i]-vel_init[0,i]) / J_temp[0, i]
    acc_x, acc_y = np.where(np.logical_or(a_temp < -3, a_temp > 2))

    # 计算 node 与 node 之间的cost 一共 16 （4*4）
    J_temp[acc_x, acc_y] = np.inf
    J_temp += t[4,:]
    t[5,0] = np.min(J_temp, axis = 1)
    indices = np.argmin(J_temp, axis=1)
    a[5,0] = acc[indices,0]
    
    print(t)
    print(a)


drone_case()



# ME:5114 Nonlinear Control
# Lecture 32: car-like mobile robot
# by Prof. Shaoping Xiao

# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import abc # abstract base class
import math
import scipy as sp


# polynomial planner
def TMatrix(d, t):
    """Computes the T matrix at time t
    Args: t (int, float): time
    Returns: T (ndarray): T matrix
    """
    n = d+1 # first dimension of T
    m = 2*d+2 # second dimension of T

    T = np.zeros([n, m])

    for i in range(0, m):
        T[0, i] = t ** i / math.factorial(i)
        for j in range(1, n):
            T[j, j:m] = T[0, 0:m-j]
    return T

def PolyCoef(d, t0, tf, YA, YB):
    """ calculate the coefficients for the polynomical planner
    """
    T0=TMatrix(d,t0)
    Tf=TMatrix(d,tf)

    Y = np.append(YA, YB)
    T = np.append(T0, Tf, axis=0)

    # solve the linear equation system for c
    return np.linalg.solve(T, Y)

def Traj(d, tt, c):
    """ evaluate state variable along the trajectory
    """
    Y = np.zeros([len(tt), d+1])
    m=0
    for t in tt:
       Y[m] = np.dot(TMatrix(d,t), c)
       m=m+1
    return Y
    

t0 = 0 # start time of transition
tf = 1 # final time of transition
tt = np.linspace(t0, tf, 100) 

d = 2

# this is for one state varialbe, [xd, dxd, ddxd]
XA = np.array([0, 0, 0]) # t = t0
XB = np.array([5, 0, 0]) # t = tf

c = PolyCoef(d, t0, tf, XA, XB)
X = Traj(d, tt, c)



#plot the trajectory
plt.figure(1)
plt.ylim(-30, 30)
plt.plot(tt, X)
plt.title('Planned trajectory')
plt.legend([r'$x_d(t)$', r'$\dot{x}_d(t)$',r'$\ddot{x}_d(t)$'])
plt.xlabel(r't in s')
plt.grid(True)


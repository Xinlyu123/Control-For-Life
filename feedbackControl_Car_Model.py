#Author: Xin Lyu
#car-like mobile robot


# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import abc # abstract base class
import math
import scipy as sp
from numpy import cos, arccos, sin, arcsin, tan, arctan, arctan2, pi
from scipy.integrate import odeint

def solve_car(x, t):
    x1, x2, x3 = x  # state vector
    u1, u2 = control(t, x)  # control vector
    # dxdt = f(x, u):
    return [u1 * np.cos(x3), u1 * np.sin(x3), 1.0/l * u1 * np.tan(u2)]

def control(t, x):
    """Function of the control law

    Args:
        x: state vector
        t: time

    Returns:
        u: control vector

    """
    g_t=np.dot(TMatrix(1,t), c_g)
    f_y1 = np.dot(TMatrix(2,g_t[0]), c_f) # y2 = f(y1) = f(g(t))

    
    # design gain
    k1 = 1
    k2 = 2
    k3 = 2
    # LISTING_END DefineControllerPara

    # state vector
    y1 = x[0]
    y2 = x[1]
    theta = x[2]

    dy2 = sin(theta)

    # LISTING_START DefineRefTraj
    # reference trajectories yd, yd', yd''
    y1d = g_t[0]
    dy1d = 1/(np.sqrt(1 + f_y1[1] ** 2))

    y2d = f_y1[0]
    dy2d = f_y1[1]/(np.sqrt(1 + f_y1[1] ** 2))
    ddy2d = f_y1[2]/(1 + f_y1[1] ** 2)
    # LISTING_END DefineRefTraj

    # LISTING_START CalcStabInputs
    # stabilizing inputs
    w1 = dy1d - k1 * (y1 - y1d)
    w2 = ddy2d - k2 * (dy2 - dy2d) - k3 * (y2 - y2d)
    # LISTING_END CalcStabInputs

    # LISTING_START ControlLaw
    # control laws
    ds = g_t[1] * np.sqrt(1 + (f_y1[1]) ** 2) #desired velocity
    u1 = ds*np.sqrt(w1**2+dy2**2)
    u2 = arctan2(l * (w2 * w1), cos(theta)**2.0)

    
    return np.array([u1, u2]).T


# polynomial planner
# Matrix T = [[1 t t^2/2 ... t^(2n+1)/(2n+1)!],[0 1 t ... t^2n/(2n)!],[] ....]
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
    

# Physical parameter
l = 0.3         # define car length
w = l*0.3       # define car width

# Simulation parameter
t0 = 0          # start time
tf = 10         # final time
dt = 0.04       # step-size
tt = np.arange(t0, tf +dt, dt)
# initial state and final state
x0 = [0, 0, 0]
xf = [5, 5, 0]

# boundary conditions for y1, and planner g
Y1A = np.array([x0[0], 0])
Y1B = np.array([xf[0], 0])
c_g = PolyCoef(1, t0, tf, Y1A, Y1B)

# boundary conditions for y2 and planner f
Y2A = np.array([x0[1], tan(x0[2]), 0])
Y2B = np.array([xf[1], tan(xf[2]), 0])
c_f = PolyCoef(2, Y1A[0], Y1B[0], Y2A, Y2B)  # f(g) so it is from g0 to gt

sol1 = odeint(solve_car, x0, tt)

# control signals
u_c = np.zeros([len(tt),2])
for i in range(0, len(tt)):
    u_c[i] = control(tt[i], sol1[i,:])
# reference trajectory
y1D = Traj(1, tt, c_g)
y2D = Traj(2, y1D[:,0], c_f)
x_ref = np.zeros_like(sol1)
x_ref[:,0] = y1D[:,0]
x_ref[:,1] = y2D[:,0]
x_ref[:,2] = arctan(y2D[:,1])

#plot the trajectory
plt.figure(1)
plt.plot(tt, sol1[:,0:2], tt, x_ref[:,0:2])
plt.title('Positon coordinates')
plt.legend([r'$x(t)$', r'$y(t)$', '$x_d(t)$', '$y_d(t)$'])
plt.ylabel('position (m)', size=15)
plt.xlabel(r't in s')
plt.grid(True)
plt.show()
plt.close()

plt.figure(2)
plt.plot(tt, np.rad2deg(sol1[:,2]), tt, np.rad2deg(x_ref[:,2]))
plt.title('orientation')
plt.legend([r'$\theta(t)$', r'$\theta_d(t)$'])
plt.ylabel('degree', size=15)
plt.xlabel(r't in s')
plt.grid(True)
plt.show()
plt.close()

plt.figure(3)
plt.plot(tt, u_c[:,0])
plt.title('velocity')
plt.ylabel('m/s', size=15)
plt.xlabel(r't in s')
plt.grid(True)
plt.show()
plt.close()

plt.figure(4)
plt.plot(tt, np.rad2deg(u_c[:,1]))
plt.title('steering angle')
plt.ylabel('degree', size=15)
plt.xlabel(r't in s')
plt.grid(True)
plt.show()
plt.close()




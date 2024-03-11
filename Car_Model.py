#Author: Xin Lyu
#car-like mobile robot



import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

#kinematics of car-like mobile robot

# Physical parameter
l = 0.3         # define car length

# Simulation parameter
t0 = 0          # start time
tf = 10         # final time
dt = 0.04       # step-size

def solve_car(x, t):
    x1, x2, x3 = x  # state vector
    u1 = 1.0-0.1*t
    u2 = 0.25
    return [u1 * np.cos(x3), u1 * np.sin(x3), 1.0/l * u1 * np.tan(u2)]


tt = np.arange(t0, tf +dt, dt)
# initial state
x0 = [0, 0, 0]

sol = odeint(solve_car, x0, tt)


xx = sol[:,0]
yy = sol[:,1]
phi = sol[:,2]

plt.figure()
plt.plot(tt, xx, 'r-', label='x')
plt.plot(tt, yy, 'b-', label='y')
plt.plot(tt, 0*tt, 'k--')
plt.title('Position coordinates', size = 15)
plt.ylabel('position (m)', size=15)
plt.xlabel('time(s)', size=15)
plt.legend(loc=0, fontsize =15)
plt.show()
plt.close()
    
plt.figure()
plt.plot(tt, np.rad2deg(phi), 'g-', label=r'$\theta(t)$')
plt.title('orientation', size = 15)
plt.ylabel('degree', size=15)
plt.xlabel('time(s)', size=15)
plt.legend(loc=0, fontsize =15)
plt.show()
plt.close()


   

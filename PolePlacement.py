# -*- coding: utf-8 -*-
#Author: Xin Lyu
#Topic: Pole placement method on pendulum with torque
#Design requirements: stabilizing the pendulum at an angle theta_d
#Constraint: The torque must have steday state component Tss
# T = u + a/c * math.sin(theta_d)


import math
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import sympy as sp
from scipy.integrate import odeint

theta_d = math.pi/4
a = 9.81
b = 1
c = 1
x1 = sp.Symbol('x1')
x2 = sp.Symbol('x2')
f1 = x2
f2 = -a*sp.sin(x1+theta_d)-b*x2+a*sp.sin(theta_d)   
# Linearization
A11 = sp.diff(f1,x1)
A12 = sp.diff(f1,x2)
A21 = sp.diff(f2,x1)
A22 = sp.diff(f2,x2)
# Calculate numerical A
A11_N = A11.subs([(x1, 0), (x2, 0)])
A12_N = A12.subs([(x1, 0), (x2, 0)])
A21_N = A21.subs([(x1, 0), (x2, 0)])
A22_N = A22.subs([(x1, 0), (x2, 0)])

A = np.zeros((2,2))
B = np.zeros((2,1))

A[0,0] = A11_N
A[0,1] = A12_N
A[1,0] = A21_N
A[1,1] = A22_N

B[1,0] = c
print(A)
print(B)
P = np.array([-3,-4])
# Check Controllability matrix [B AB A^2B ...]

# Design a controller ploe placement
res = signal.place_poles(A, B, P, method='KNV0')
print(res.gain_matrix)
k1_n = res.gain_matrix[0,0]
k2_n = res.gain_matrix[0,1]
k1_n = -5
k2_n = 1
# x1 = e-theta_d x2 = edot=theta_dot
# u = -k1x1-k2x2 = -k1(theta-theta_d)-k2*theta_dot

def dynamic(x,t):
    x1,x2= x
    dx1 = x2
    dx2 = -a*math.sin(x1)-b*x2-c*k1_n*(x1-theta_d)-c*k2_n*x2+a*math.sin(theta_d)
    return [dx1,dx2]
t = np.linspace(0,10)

f0=[0,0]

s = odeint(dynamic,f0,t)

plt.plot(t,s[:,0],'r--', linewidth=2.0)
plt.plot(t,s[:,1],'b-', linewidth=2.0)
plt.legend([r'$\theta(t)$', r'$\theta_d(t)$'])
plt.grid(True)
plt.show()



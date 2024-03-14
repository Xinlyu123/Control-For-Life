#  Author : Xin Lyu
#  LQR implementation

# exmaple: spring damper system
# mx_dd = f-bx_d-kx
import numpy as np
import matplotlib.pyplot as plt
m =1 # kg
k =1 # N/m
b = 0.5 #Ns/m
x_d = np.array([[0],[0]])
# define Q matrix
Q = np.identity(2)
#define R
R = 0.1
A = np.array([[0 ,1],[-1 ,-0.5]])
B = np.array([[0],[1]])
k_steps = 100
# state dimension
n = A.shape[0]
p = B.shape[1]
# discretize the system
delta_t = 0.1
#A_d = np.identity(2)+delta_t*A
#B_d = delta_t*B
A_d = np.array([[0.995,0.0974],[-0.0974 ,0.9464]])
B_d = np.array([[0.0049],[0.0974]])
# intiallize state and define x,u trajectory
x_init = np.array([[1],[0]])
u_init = 2
x = x_init
x_history = np.zeros((n,k_steps))
x_history[:,0] = x.flatten()
u = u_init
u_history = np.zeros([p,k_steps])
u_history[:,0] = u


timeline = np.arange(k_steps)
# calculate F
P_k = Q
F_history = []
for k in range(k_steps-1):
    # calculate F[N-k] where N is how many steps
    S = np.matmul(np.matmul(np.transpose(B_d),P_k),B_d)
    S1 = np.matmul(np.matmul(np.transpose(B_d),P_k),A_d)
    F = np.matmul(np.linalg.inv(R+S),S1)
    S2 = A_d-np.matmul(B_d,F)
    S3 = np.matmul(np.matmul(np.transpose(S2),P_k),S2)
    S4 = np.matmul(np.transpose(F)*R,F)
    P_k = S3+S4+Q

    P_k = np.transpose(A_d-B_d*F)*P_k*(A_d-B_d*F)+np.transpose(F)*R*F+Q
    F_history.insert(0,F.flatten())


for k in range(k_steps-1):
    # calculate u
    u = -np.matmul(F_history[k],x)
    # calculate x
    x = np.matmul(A_d,x)+B_d*u

    x_history[:,k+1]= x.flatten()
    u_history[:,k+1] = u

fig,axs = plt.subplots(2,1,figsize=(8,6))
axs[0].plot(timeline,x_history[0,:],label='x1')
axs[0].plot(timeline,x_history[1,:],label='x2')
axs[0].set_ylabel('x')
axs[0].legend()

axs[1].plot(timeline,u_history[0,:])
axs[1].set_ylabel('u')
axs[1].set_xlabel('Time')
axs[1].legend('u')

plt.tight_layout
plt.show()
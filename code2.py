# Exercise: Heated Tanks (inspired by Doyen 2018) 
import numpy as np
import matplotlib.pyplot as pt
import control as ct
from control.matlab import *
pt.close('all')
fig, ax = pt.subplots(4,1)

################################################################################
# fix model parameters
a1 = a2 = 0.01     # heat transfer to environment
b1 = b2 = 0.005    # heat transfer between tanks
# fix controller parameters
kp1 = kp2 = 1      # proportional gain
Ti1 = Ti2 = 1      # integral
Td1 = Td2 = 1      # derivative
T1  = T2  = 1      # derivative delay
# simulation parameters
x0 = [0, 50]       # initial state
t0, tfin = 0, 700  # simulation period

################################################################################
# set up state-space model of the open-loop system
A = [[-a1, b1], [b2, -a2]]
B = [[1, 0], [0, 0]]
C = [[1, 0], [0, 1]] # direct state output
D = [[0, 0], [0, 0]] # no disturbance
modeL1 = ss(A, B, C, D, inputs=['h1', 'h2'], states=['x1', 'x2'], outputs=['x1', 'x2'])

# investigate step response of the open-loop system
y, t = step(modeL1)
ax[0].set(title='Step response (open loop)')
ax[0].plot(t, y[:,0], 'r-', label=r'x$_1$')
ax[0].plot(t, y[:,1], 'b-', label=r'x$_2$')
dcgain(modeL1)

# simulate open-loop system from x0 for a given period and step stimulus
h1 = h2 = 2      
t = np.linspace(t0, tfin, 100)
u = np.transpose([h1*np.ones(t.size), h2*np.ones(t.size)])
y, t, x = lsim(modeL1, u, t, x0)
ax[1].set(title='Simulation (open loop)')
ax[1].plot(t, y[:,0], 'g-', label=r'x$_1$')
ax[1].plot(t, y[:,1], 'b-', label=r'x$_2$')

################################################################################
# design P controllers and closed-loop system
w = np.transpose([70*np.ones(t.size),30*np.ones(t.size)])
P1 = tf(kp1, 1)
P2 = tf(kp2, 1)
P = ct.append(P1, P2) # MIMO P controller
print(P)
OpenLoop = series(P, modeL1)
PClosedLoop = feedback(OpenLoop, OpenLoop, -1) # unity feedback
y, t, x = lsim(PClosedLoop, w, t, [0,50,0,0])
ax[2].set(title='Simulation (closed loop, with P law)')
ax[2].plot(t, y[:,0], 'g-', label=r'x$_1$')
ax[2].plot(t, y[:,1], 'b-', label=r'x$_2$')

################################################################################
# design LQR controller
Qx = np.diag([1, 1])  # state weighting matrix
Ru = Qx # np.diag([1, 1])  # input weighting matrix
K, P, E = lqr(A, B, Qx, Ru) # obtain LQR law u=-K*x
LQRClosedLoop = ss(A-B*K,B*K,C,D)
y, t, x = lsim(LQRClosedLoop, w, t, x0)
ax[3].set(title='Simulation (closed loop with MIMO/coupled LQR law)')
ax[3].plot(t, y[:,0], 'g-', label=r'x$_1$')
ax[3].plot(t, y[:,1], 'b-', label=r'x$_2$')

# decorate and show all plots
for a in ax:
    a.set(xlabel='Time (s)', ylabel=r'Temperatures (\textdegree C)', xlim=(0,tfin))
    a.grid(color='lightgray', linestyle='--')
    a.legend()
pt.show()

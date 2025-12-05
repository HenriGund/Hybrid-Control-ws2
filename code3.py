#!/usr/bin/python
# Exercise: LQ Optimal Control (inspired by Lunze 2020, Lin 2022) 
import numpy as np
import matplotlib.pyplot as pt
import control as ct
from control.matlab import * 
pt.close()
fig, ax = pt.subplots(1,1)

# inverted pendulum parameters
g = 9.81 # m/s^2
l = 3*g/(2*21.5)
m = 3/(0.6*l**2)
# state-space model of the *linearised* inverted pendulum
A = np.array([[0, 1], [(3*g)/(2*l), 0]]) # given in the question
B = np.array([[0], [3/(m*l**2)]])  # given in the question - 2x1, so only 1 input
C = np.array([[1, 0], [0, 1]]) # given in the question
D = np.zeros((2, 1))


# optimisation parameters
dt = 0.005 # precision
tf = 3.25 # time span
t = np.arange(0,tf,dt) # sampling points
# Q = np.eye(2)
Q = np.diag([800.0, 1.0]) # state cost matrix
R = np.array([10]) # input cost matrix
x0 = [[-.52], [0]] # initial state [rad, rad/s]

# build open-loop model of the inverted pendulum
InvPendSimple = ss(A, B, C, D, states=['$\phi$', '$\dot\phi$'], inputs=['u'], outputs=['$\phi$', '$\dot\phi$'])

# find optimal (LQ) controller
xe = [[0.5], [0]] # equilibrium state
ue = [[0]] # equilibrium input
print("Rank", np.linalg.matrix_rank(ct.ctrb(A, B))) # controllable?
print(np.linalg.eig(Q)) # definiteness (all eigenvalues real pos/neg)
K, P, E = lqr(A, B, Q, R); # solve Riccati DE and get optimal controller
LQRClosedLoop = ss(A - B @ K, B, C, D)
u = np.zeros((len(t), 1))
y, t, x = lsim(LQRClosedLoop, u, t, x0)
ax.set(title='Simulation (closed loop, inverted pendulum)')
ax.plot(t, y[:,0], 'g-', label='phi [rad]')
ax.plot(t, y[:,1], 'b-', label='dotphi [rad/s]')
ax.set(xlabel='Time', ylabel='Deviation from equilibrium', xlim=(0,tf))
ax.grid(color='lightgray', linestyle='--')
ax.legend()
pt.show()

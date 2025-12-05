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

q_phi_values  = [0.1, 1, 5, 10, 15, 20, 35, 50, 60, 100]
q_dphi_values = [0.01, 0.1, 0.5, 1, 5, 10, 20, 50, 100]
r_values      = [0.01, 0.1, 0.5, 1, 5, 10, 20, 30, 50]


# optimisation parameters
dt = 0.005 # precision
tf = 3.25 # time span
t = np.arange(0,tf,dt) # sampling points
# Q = np.diag([100.0, 0.02]) # state cost matrix
# R = np.array([1]) # input cost matrix
x0 = [[-.52], [0]] # initial state [rad, rad/s]

results = {'index':[], 'overshoot': [], 'overshooting_v':[], 'settling_time': []}


for q_phi in q_phi_values:
    for q_dphi in q_dphi_values:
        for r in r_values:
            index = str(q_dphi) + '_' + str(q_phi) + '_' + str(r)
            Q = np.diag([q_phi, q_dphi])
            R = np.array([r])

            # build open-loop model of the inverted pendulum
            InvPendSimple = ss(A, B, C, D, states=['$\phi$', '$\dot\phi$'], inputs=['u'], outputs=['$\phi$', '$\dot\phi$'])

            # find optimal (LQ) controller
            xe = [[0], [0]] # equilibrium state
            ue = [[0]] # equilibrium input
            # print("Rank", np.linalg.matrix_rank(ct.ctrb(A, B))) # controllable?
            # print(np.linalg.eig(Q)) # definiteness (all eigenvalues real pos/neg)
            K, P, E = lqr(A, B, Q, R); # solve Riccati DE and get optimal controller
            LQRClosedLoop = ss(A - B @ K, B, C, D)
            u = np.zeros((len(t), 1))
            y, t, x = lsim(LQRClosedLoop, u, t, x0)
            # ax.set(title='Simulation (closed loop, inverted pendulum)')
            # ax.plot(t, y[:,0], 'g-', label='phi [rad]')
            # ax.plot(t, y[:,1], 'b-', label='dotphi [rad/s]')
            # ax.set(xlabel='Time', ylabel='Deviation from equilibrium', xlim=(0,tf))
            # ax.grid(color='lightgray', linestyle='--')
            # ax.legend()
            # pt.show()
            
            overshoot = (np.max(y[:,0]) - 0)/0.52 * 100 # in percentage of initial condition
            overshooting_v = np.max(y[:,1])
            settling_index = np.where(np.abs(y[:,0]) <= 1e-5)[0]
            settling_time = t[settling_index[0]] if settling_index.size > 0 else tf
            results['index'].append(index)
            results['overshoot'].append((overshoot))
            results['overshooting_v'].append((overshooting_v))
            results['settling_time'].append((settling_time))


for i in range(len(results['index'])):
    qphi = results['index'][i].split('_')[1]
    qdphi = results['index'][i].split('_')[0]
    r = results['index'][i].split('_')[2]

    print(f'Results for Q = diag([{qphi}, {qdphi}]) and R = [{r}]:')

    print("Index:", results['index'][i], 
          "| Overshoot (%):", results['overshoot'][i], 
            "| Overshooting velocity (rad/s):", results['overshooting_v'][i],
          "| Settling time (s):", results['settling_time'][i])
    print('-----------------------------------------')


settling_limit = 1 # seconds
overshoot_v_limit = 1.6 # rad/s
print(f"\nCombinations that meet the performance criteria (settling time < {settling_limit}s and overshooting velocity < {overshoot_v_limit} rad/s):")

for i in range(len(results['index'])):
    if results['settling_time'][i] < settling_limit and results['overshooting_v'][i] < overshoot_v_limit:
        qphi = results['index'][i].split('_')[1]
        qdphi = results['index'][i].split('_')[0]
        r = results['index'][i].split('_')[2]
        print(f'Q = diag([{qphi}, {qdphi}]), R = [{r}] --> Settling time: {results["settling_time"][i]} s, Overshooting velocity: {results["overshooting_v"][i]} rad/s')


Q = np.diag([35, 0.01])
R = np.array([0.01])

# build open-loop model of the inverted pendulum
InvPendSimple = ss(A, B, C, D, states=['$\phi$', '$\dot\phi$'], inputs=['u'], outputs=['$\phi$', '$\dot\phi$'])

# find optimal (LQ) controller
xe = [[0], [0]] # equilibrium state
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
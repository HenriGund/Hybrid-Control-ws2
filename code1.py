# Room Heating Control (inspired by Lunze 2020) 
import numpy as np
import matplotlib.pyplot as pt
import control as ct
from control.matlab import * # ss, step, rss, dcgain
pt.close('all')

# define state-space model of heated room
A = [-0.2] # system matrix
B = [0.2]  # input matrix
C = [5]    # output matrix
D = [0]    # disturbance matrix

# controller parameters
kp = 20
Ti = 0.1
Td = 15
T  = 50
# system parameters
ks = 10
y0 = 2.0
x0 = [1 / ks * y0] # initial state
t0, tfin = 0, 50
S = np.linspace(t0, tfin, 100) # sample series

################################################################################
# design system and P|I|D controllers
HeatedRoom = ss(A, B, C, D)

P = tf(kp,1)
POpenLoop = series(P, HeatedRoom)

I = tf(1, [Ti, 0])
D = tf([Td, 0], [T, 1])

PI = parallel(P, I) # we add both elements to get PI controller

PIOpenLoop = series(PI, HeatedRoom)

PID = parallel(P, I, D) # we add D element to get PID controller
PIDOpenLoop = series(PID, HeatedRoom)

PD = parallel(P, D) # we add P and D elements to get PD controller
PDOpenLoop = series(PD, HeatedRoom)

# ClosedLoop = feedback(OpenLoop, 1)

# simulate response to temperature set-point 25 degc
wref = 25
w = wref * np.ones(S.size) # set-point step stimulus
pt.axhline(y=w[0], color='r', linestyle='--')
pt.axvline(x=10, color='r', linestyle='--')
x0 = [3] # initial P state
y, t, x = lsim(feedback(POpenLoop, 1), w, S, x0)
pt.plot(t, y, linewidth=2.0, color='orange', label='P')
x0 = [0,3] # initial Px state
y, t, x = lsim(feedback(PIOpenLoop, 1), w, S, x0)
pt.plot(t, y, linewidth=2.0, color='blue', label='PI')
x0 = [0,0,3] # initial Pxy state
y, t, x = lsim(feedback(PIDOpenLoop, 1), w, S, x0)
pt.plot(t, y, linewidth=2.0, color='green', label='PID')
x0 = [0,3] # initial Pxy state
y, t, x = lsim(feedback(PDOpenLoop, 1), w, S, x0)
pt.plot(t, y, linewidth=2.0, color='red', label='PD')
pt.grid(color='grey', linestyle='--')
pt.legend()
pt.show()

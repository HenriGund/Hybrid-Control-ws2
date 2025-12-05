# Group: Henrique Gundlach and Lara Polachini
## Contrôle optimal et systèmes hybrides - WS 2
---

# Question 1

### (a) Gradually enhance your P controller towards a SISO PID controller.

Done in class

### (b) and (c) Comparison of controls and parameters modifications

The results of all the controllers (P, PD, PI and PID) are shown in the figure:

![alt text](image.png)

The figure shows the responses of the P, PD, PI, and PID controllers. As expected, the P and PD controllers stabilize below the desired temperature (25 °C). This happens because neither P nor PD contains integral action, so they cannot eliminate the steady-state error. In contrast, the PI and PID controllers reach the correct setpoint, although with different overshoot and settling times due to the different contributions of integral and derivative terms.

We modified the parameters considering that: Kp controls the speed in which the system gets stable; Kd controls the variance in error, avoids oscilations; Ki adapts to the correct point. Making progressive corrections, the figure below shows the responses of all four controllers (P, PI, PD, and PID) using the tuned parameters (Kp = 20, Ti = 0.1, Td = 15). With these values, the controllers exhibit significantly improved behavior compared to the initial tests.

![alt text](image-1.png)

As expected, the P and PD controllers still stabilize slightly below the setpoint. This happens because neither controller includes an integral term, so a small steady-state error always remains in systems of this type. However, with the high Kp and large static gain, the residual error becomes very smaller.

---

# Question 2

The script begins by defining the physical parameters of the heated-tanks model (a1, a2, b1, b2), the controller settings (kp1, kp2, etc.), the initial temperatures x0 = [0, 50], and the simulation interval from t0 to tfin. These serve as the inputs to the model. With these values, the code builds a 2-state state-space representation modeL1 capturing the temperature dynamics of the two coupled tanks. It first evaluates the open-loop step response of this model and simulates the temperatures when both heaters receive constant inputs (h1 = h2 = 2). Next, two proportional controllers are created using transfer functions P1 and P2, combined into a MIMO P-controller with ct.append. A closed-loop system is then formed using unity feedback, and its response to a reference signal w = [70, 30]ᵀ is simulated. Finally, the script designs an LQR controller by selecting weighting matrices Qx and Ru, computing the optimal gain K, and simulating the resulting state-feedback closed-loop system.

The expected output is four subplots that show how the tank temperatures evolve under different scenarios. The first subplot displays the open-loop step response, highlighting how each tank’s temperature responds independently to a step in the heater input. The second subplot shows the open-loop simulation under constant heating, revealing the natural slow warming dynamics and inter-tank coupling. The third subplot presents the closed-loop behaviour under proportional control, where the temperatures attempt to track the reference values but exhibit oscillations. The final subplot shows the closed-loop LQR response, where the optimised state-feedback controller drives both tanks smoothly toward the desired temperatures.

Together, these plots match the expected behaviour for the heated-tanks system under the given control strategies. The open-loop results confirm that the system is inherently slow and coupled, making precise temperature regulation difficult without feedback. The proportional controller improves tracking but introduces oscillatory behaviour due to its limited ability to manage the interdependence between the tanks. In contrast, the LQR controller delivers the expected superior performance: smooth, fast, and stable convergence to the reference temperatures. This demonstrates how multivariable optimal control more effectively handles the coupling and dynamics of the two-tank system.

![alt text](image-2.png)

---

# Question 3

The first result for this question is shown in the figure below:

![alt text](image-3.png)

The code first constructs a linearized state-space model of the inverted pendulum around its upright equilibrium using the matrices $A$ and $B$ provided in the problem. The two states are the angular position $\phi$ and angular velocity $\dot{\phi}$. An LQ cost is defined with $Q = I_2$, penalizing both states equally, and $R = 1$, which penalizes the control effort. The lqr(A, B, Q, R) function computes the optimal feedback gain $K$, and the closed-loop system is formed as $A_{\text{cl}} = A - BK$. The simulation then integrates this closed-loop system starting from the initial state $x_0 = [-0.52,\ 0]^T$ with zero external input. Both states are used as outputs so that the evolution of angle and angular velocity can be plotted.

In the generated plot, the angle $\phi$ (green curve) begins at $-0.52$ rad and smoothly converges toward zero, showing that the LQR controller successfully stabilizes the pendulum in the upright configuration. The angular velocity $\dot{\phi}$ (blue curve) displays an initial peak, corresponding to the corrective action of the controller, and then decays exponentially back to zero. Overall, the response is stable, well-damped, and free of persistent oscillations, with only a small overshoot in velocity and a settling time of a few seconds. This behavior aligns with what is expected from an LQR-controlled linear system with moderate weighting on both state deviation and control effort.

Updating variables R and Q (increasing Q and decreasing R), we arrived to a system that gets stable faster, before 1 seconds, with the cost of more overshooting in the angular velocity. 

![alt text](image-4.png)

In order to test more values for R and Q, a series test was developed, considering:

q_phi_values  = [0.1, 1, 5, 10, 15, 20, 35, 50, 60, 100]

q_dphi_values = [0.01, 0.1, 0.5, 1, 5, 10, 20, 50, 100]

r_values      = [0.01, 0.1, 0.5, 1, 5, 10, 20, 30, 50]

With a logic of saving all results and calculating the settling time and overshooting for angular velocity, we've got, with the used values, the following results:

Combinations that meet the performance criteria (settling time < 1s and overshooting velocity < 1.6 rad/s):

Q = diag([35, 0.1]), R = [0.01] --> Settling time: 0.88 s, Overshooting velocity: 1.335514782392843 rad/s

Q = diag([50, 0.1]), R = [0.01] --> Settling time: 0.75 s, Overshooting velocity: 1.4509671538183238 rad/s

Q = diag([50, 0.5]), R = [0.01] --> Settling time: 0.925 s, Overshooting velocity: 1.4073125923375913 rad/s

Q = diag([60, 0.01]), R = [0.01] --> Settling time: 0.67 s, Overshooting velocity: 1.5270289370929282 rad/s

Using the first result, the graphic looks like the one shown below:

![alt text](image-5.png)


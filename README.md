# Autonomous Vehicle Trajectory Optimization under Friction Constraints

An end-to-end trajectory generation and validation engine for emergency high-speed lane-change maneuvers using quintic (5th-degree) parametric polynomials. This repository models the boundaries where vehicle kinematics, tire-road surface friction, and human comfort parameters intersect under realistic operational conditions.

## 📌 Project Overview
When an autonomous vehicle executes a sudden obstacle-avoidance maneuver at high speeds ($25 \text{ m/s}$ / $\approx 90 \text{ km/h}$), path planning cannot rely on simple linear geometry. Discontinuous paths generate infinite lateral jerk ($\dot{a}_y = \infty$), triggering suspension instability and a catastrophic loss of control.

This project implements a **quintic polynomial trajectory planner** to enforce strict boundary conditions across position ($y$), heading ($y'$), and curvature ($y''$). It then executes a parametric sweep to find the absolute minimum safe distance required to shift lanes without breaking tire adhesion or causing passenger discomfort.

---

## 🏎️ Kinematic and Design Constraints

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Longitudinal Velocity ($v_x$)** | $25.0 \text{ m/s}$ | Constant tracking speed ($\approx 90 \text{ km/h}$) |
| **Lateral Displacement ($D$)** | $3.6 \text{ m}$ | Standard highway lane width |
| **Max Lateral Acceleration ($a_{y,\max}$)** | $\le 4.0 \text{ m/s}^2$ | Tire friction ceiling on dry asphalt |
| **Max Lateral Jerk ($j_{y,\max}$)** | $\le 2.0 \text{ m/s}^3$ | Emergency threshold (IRC benchmarks recommend $\le 0.6 \text{ m/s}^3$ for nominal ride quality) |

---

## 🧮 Mathematical Foundation

The lateral path is governed by a 5th-degree polynomial:
$$y(x) = c_0 + c_1x + c_2x^2 + c_3x^3 + c_4x^4 + c_5x^5$$

Enforcing static boundary conditions at $x=0$ yields immediate values for the initial states:
* $y(0) = 0 \implies c_0 = 0$
* $y'(0) = 0 \implies c_1 = 0$
* $y''(0) = 0 \implies c_2 = 0$

Evaluating the remaining constraints at the end of the maneuver ($x = X_f$), we translate the non-trivial coefficients into a solvable linear matrix system ($A \cdot C = B$):

$$\begin{bmatrix} 
X_f^3 & X_f^4 & X_f^5 \\ 
3X_f^2 & 4X_f^3 & 5X_f^4 \\ 
6X_f & 12X_f^2 & 20X_f^3 
\end{bmatrix} 
\begin{bmatrix} c_3 \\ c_4 \\ c_5 \end{bmatrix} = 
\begin{bmatrix} D \\ 0 \\ 0 \end{bmatrix}$$

---

## 💻 Implementation Structure

The trajectory planning logic is written in an object-oriented Python structure using NumPy for matrix operations and Matplotlib for parametric plotting.

```python
import numpy as np
import matplotlib.pyplot as plt

class TrajectoryGenerator:
    def __init__(self, lane_width=3.6, velocity=25.0):
        self.D = lane_width
        self.vx = velocity
        self.C = None
        
    def solve_coefficients(self, xf):
        x0 = 0
        A = np.array([
            [1, x0,   x0**2,   x0**3,    x0**4,    x0**5],
            [1, xf,   xf**2,   xf**3,    xf**4,    xf**5],
            [0,  1, 2*x0,   3*x0**2,  4*x0**3,  5*x0**4],
            [0,  1, 2*xf,   3*xf**2,  4*xf**3,  5*xf**4],
            [0,  0,    2,   6*x0,    12*x0**2, 20*x0**3],
            [0,  0,    2,   6*xf,    12*xf**2, 20*xf**3]
        ])
        B = np.array([0, self.D, 0, 0, 0, 0])
        self.C = np.linalg.solve(A, B)
        return self.C

    def verify_safety(self, xf):
        x_vals = np.linspace(0, xf, 100)
        c3, c4, c5 = self.C[3], self.C[4], self.C[5]
        
        # Spatial-to-temporal transformation via chain rule
        ay_vals = (self.vx**2) * (6*c3*x_vals + 12*c4*x_vals**2 + 20*c5*x_vals**3)
        jerk_vals = (self.vx**3) * (6*c3 + 24*c4*x_vals + 60*c5*x_vals**2)
        
        max_ay = np.max(np.abs(ay_vals))
        max_jy = np.max(np.abs(jerk_vals))
        
        return max_ay <= 4.0, max_ay, max_jy <= 2.0, max_jy

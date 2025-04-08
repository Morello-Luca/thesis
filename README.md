### Human-like motion planning and sensorless control framework for bimanual grasping of cumbersome objects

## Multi-Robotic Arm Coordination for Object Manipulation

In scenarios involving complex object manipulation, tasks such as handling cumbersome objects often require multiple robotic arms working in synergy. This project implements a modular control law within a multi-manual framework, with a focus on tasks that involve picking up unknown objects. Drawing inspiration from force and impedance control principles, a novel adaptation policy is introduced to address the limitations of conventional methods, such as coupled impedance and hybrid position/force control. The system uses human-like motion planning, relying on data from a single RGB-D camera.

### Key Features:

- **Point Cloud Processing**: The generated point cloud is rigorously filtered and evaluated to handle incomplete surface coverage. Key geometric parameters, such as the dimensions of bounding boxes and relative contact points, are extracted for precise manipulation.
  
- **Experimental Setup**: The system utilizes two Franka Emika robots. The workflow is divided into three phases:
  1. **Initial Phase**: The object is scanned to locate potential contact regions.
  2. **Contact Phase**: An impedance control policy is applied for a compliant approach.
  3. **Post-Contact Phase**: A hybrid control policy lifts and manipulates the object.

![Insert experimental setup image here](path/to/image.png)

---

## Modular Control Strategy

The implemented control strategy allows switching between compliant behavior during contact and stable manipulation during lifting, without the need for force sensors.

### Impedance Control

To ensure compliant interaction during object approach, the system uses an impedance control law:

$$
F_{cmd} = K (x_d - x) + D (\dot{x}_d - \dot{x})
$$

Where:
- \( F_{cmd} \) is the commanded force,
- \( K \) is the stiffness matrix,
- \( D \) is the damping matrix,
- \( x_d \) and \( x \) are the desired and current positions, respectively.

### Adaptive Stiffness

To resolve conflicts between motion and contact force directions, the stiffness is adapted as:

$$
eeK_C = \text{diag}([k_{t,x},\, k_{t,y},\, \rho_{imp}\,k_{t,z},\, k_{r,x},\, k_{r,y},\, k_{r,z}])
$$

with

$$
\rho_{imp} =
\begin{cases}
1, & \text{if } \delta_{imp} \leq ee\tilde{x}_z \\
0.5\Big(1 - \cos\Big(\frac{\pi\,ee\tilde{x}_z}{\delta_{imp}}\Big)\Big), & \text{if } 0 \leq ee\tilde{x}_z < \delta_{imp} \\
0, & \text{otherwise}
\end{cases}
$$

### Force Control

For stable object manipulation, a force controller maintains the desired contact force:

$$
\tau_{f\_rc} = J_{ee}^T (q) 
\begin{pmatrix}
0 \\[8pt]
0 \\[8pt]
\rho_{frc}\,f_{eff\_rc}
\end{pmatrix}
$$

Where the effective force is given by:

$$
f_{eff\_rc} = eefd + k_p\,ee\tilde{f}_{ext} + k_i\int ee\tilde{f}_{ext}\,dt + k_d\,\dot{ee\tilde{f}}_{ext},
$$

with

$$
ee\tilde{f}_{ext} = eefd + eef_{ext,z}.
$$

A safeguard variable \( \rho_{frc} \) is used to disable force control when positional errors are large.

---

## Human-Like Motion Planning

Human-like trajectories are generated using functional Principal Component Analysis (fPCA). The motion is approximated as:

$$
x(t) \approx \bar{x} + S_0(t) + \sum_{i=1}^{5} \alpha_i\,S_i(t)
$$

Where:
- \( \bar{x} \) is the average pose,
- \( S_0(t) \) is the average trajectory,
- \( S_i(t) \) are the basis functions (fPCs),
- \( \alpha_i \) are the weighting coefficients.

The planned trajectory is computed by solving a constrained system that satisfies the boundary conditions on position, velocity, and acceleration.

---

## Conclusion and Future Works

This project demonstrates that multi-robot coordination, when combined with adaptive impedance and force control, enables robust manipulation of cumbersome objects. Future enhancements will focus on:
- Improved integration of orientation dynamics,
- Better force estimation techniques, and
- Extended vision processing to handle dynamic environments.

---

## References

1. Uchiyama, M. & Dauchez, P. (1992). *Symmetric kinematic formulation and non-master/slave coordinated control of two-arm robots*. Advanced Robotics, 7(4), 361–383.
2. Nakano, E. (1974). *Cooperational control of the anthropomorphous manipulator*. Proc. 4th Int. Symp. Industrial Robots.
3. Caccavale, F. & Uchiyama, M. (2016). *Cooperative manipulation*. Springer Handbook of Robotics, 989–1006.
4. Shahriari, E., Birjandi, S. A. B., & Haddadin, S. (2022). *Passivity-based adaptive force-impedance control for modular multi-manual object manipulation*. IEEE Robotics and Automation Letters, 7(2), 2194–2201.
5. Dehio, N., et al. (2022). *Enabling impedance-based physical human–multi–robot collaboration*. International Journal of Robotics Research, 41(1), 68–84.
6. Bouyarmane, K., et al. (2018). *Quadratic programming for multirobot and task-space force control*. IEEE Transactions on Robotics, 35(1), 64–77.
7. Hogan, N. (1984). *Impedance control of industrial robots*. Robotics and Computer-Integrated Manufacturing, 1(1), 97–113.
8. De Luca, A., et al. (2006). *Collision detection and safe reaction with the DLR-III lightweight manipulator arm*. IEEE/RSJ International Conference on Intelligent Robots and Systems.
9. De Luca, A. & Mattone, R. (2005). *Sensorless robot collision detection and hybrid force/motion control*. IEEE International Conference on Robotics and Automation.
10. Haddadin, S. (2005). *Evaluation criteria and control structures for safe human-robot interaction*. PhD Dissertation, TUM & DLR.

# Human-like motion planning and sensorless control framework for bimanual grasping of cumbersome objects
![image](https://github.com/user-attachments/assets/7acff120-92bb-435b-aad3-098cbb71a09d)


## Object Manipulation Control Law

Object manipulation can be broken down into two primary components:
1. **Object Movement**: Controlling the movement of the object within the desired workspace.
2. **Gripping the Object**: Maintaining a firm grip on the object while it is in motion to prevent slippage or loss of control.

## Hybrid Control

Hybrid control integrates two distinct control strategies to achieve effective manipulation:

- **Force Control**: This approach focuses on controlling the contact wrench (the combination of force and torque) applied to the object, ensuring a stable grip and preventing any unwanted slippage during manipulation.

- **Impedance Control**: This method is used to regulate the motion of the object, by controlling its dynamics (position, velocity, and force) while interacting with the environment. It aims to provide a smooth response to external disturbances, ensuring controlled movement.




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

## Main Limitation of Force/Motion Hybrid Control

One of the primary limitations of force/motion hybrid control is the potential **conflict between the direction of the desired force and the desired motion**.

### Example Scenario

Consider a situation where the desired motion of the object is in the **opposite direction** of the desired grasp force. In this case:

- Following the desired motion might result in **losing contact** with the object.
- This loss of contact could lead to **grasp failure**, as the object may slip or become unstable.

This conflict between force and motion directions can present challenges in achieving stable and effective object manipulation.
![image](https://github.com/user-attachments/assets/f33e2382-6580-4fe5-9cac-47dac9b9ab31)
## Desired Trajectory in Task Frame

The term $$^{𝑇𝐹𝑥}_𝑑 $$ represents the desired trajectory of the object defined in a **task frame**.

### Objective

The objective is to establish a **shared point of reference** that allows multiple manipulators to work together without needing to coordinate the movements of each arm in relation to the others. Instead, the focus is on:

- **Controlling the pose of the object** rather than the individual movements of each manipulator.
- Ensuring that the object follows the desired trajectory **within the task frame** while maintaining coordination among the manipulators.



### Human-like motion planning and sensorless control framework for bimanual grasping of cumbersome objects

## Multi-Robotic Arm Coordination for Object Manipulation

In scenarios involving complex object manipulation, tasks such as handling cumbersome objects often require multiple robotic arms working in synergy. This project implements a modular control law within a multi-manual framework, with a focus on tasks that involve picking up unknown objects. Drawing inspiration from force and impedance control principles, a novel adaptation policy is introduced to address the limitations of conventional methods, such as coupled impedance and hybrid position/force control. The system uses human-like motion planning, relying on data from a single RGB-D camera.

### Key Features:

- **Point Cloud Processing**: The generated point cloud is rigorously filtered and evaluated to handle incomplete surface coverage. Key geometric parameters, such as the dimensions of bounding boxes and relative contact points, are extracted for precise manipulation.
  
- **Experimental Setup**: The system utilizes two Franka Emika robots. The workflow is divided into three phases:
  1. **Initial Phase**: The system assesses the object to identify critical contact areas required for successful manipulation.
  2. **Contact Phase**: An impedance control policy is applied to ensure stable and responsive contact during manipulation.
  3. **Post-Contact Phase**: A hybrid control policy is used to lift and manipulate the object effectively.

## Introduction

Robots are increasingly being integrated into human environments, with industries expecting humanoid robots to replace human workers without the need for workspace redesign. In home and healthcare settings, robots must be adaptable to human-centric environments. As technology advances, humanoid robots will improve in their ability to mimic human movements and manipulation skills.

Interest in dual-arm manipulation has grown, adding complexity and challenges not present in single-arm systems. Addressing these challenges requires sophisticated integration, planning, reasoning, and control strategies. This complexity drives the development of technologies for coordinating multiple robots' motions, controlling motion and force, and optimizing contact forces in real-time using techniques like quadratic programming.

Cooperative multi-arm systems must control both the motion of an object and its internal stresses. A force controller can regulate force on the object's surface and estimate external wrenches by comparing applied torque with model-based instructions.

Previous research focused on manipulation tasks assuming contact had already been made. 

This study explores the impact of imperfect grasp due to vision and approach control inaccuracies on manipulation outcomes. The goal is to fill a research gap by proposing a method to identify objects and coordinate manipulator movements based on gathered information.

An RGB-D camera captures the workspace's point cloud, enabling the vision node to filter and cluster data to identify potential contact points, even without prior knowledge of the object. Using this data, a Human-Like trajectory is planned for the manipulators to make contact with the object. In the final stage, hybrid control (combining force and Cartesian control) ensures a stable grip, facilitating the subsequent manipulation process.


## Problem Formulation

I have been working with a framework consisting of two seven-jointed manipulators that are firmly grasping a common rigid object. In order to ensure a stable grasp, each manipulator must apply a normal force $ {}^{ee}f_d $ at their respective contact points. 

This algorithm, designed for an $$n$$-DoF manipulator, assumes that $$ {}^{ee}R $$ rotates its frame with respect to the task frame. The other cooperative arms will use the same algorithm, each with their associated frame rotation matrix (see Fig. `franka_box_1`).

Assuming a stable grasp between the end-effector and the object, we define the end-effector's vector $$ {}^{ee}p_{cr} $$ within its frame, linking it to the object's rotation center. This common point among all manipulators is critical, as it provides a uniform reference for all robots. This is necessary in our dual-arm framework to preserve modularity by controlling the object's center of rotation instead of individual manipulator control.

To relate velocities, I define the Jacobian matrix $$ J_{cr} \in \mathbb{R}^{6 \times 6} $$, where $$ x $$ and $$ x_{ee} $$ represent the Cartesian poses of the object's rotation center and the manipulator's end-effector, respectively.

As a result, the manipulator joint velocities can be mapped into Cartesian space at the object's center of rotation through the following equation:

$$
\dot{x} = \underbrace{J_{cr} J_{ee}(q)}_{J(q)} \dot{q}
$$

Finally, the dynamics of the manipulator in Cartesian space can be defined with respect to the object's center of rotation as:

$$
M_C(q) \ddot{x} + C_C(q \dot{q}) \dot{x} + f_g(q) = f_{in} + J_{cr}^{-T} f_{ext}
$$

where $$ f_{ext} \in \mathbb{R}^6 $$ is the external wrench acting on the robot.

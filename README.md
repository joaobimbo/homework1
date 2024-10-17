# Take-home exercise
In this exercise on robot kinematics, I took the following approach:
- I created a class PlanarLink that has a length and an angle. It has a function that gives out the homogenous matrix from base to tip
- I created a class Planar3Robot that contains an array (std::vector) of PlanarLinks, as well as the relevant kinematics and auxiliary functions
- I use Eigen for matrix operations
- I use cmake for building. I build a library for the kinematics and a main that uses that library for testing
- I use gtest for testing

## Forward Kinematics
- I use homogenous matrices. I find this to be the best, most readable way of doing kinematic operations. I **never** use euler angles, and use quaternions only if there is a good reason for it.
- The forward kinematics is just the concatenation of the base-to-tip homogenous transform (```PlanarLink::getTransform()``` ) for each of the links in the robot

## Inverse Kinematics
- I use the simplest version of inverse kinematics that I could think of (Jacobian Transpose and a sort of gradient descent to the goal pose).
- Given:
    - a current configuration of the robot $\textbf{q}_i$, 
    - a desired pose $\textbf{x}_d$, and 
    - the Forward Kinematics $F(\textbf{q}) = \textbf{x}$, 
- we want to find the set of angles $\textbf{q}_d$ that put the end-effector in the desired pose

The Jacobian matrix $J$ maps the joint movements to the end-effector velocity, which I assume is also OK for small displacements. 
$ J \dot{\textbf{q}} = \dot{\textbf{x}} \approx  J \Delta \textbf{q} = \Delta \textbf{x} $

Then, from the error $\textbf{e} = \textbf{x}_d - \textbf{x}_t$, we can move the robot along the error vector through the following:

$\textbf{q}_{t+1} = \textbf{q}_{t} + \Delta \textbf{q} $

Here, inverting the Jacobian matrix would be the solution to obtain $\Delta \textbf{q}$ (actually pseudo-inverting is done). 

Now, for simplicity I use the transpose, which is acceptable because of the principle of virtual work:
- The Work done is irrespective of whether we describe it in joint space or task space:
$ \tau^T \Delta q = F^T \Delta x$
Now, replace with the Jacobian definition: $ \Delta x = J \Delta q$ yields:
$ \tau = J^T F$

This means that, essentially the transpose Jacobian maps the cartesian forces onto the joint torques, and we can use it to move the end-effector along the desired direction to the goal:

$\textbf{q}_{t+1} = \textbf{q}_{t} + \lambda J^T \textbf{e} $

where $\lambda$ is the step-size.

- I used a trick to speed-up the calculation, where I increase $\lambda$ if the error is getting smaller, and decrease it if the step was too large.

## Improvements
- I would probably have done my own matrix class to avoid the dependence on Eigen
- I would do the Moore-Penrose pseudo-inverse to the Jacobian, but the transpose is easier.
- I would have changed the Link and Robot classes to allow 3D robots 

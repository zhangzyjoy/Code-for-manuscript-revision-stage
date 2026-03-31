# Code specifications<br/>
We already upload two folders 'Code For Simulation' and 'Code For Experiment' in the main branch in this repository. The README file introduces how to run the simulation models using MATLAB/SIMULINK, how to run the software-in-loop validation using Python and how to deploy the experiment on multiple crazyflie UAVs. Detailed controller scheme is described with equations and diagrams.<br/>
## Simulation scheme<br/>
Simulation scripts and models are uploaded in 'Code For Simulation' folder and can be tested using MATLAB R2020a software with SIMULINK toolbox installed.<br/>
MATLAB version: 9.8.0.1323502 (R2020a) or higher<br/>
Simulink version: 10.1 (MATLAB R2020a) or higher<br/>
The proposed method includes five main modules: <br/>
1. practical fixed-time distributed state observer (PFxTDSO)<br/>
2. fixed-time disturbance observer in rotational subsystem (FxTDO)<br/>
3. nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>
4. fixed-time disturbance observer in translational subsystem (FxTDO)<br/>
5. practical fixed-time decentralized formation controller (PFxTDFC)<br/>

### Symbol definition list
- $g$ : gravitational acceleration
- $\bar{e}_{3}$ : unit vector $[0,0,1]^{T}$
- $m_{i}$ : mass
- $\Lambda_{i}$ : inertia matrix 
- $p_{0}$ : position vector of leader UAV
- $v_{0}$ : linear velocity vector of leader UAV
- $u_{0}$ : linear acceleration vector of leader UAV
- $p_{i}$ : position vector of follower UAV node $i$
- $v_{i}$ : linear velocity vector of follower UAV node $i$
- $e_{i}^{p}$ : position tracking error of follower UAV node $i$
- $e_{i}^{v}$ : velocity tracking error of follower UAV node $i$
- $u_{i}$ : translational control input vector of follower UAV node $i$
- $\hat{p}_{i}^{d}$ : desired position observation for follower UAV node $i$
- $\hat{v}_{i}^{d}$ : desired velocity observation for follower UAV node $i$
- $e_{i,p}^{d}$ : observation error of desired position for follower UAV node $i$
- $e_{i,v}^{d}$ : observation error of desired velocity for follower UAV node $i$
- $Q_{i} = [\rho_{i}, q_{i}^{T}]^{T} = [\rho_{i}, q_{i}^{1}, q_{i}^{2}, q_{i}^{3}]^{T}$ : quaternion
- $R\left( Q_{i} \right)$ : rotation matrix
- $\varpi_{i}$ : angular velocity
- $Q_{i}^{c}$ : quaternion command
- $R\left( Q_{i}^{c} \right)$ : rotation matrix command
- $\varpi_{i}^{c}$ : angular velocity command
- 

### Control Scheme<br/>
#### practical fixed-time distributed state observer (PFxTDSO)<br/>



#### fixed-time disturbance observer in rotational subsystem (FxTDO)<br/>



#### nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>



#### fixed-time disturbance observer in translational subsystem (FxTDO)<br/>



#### practical fixed-time decentralized formation controller (PFxTDFC)<br/>





## Experimental validation scheme




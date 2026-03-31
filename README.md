# Code Specifications<br/>
We already upload two folders 'Code For Simulation' and 'Code For Experiment' in the main branch in this repository.
The README file includes four chapters: 
1. symbol definition list, indicating all symbols utilized in the control design;
2. controller design, demonstrating detailed controller scheme with equations and diagrams;
3. simulation scheme and implementation procedure, introducing how to run the simulation models using MATLAB/SIMULINK;
4. experimental validation scheme and implementation procedure, introducing how to run the software-in-loop simulation in Python before conducting the experiment, and how to deploy the controller on multiple crazyflie UAVs. <br/>

**Note: A video is developed to illustrate the performance metrics of the proposed / compared method in the real-world experiment by deploying 'Code For Experiment' to drive the crazyflie UAVs.** <br/>
**The video can be accessed at https://youtu.be/3SR10K3WDYw?si=wBZPJSE5iLvDWyJl or https://www.bilibili.com/video/BV1T8XrBbEVf/**

## Symbol Definition List<br/>
- $g$ : gravitational acceleration
- $\overline e _3$ : unit vector $[0,0,1]^{T}$
- $m_{i}$ : mass
- $\Lambda_{i}$ : inertia matrix 
- $p_{0}$, $p_{i}$ : position vector of leader UAV, follower UAV node $i$
- $v_{0}$, $v_{i}$ : linear velocity vector of leader UAV, follower UAV node $i$
- $u_{0}$, $u_{i}$ : translational control input of leader UAV, follower UAV node $i$
- $e_{i}^{p}$, $e_{i}^{v}$ : position, linear velocity tracking error of follower UAV node $i$
- $\hat p_i ^d, \hat v_i ^d$ : desired position, linear velocity observation for follower UAV node $i$
- $e_{i,p}^{d}$, $e_{i,v}^{d}$ : observation error of desired position, linear velocity for follower UAV node $i$
- $\tilde e _{i,p} ^d$, $\tilde e _{i,v} ^d$ : lumped formation observation error of desired position, linear velocity for follower UAV node $i$
- $\chi_i$, $\phi_i$ : virtual linear velocity tracking vector, tracking error
- $d _i ^v$, $\hat d _i ^v$ : translational disturbance, disturbance observation
- $\sigma _i ^v$, $\overline \sigma _i ^v$, $\hat {\overline \sigma} _i ^v$ : virtual linear velocity tracking vector, tracking error, tracking error observation
- $Q_{i} = [\rho_{i}, q_{i}^{T}]^{T} = [\rho_{i}, q_{i}^{1}, q_{i}^{2}, q_{i}^{3}]^T$ : quaternion
- $Q_{i}^{c}$, $Q_{i}^{e}$ : quaternion command, error
- $R(Q_{i})$, $R(Q_{i}^{c})$, $R(Q_{i}^{e})$ : rotation matrix, command, error
- $\varpi_{i}$, $\varpi_{i}^{c}$, $\varpi_{i}^{e}$ : angular velocity, command, error
- $\psi_i^e = [\Psi(R(Q_i^e))]_\vee$ : rotational error in Lie Algebra
- $\overline S_i$ = $[ \overline S_{i,x}, \overline S_{i,y}, \overline S_{i,z} ]^T$ : auxiliary sliding mode surface
- $\Phi(\psi_i^e) = [ \Phi_x(\psi_{i,x} ^e), \Phi_y(\psi_{i,y} ^e), \Phi_z(\psi_{i,z} ^e)]^T$ : intermediate auxiliary rotational error
- $\overline \Phi (\psi_i^e) = [ \overline \Phi _x(\psi ^e _{i,x}), \overline \Phi _y(\psi ^e _{i,y}), \overline \Phi _z(\psi ^e _{i,z})]^T$ : first derivative of intermediate auxiliary rotational error
- $F_i^S$ : rotation compensation term
- $\tau_i$ : applied torque rotational control input
- $d _i ^\varpi$, $\hat d _i ^\varpi$ : rotational disturbance, disturbance observation
- $\sigma _i ^\varpi$, $\overline \sigma _i ^\varpi$, $\hat {\overline \sigma} _i ^\varpi$ : virtual angular velocity tracking vector, tracking error, tracking error observation

## Controller Design <br/>
### practical fixed-time distributed state observer (PFxTDSO)<br/>

Observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
e _{i,v} ^d = \hat v _i ^d - v_0 - \dot \delta _i
\end{aligned}
\quad\quad(1)$$<br/>

$$
\begin{aligned}
e _{i,p} ^d = \hat p _i ^d - p_0 - \delta _i
\end{aligned}
\quad\quad(2)$$<br/>

Lumped formation observation error for follower UAV node $i$<br/>

$$
\begin{aligned}
\tilde e _{i,v} ^d &= b _{i0} e ^d _{i,v} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,v} - e ^d _{j,v} ) \\
&= b _{i0} ( \hat v _i ^d - v_0 - \dot \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat v _i ^d - \dot \delta _i ) - ( \hat v _j ^d - \dot \delta _j ) )
\end{aligned}
\quad\quad(3)$$<br/>

$$
\begin{aligned}
\tilde e _{i,p} ^d &= b _{i0} e ^d _{i,p} + \sum ^N _{j=1} w _{ij} ( e ^d _{i,p} - e ^d _{j,p} ) \\
&= b _{i0} ( \hat p _i ^d - p_0 - \delta _i ) + \sum ^N _{j=1} w _{ij} ( ( \hat p _i ^d - \delta _i ) - ( \hat p _j ^d - \delta _j ) )
\end{aligned}
\quad\quad(4)$$<br/>

Update desired state observation for follower UAV node $i$<br/>

$$
\begin{aligned}
\dot {\hat v} _i ^d = - \ell _1 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _1, \mu _o ^p ) - \ell _2 ^v \vartheta ( \tilde e ^d _{i,v}, \gamma _2, \mu _o ^p ) )
\end{aligned}
\quad\quad(5)$$<br/>

$$
\begin{aligned}
\dot {\hat p} _i ^d = \hat v _i ^d - \ell _1 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _1, \mu _o ^p ) - \ell _2 ^p \vartheta ( \tilde e ^d _{i,p}, \gamma _2, \mu _o ^p ) )
\end{aligned}
\quad\quad(6)$$<br/>

### fixed-time disturbance observer in rotational subsystem (FxTDO)<br/>

Update virtual linear velocity tracking vector<br/>

$$
\begin{aligned}
\dot \sigma _i ^v = -g \overline e _3 + T_i R(Q_i) \overline e _3/ m _i + c _i ^{v,3} \overline \sigma _i ^v
\end{aligned}
\quad\quad(7)$$<br/>

Virtual linear velocity tracking error

$$
\begin{aligned}
\overline \sigma _i ^v = v _i - \sigma _i ^v
\end{aligned}
\quad\quad(8)$$<br/>

High-order nonlinear differentiator

$$
\begin{aligned}
\left{& \tilde \sigma _0 = \overline \sigma _i ^v - \hat \sigma _0 \\
& \dot {\hat \sigma} _2 = - c _1 ^i \mathrm {sgn} ( \tilde \sigma _0 ) - c _1 ^i ( 1 - \hbar ) \mathrm {si} \mathrm g^{1+\varsigma} ( \tilde \sigma _0 ) \\
\end{aligned}
\quad\quad(9)$$<br/>


### nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>



### fixed-time disturbance observer in translational subsystem (FxTDO)<br/>



### practical fixed-time decentralized formation controller (PFxTDFC)<br/>




## Simulation Scheme and Implementation Procedure<br/>
Simulation scripts and models are uploaded in 'Code For Simulation' folder and can be tested using MATLAB R2020a software with SIMULINK toolbox installed.<br/><br/>
MATLAB version: 9.8.0.1323502 (R2020a) or higher<br/>
Simulink version: 10.1 (MATLAB R2020a) or higher<br/><br/>
The proposed method includes five main modules: <br/>
1. practical fixed-time distributed state observer (PFxTDSO)<br/>
2. fixed-time disturbance observer in rotational subsystem (FxTDO)<br/>
3. nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>
4. fixed-time disturbance observer in translational subsystem (FxTDO)<br/>
5. practical fixed-time decentralized formation controller (PFxTDFC)<br/>


## Experimental Validation Scheme and Implementation Procedure<br/>


**If there exist any question about this webpage, please do not hesitate to contact us at any time by zyzhang9921@buaa.edu.cn, or zhaoyuzhang9921@gmail.com**



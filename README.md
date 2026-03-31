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

$$ p_{i} \text{—— position vector of follower UAV node } i $$ <br/>
$$ v_{i} \text{—— velocity vector of follower UAV node } i $$ <br/>
$$ g \text{—— gravitational acceleration} $$ <br/>
$$ \bar{e}_{3} \text{—— unit vector } [0,0,1]^{T} $$ <br/>



### Control Scheme<br/>
#### practical fixed-time distributed state observer (PFxTDSO)<br/>



#### fixed-time disturbance observer in rotational subsystem (FxTDO)<br/>



#### nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>



#### fixed-time disturbance observer in translational subsystem (FxTDO)<br/>



#### practical fixed-time decentralized formation controller (PFxTDFC)<br/>





## Experimental validation scheme




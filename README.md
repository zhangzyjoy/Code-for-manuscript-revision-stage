# Code specifications
We already upload two folders 'Code For Simulation' and 'Code For Experiment' in the main branch in this repository. The README file introduces how to run the simulation models using MATLAB/SIMULINK, how to run the software-in-loop validation using Python and how to deploy the experiment on multiple crazyflie UAVs. Detailed controller scheme is described with equations and diagrams. 

## Simulation scheme

Simulation scripts and models are uploaded in 'Code For Simulation' folder and can be tested using MATLAB R2020a software with SIMULINK toolbox installed.

MATLAB version: 9.8.0.1323502 (R2020a) or higher

Simulink version: 10.1 (MATLAB R2020a) or higher

The proposed method includes five main modules: 
1) practical fixed-time distributed state observer (PFxTDSO)
2) fixed-time disturbance observer in rotational subsystem (FxTDO)
3) nonsingular Lie-algebra-based sliding mode attitude controller (NLASMAC)
4) fixed-time disturbance observer in translational subsystem (FxTDO)
5) practical fixed-time decentralized formation controller (PFxTDFC)

Symbol definition: 
$$p_{i}$$



## Experimental validation scheme




# MPC-ACC

## Description
Compared with other control algorithms, MPC has the ability to predict future events and adjust the control strategy accordingly. Conventional proportional-integral-derivative (PID) controllers lack this predictive performance. Overall, the application of MPC to the control process provides a powerful and flexible way to improve performance and meet complex constraints. Objectives of this system are:

- Avoiding rear-end collision  
- Reduce energy consumption 
- Improves ride comfort 

## Software structure
- MATLAB/Simulink: as vehicle simulator and server
- Python: as a client, the expected acceleration is calculated based on the received signal by employing a 
Model Predictive Control (MPC) algorithm.
![image](https://github.com/ytyiting/MPC-ACC/blob/main/structure2.png?raw=true)

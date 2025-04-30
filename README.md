## 1. Data Collection
Collect data with decoupled dynamics using:
Data_Collection_CT_Decoupled_X.m
Data_Collection_CT_Decoupled_Y.m
Data_Collection_CT_Decoupled_Z.m

Input: Motion Capture System, Agent.
Output: Training_CT_TestDay_i.mat, for i=X,Y,Z. / Testing_CT_TestDay_i.mat, for i=X,Y,Z.


## 2. Data Processing
Compute velocity and acceleration

DataPreProccCT_X.m
DataPreProccCT_Y.m
DataPreProccCT_Z.m

input: Testing_CT_TestDay_i.mat, for i=X,Y,Z.
Output:OutputVector_i, for i=X,Y,Z.

## 3. Filter acceleration to decrease the noise effect using:
DesiredSignal.m 
SignalProcessing2.slx

input: OutputVector_i, for i=X,Y,Z.
Output:OutputVector_i, for i=X,Y,Z.
## 4. Extract coeficients from the result using:
DataPreProccCT_X.m
DataPreProccCT_Y.m
DataPreProccCT_Z.m

input: OutputVector_i, for i=X,Y,Z.
Output:K1, K2

## Citing the paper:

If you use this code, please use the following BibTeX entry:
```bibtex

@article{amiri2024closed,
  title={Closed-loop model identification and MPC-based navigation of quadcopters: A case study of parrot bebop 2},
  author={Amiri, Mohsen and Hosseinzadeh, Mehdi},
  journal={IFAC-PapersOnLine},
  volume={58},
  number={28},
  pages={330--335},
  year={2024},
  publisher={Elsevier}
}





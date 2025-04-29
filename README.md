# 1. Data Collection
Collect data with decoupled dynamics using:
Data_Collection_CT_Decoupled_X.m
Data_Collection_CT_Decoupled_Y.m
Data_Collection_CT_Decoupled_Z.m

Input: Motion Capture System, Agent.
Output: Training_CT_TestDay_i.mat, for i=X,Y,Z. / Testing_CT_TestDay_i.mat, for i=X,Y,Z.


# 2. Data Processing
Compute velocity and acceleration

DataPreProccCT_X.m
DataPreProccCT_Y.m
DataPreProccCT_Z.m

input: Testing_CT_TestDay_i.mat, for i=X,Y,Z.
Output:OutputVector_i, for i=X,Y,Z.

# 3. Filter acceleration to decrease the noise effect using:
DesiredSignal.m 
SignalProcessing2.slx

input: OutputVector_i, for i=X,Y,Z.
Output:OutputVector_i, for i=X,Y,Z.
# 4. Extract coeficients from the result using:
DataPreProccCT_X.m
DataPreProccCT_Y.m
DataPreProccCT_Z.m

input: OutputVector_i, for i=X,Y,Z.
Output:K1, K2

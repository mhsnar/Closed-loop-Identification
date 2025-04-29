close all
clear all
clc

%training data

Training='Testing_CT_TestDay_Y.mat';



Training_Data=load(Training);
SS_training=Training_Data.SS(1:3,:);



CommandInput_training =Training_Data.CI;

% Eliminating 10 first samples because of high Computing times
Time_training=Training_Data.Time(9:end-1)-Training_Data.Time(9);
Training_output_data = SS_training(1:3,9:end);
Training_input_data = [CommandInput_training(1:2,9:end);CommandInput_training(4,9:end)];


for i=1:size(Time_training)-1
Timeee(i)=Time_training(i+1)-Time_training(i);
end

% Interpolating to generate new data in the gaps of samples (32 gap samples)
Training_output_data_new=[];
Training_input_data_new=[];
for i=1:size(Time_training,1)-1
    % We do not have gap sample
    if Time_training(i+1)-Time_training(i)<=0.0084
        Training_output_data_new=[Training_output_data_new Training_output_data(1:3,i)];
        Training_input_data_new=[Training_input_data_new Training_input_data(1:3,i)];

        % We do have gap sample as long as twice of a sample time
    elseif Time_training(i+1)-Time_training(i)>0.0084 && Time_training(i+1)-Time_training(i)<=0.017
         Training_output_data_new=[Training_output_data_new Training_output_data(1:3,i) Training_output_data(1:3,i)+(Training_output_data(1:3,i+1)- Training_output_data(1:3,i))/2];
        Training_input_data_new=[Training_input_data_new Training_input_data(1:3,i) Training_input_data(1:3,i)+(Training_input_data(1:3,i+1)- Training_input_data(1:3,i))/2];

         % We do have gap sample as long as Triple of a sample time
    elseif Time_training(i+1)-Time_training(i)>0.017 && Time_training(i+1)-Time_training(i)<=0.025
      Training_output_data_new=[Training_output_data_new Training_output_data(1:3,i) Training_output_data(1:3,i)+(Training_output_data(1:3,i+1)- Training_output_data(1:3,i))/3 Training_output_data(1:3,i)+2*(Training_output_data(1:3,i+1)- Training_output_data(1:3,i))/3 ];
        Training_input_data_new=[Training_input_data_new Training_input_data(1:3,i) Training_input_data(1:3,i)+(Training_input_data(1:3,i+1)- Training_input_data(1:3,i))/3 Training_input_data(1:3,i)+2*(Training_input_data(1:3,i+1)- Training_input_data(1:3,i))/3 ];
    end
end
Positions=Training_output_data_new;

% Generation velcities from positions data
deltaT=0.008333333333212;
for i=2:size(Positions,2)
Velocities(:,i)=[(Positions(1,i)-Positions(1,i-1))/deltaT ;(Positions(2,i)-Positions(2,i-1))/deltaT ;(Positions(3,i)-Positions(3,i-1))/deltaT];


end
for i=2:size(Positions,2)
Accelerations(:,i)=[(Velocities(1,i)-Velocities(1,i-1))/deltaT ;(Velocities(2,i)-Velocities(2,i-1))/deltaT ;(Velocities(3,i)-Velocities(3,i-1))/deltaT];


end
Accelerations=Accelerations(:,2:end);
% Generation states [x;xdot;y;ydot;z;zdot]
Training_output_data_new=[Positions(1,2:end);Velocities(1,2:end);Accelerations(1,1:end);Positions(2,2:end);Velocities(2,2:end);Accelerations(2,1:end);Positions(3,2:end);Velocities(3,2:end);Accelerations(3,1:end)];
Training_input_data_new=Training_input_data_new(:,2:end);

OutputVector_Y=Training_output_data_new(4:6,:);

u_y=Training_input_data_new(2,:);


%%
u = u_y;
xdot =OutputVector_Y(2,:); % Your xdot data here;
xddot = OutputVector_Y(3,:);% Your xddot data here;

% Initial guess for k1 and k2
initial_guess = [1, 1]; % You can adjust this based on your estimation

% Define the objective function
objective_function = @(k) xddot - [k(1), k(2)] * [xdot; u];
% x=lsqr([xdot', u'],[xddot'])
% Use lsqnonlin to minimize the least squared error
result = lsqnonlin(objective_function, initial_guess);
% [x, resnorm, residual, exitflag, output]
% Extract k1 and k2 from the result
k1 = result(1);
k2 = result(2);


% Display the results
fprintf('k1: %.4f\n', k1);
fprintf('k2: %.4f\n', k2);

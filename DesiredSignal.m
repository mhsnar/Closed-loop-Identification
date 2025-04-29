
% tt=10:0.00833333333333333:100
% Ydesired=.30*sin(.2*pi*tt)+.06*sin(.4*pi*tt)+.01*sin(.7*pi*tt)+.01*sin(pi*tt);
% plot(Ydesired)
% hold on
% plot(Training_output_data_new(4,:))
% DYdesired=.3*.2*pi*cos(.2*pi*tt)+0.06*.4*pi*cos(.4*pi*tt)+0.01*.7*pi*cos(.7*pi*tt)+0.01*pi*cos(pi*tt);
% plot(DYdesired)
% 
% DDYdesired

Vel=[0.008333333333212*[0:1:size(OutputVector_Z(2,:),2)-1]' OutputVector_Z(2,:)'];


Filtered_Velocity=out.FilteredVelocitiy;

Shifted_Filtered_Velocity=[0.008333333333212*[0:1:size(Filtered_Velocity,1)-21]' Filtered_Velocity(21:end)];
% Shifted_Filtered_Velocity=[0.008333333333212*[0:1:22975]' Filtered_Velocity(21:end)];

Acceleration_=[0.008333333333212*[0:1:size(out.Acceleration,1)-1]' out.Acceleration];



Filtered_Acceleration=out.Filtered_Acceleration;
Shifted_Filtered_Acceleration=[0.008333333333212*[0:1:size(Filtered_Velocity,1)-13]' Filtered_Acceleration(13:end)];
% Shifted_Filtered_Acceleration=[0.008333333333212*[0:1:22975]' Filtered_Acceleration(21:end)];


% OutputVector_X=[Positions(1,2:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2)));Velocities(1,2:min(size(Shifted_Filtered_Acceleration,1),size(Velocities,2)));Shifted_Filtered_Acceleration(1:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2))-1,2)'];
% OutputVector_Y=[Positions(2,2:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2)));Velocities(2,2:min(size(Shifted_Filtered_Acceleration,1),size(Velocities,2)));Shifted_Filtered_Acceleration(1:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2))-1,2)'];
OutputVector_Z=[Positions(3,2:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2)));Velocities(3,2:min(size(Shifted_Filtered_Acceleration,1),size(Velocities,2)));Shifted_Filtered_Acceleration(1:min(size(Shifted_Filtered_Acceleration,1),size(Positions,2))-1,2)'];

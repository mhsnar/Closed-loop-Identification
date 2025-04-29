clear all
close all
clc

global SIASclient    FLAG
global goalDesired Error CommandInput SS SS2 commandArray_Idf Cont GoalPt CI
global deltaT inc drone_mode ContGoal xyDesired vertDesired yawDesired DESIREDPOINT
global p state state2 xyDesired vertDesired rollDesired pitchDesired yawDesired DESIRED
global SIASclient Error CommandInput ContGoal Time
global goalDesired SS SS2 commandArray_Idf CI
global deltaT drone_mode Cont GoalPt DESIRED1
global p state state2 inc goalDesired deltaT  xyDesired vertDesired rollDesired pitchDesired yawDesired DESIREDPOINT
 
SIASclient = natnet;
SIASclient.connect; 
pause(2)

fprintf('\n\nConnecting to Drone...\n') 
p = parrot(); 
fprintf('Connected to %s\n', p.ID) 
fprintf('Battery Level is %d%%\n', p.BatteryLevel)
takeoff(p); 
pause(1)
% 



% deltaT=8.333333333439441e-03;
% deltaT=0.009;
goalDesired = [0;0;1;0;0;deg2rad(0)];

% testing
% goalDesired = [ 0   0.3 -0.3  -0.2 0.7  0   0   0    0    0  0.4    -.4 0 0 0; 
%               -0.5 -1 0.5   -0.8  1     0   0   -.7 0.5  0.5  .5     -.5 0 0 0; 
%                0.8   1  0.9   1.2  0.8  1.1 0.8 .8  0.8   1.5  1.5   .8 2 .8 1.5 ; 
%                 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;  
%                 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%               deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0)];

%trainig

% goalDesired = [ 0.5   0.3  -0.2  -0.3      0       0   -0.1 0  0  0    0   0   0  0   0.5   0.2   0.7  0   0.1 -0.3 0.4 0.7       .5   .5    .5  .5      .5      .5   -0.5   .5   -.5  0.7   0    0 0 0 0.5  .5  .5 .5 .5 .5 .5 .5 .5; 
%                 0.5  -0.5   0.5  -0.5     -0.5     0    1   0  0  0    0    0  0  0   0.7  -0.3    0  0.1 -0.2 -1 0.6 0         0     0    .7  -0.7     -.7    -.7   .7   -.7   0   0    0 -0.5 1  0  -.5 -.5 -.5 -.5 .5 .5 .5 .5 .5; 
%                  1    0.8   1.2   2.1       1     0.8  0.8  .8 2.5  .8  1.9  .9  2.5   1 .8   0.8   1.2 0.9   2.2 0.8 0.8 0.8         1.5  0.8   .8   .8      1.5    1     0.8  1.5   0.8 .8 0.8  .8  .8 0.9 0.8 2.5 .8 1.7 .9 2 1.5 2.1 1.7; 
%                 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0  0  0    0   0   0  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%                 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0  0  0    0   0   0  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0  0  0    0   0   0  0  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];


 
 Error=zeros(4,1);
 CommandInput=zeros(4,1);
inc=1;
Cont=1;
ContGoal=0;
GoalPt=1;

% deltaT=0.3;
% SamplingTime = timer('ExecutionMode','fixedRate','Period',deltaT,'TimerFcn',@(~,~)myfile);
% start(SamplingTime);
% 
% % 
% % 
% % 
% % 
% function myfile
while(1>0)


Time(inc,1)=double(SIASclient.getFrame.fTimestamp);

     Position=double([SIASclient.getFrame.RigidBodies(1).x;SIASclient.getFrame.RigidBodies(1).y;SIASclient.getFrame.RigidBodies(1).z]);
    q=quaternion( SIASclient.getFrame.RigidBodies(1).qw, SIASclient.getFrame.RigidBodies(1).qx, SIASclient.getFrame.RigidBodies(1).qy, SIASclient.getFrame.RigidBodies(1).qz );
    eulerAngles=quat2eul(q,'xyz')*180/pi;
    Angle=[eulerAngles(1);eulerAngles(2);eulerAngles(3)];
    state=[Position;Angle];
    [errorArray]=ControlCommand;
    if FLAG(inc)==1
        SS(:,inc)=state;
        CI(:,inc)=commandArray_Idf;

       DESIRED(:,inc)=[goalDesired(1:2,GoalPt);goalDesired(3,GoalPt); goalDesired(6,GoalPt)];
        Error(:,inc)=errorArray;
       inc=inc+1;
    end
% end

end


function [errorArray]=ControlCommand
 
global p state  inc FLAG SS CI Error commandArray_Idf  GoalPt ContGoal goalDesired  Time DESIRED





%% Define desired tolerances and gains

 kYaw =-1; 
kPitch = -0.65;
kRoll = -0.65; 
kVert = -1.7;
% Derivative Gain Valuesla
kD_Pitch = -.4; 
kD_Roll = -.4; 
kD_Yaw =-0.8;
kD_Vert = -0.05;





 tt=inc*8.333333333439441e-03;

% Xdesired=.30*sin(.2*pi*tt)+.06*sin(.4*pi*tt)+.01*sin(.6*pi*tt)+.01*sin(.8*pi*tt)+.01*sin(1*pi*tt);
%Test
Xdesired=.30*sin(.1*pi*tt)+.06*sin(.3*pi*tt)+.01*sin(.5*pi*tt)+.01*sin(.7*pi*tt)+.01*sin(.9*pi*tt);

xyDesired(1:2,1) = [Xdesired;goalDesired(2,GoalPt)];



vertDesired = goalDesired(3,GoalPt);  
yawDesired= goalDesired(6,GoalPt); 


 if inc == 1 

 old_xy_Error = 0; 
 old_yaw_Error = 0; 
 old_vert_Error = 0; 

    % takeoff(p); 

 else
     old_yaw_Error=Error(3,inc-1);
     old_xy_Error=Error(1:2,inc-1);
     old_vert_Error=Error(4,inc-1);

 end

 positionActual = state(1:2);
 yawActual = deg2rad(state(6)); 
 vertActual = state(3);

 
 % to rotate X,Y from world frame to robot frame
 Tw2r = [cos(yawActual), sin(yawActual); -sin(yawActual), cos(yawActual)]; 


 % 
 
 
 
 % Compute the errors
 % Yaw Error
 if inc==1
     dt=0.008301;
 else
     dt=Time(inc,1)-Time(inc-1,1);
 end

if dt < eps 
   dt = 0.008301; % Average calculated time step
end

% dt

yawe1=(yawDesired - yawActual);
 yawError = wrapToPi(yawe1);
 yawD_Error = (yawError-old_yaw_Error)/dt;
 
 % compute the yaw commands
 yawCmd = kYaw*yawError+kD_Yaw*yawD_Error;
 
 if abs(yawCmd) > 3.4 
 yawCmd = sign(yawCmd)*3.4; 
 end 


 
 % Position Error
 xyError = xyDesired(1:2,1) - positionActual; 
 xyD_Error = (xyError - old_xy_Error)/dt; 
  % compute the pitch commands
 roll_pitch_cmd = (Tw2r)*xyError; %error in robot frame
 

 pitchCmd = kPitch*roll_pitch_cmd(1) + kD_Pitch*xyD_Error(1);

 if abs(pitchCmd) > 0.06 % limitations of Parrot Drone
 pitchCmd = sign(pitchCmd)*0.06; 
 end

  % compute the roll commands

rollCmd = kRoll*roll_pitch_cmd(2)+ kD_Roll*(xyD_Error(2));

 if abs(rollCmd) > 0.06 % limitations of Parrot Drone
 rollCmd = sign(rollCmd)*0.06; 
 end 


 % Altitude Error
 vertError = vertDesired - vertActual; 
 vertD_Error = (vertError - old_vert_Error)/dt; 
 
 vertCmd = kVert*vertError+kD_Vert*vertD_Error; 
 
 if vertCmd < -.2 
 vertCmd = -.2;
 elseif vertCmd > .2
      vertCmd = .2;
 end
 TotalError = norm([xyError; yawError; vertError]); 
 
 % store data for post analysis
 errorArray = [xyError; yawError; vertError]; 
 
 % TotalError = norm(totalError); 

  if inc>1 
      if Time(inc,1)-Time(inc-1,1)>eps
          commandArray_Idf= [pitchCmd; rollCmd; yawCmd; vertCmd];
          FLAG(inc)=1;
  else
      commandArray_Idf= [CI(1,inc-1); CI(2,inc-1); CI(3,inc-1); CI(4,inc-1)];
      FLAG(inc)=0;
      end
  else
      commandArray_Idf= [pitchCmd; rollCmd; yawCmd; vertCmd];
       FLAG(inc)=1;
  end



move(p, 0.1, 'RotationSpeed', commandArray_Idf(3),'VerticalSpeed', commandArray_Idf(4),'roll', commandArray_Idf(2), 'pitch', commandArray_Idf(1));







if TotalError<=0.1
ContGoal=ContGoal+1;
end

if ContGoal==Inf
    GoalPt=GoalPt+1;
    ContGoal=0;
end

 if GoalPt>size(goalDesired,2)
    land(p)
    save('Training_CT.mat','SS','Time','CI','DESIRED')

 end

end



clear all
close all
clc

global SIASclient
global goalDesired Error CommandInput    Cont GoalPt kYaw kD_Yaw Time CI SS DESIRED1 theta theta u xbar ubar V ErrorV NormU ErrorU
global deltaT inc  ContGoal yawDesired Constraint TG p A B C EstimationError GoalPt ContGoal goalDesired
global Prediction_Horizion Ad Bd xhat Qn u_des r Qv Abar Bbar QuBar QxBar M1Bar M2Bar psi u_up u_down Kpsi1 Kpsi2 Kpsi3 M1 M2 N theta_p u_p Uconstraint K
yalmip
SIASclient = natnet;
SIASclient.connect;
pause(2)

fprintf('\n\nConnecting to Drone...\n')
p = parrot();
fprintf('Connected to %s\n', p.ID)
fprintf('Battery Level is %d%%\n', p.BatteryLevel)
takeoff(p);
pause(1)



inc = 0;

deltaT=0.2;

% goalDesired = [0;-0.5;1.4;0;0;deg2rad(0)];
% goalDesired=[0;0;2;0;0;0];


% Square
% goalDesired = [ 1   -1  -1   -1      1      ;
%                 2    2   -1  -2     -2     ;
%               2.5    2.5   2.5   2.5       2.5     ;
%                 0 0 0 0 0 ;
%                 0 0 0 0 0 ;
%                0 0 0 0 0 ];

% line interupted
% goalDesired = [-2  2   2  -2   0      1      ;
%                -2  -2    2   2  0     -2     ;
%               2.5 2.5    2.5   2.5   2.5       2.5     ;
%                0 0 0 0 0 0 ;
%               0  0 0 0 0 0 ;
%               0 0 0 0 0 0 ];


%snake
% path=[                         0                         0     2.000000000000000e+00
%      1.597419993350742e-01    -1.570383560109744e-01     2.000000000000000e+00
%      3.194839986701484e-01    -2.981836792692507e-01     2.000000000000000e+00
%      4.792259980052226e-01    -4.091513879540845e-01     2.000000000000000e+00
%      6.389679973402969e-01    -4.787110191810027e-01     2.000000000000000e+00
%      7.987099966753711e-01    -4.998228055617263e-01     2.000000000000000e+00
%      9.584519960104453e-01    -4.703501333355167e-01     2.000000000000000e+00
%      1.118193995345520e+00    -3.932757779013213e-01     2.000000000000000e+00
%      1.277935994680594e+00    -2.764000326805968e-01     2.000000000000000e+00
%      1.437677994015668e+00    -1.315512821137606e-01     2.000000000000000e+00
%      1.597419993350742e+00     2.661108742108925e-02     2.000000000000000e+00
%      1.757161992685816e+00     1.820802876264110e-01     2.000000000000000e+00
%      1.916903992020891e+00     3.191220918224099e-01     2.000000000000000e+00
%      2.076645991355965e+00     4.238672139448355e-01     2.000000000000000e+00
%      2.236387990691039e+00     4.857149466323549e-01     2.000000000000000e+00
%      2.396129990026113e+00     4.984060035153751e-01     2.000000000000000e+00
%      2.555871989361187e+00     4.606559889352066e-01     2.000000000000000e+00
%      2.715613988696262e+00     3.762853849280692e-01     2.000000000000000e+00
%      2.875355988031336e+00     2.538329001694200e-01     2.000000000000000e+00
%      3.035097987366410e+00     1.056913118148122e-01     2.000000000000000e+00
%      3.194839986701485e+00    -5.314674282368256e-02     2.000000000000000e+00
%      3.354581986036559e+00    -2.066060928841889e-01     2.000000000000000e+00
%      3.514323985371633e+00    -3.391559181348078e-01     2.000000000000000e+00
%      3.674065984706707e+00    -4.373815422659805e-01     2.000000000000000e+00
%      3.833807984041781e+00    -4.913420622962604e-01     2.000000000000000e+00
%      3.993549983376856e+00    -4.955764155020035e-01     2.000000000000000e+00
%      4.153291982711930e+00    -4.496560650856097e-01     2.000000000000000e+00
%      4.313033982047004e+00    -3.582283701491580e-01     2.000000000000000e+00
%      4.472775981382078e+00    -2.305462507246632e-01     2.000000000000000e+00
%      4.632517980717153e+00    -7.953174800953595e-02     2.000000000000000e+00
%      4.792259980052227e+00     7.953174800953559e-02     2.000000000000000e+00
%      4.952001979387301e+00     2.305462507246629e-01     2.000000000000000e+00
%      5.111743978722375e+00     3.582283701491571e-01     2.000000000000000e+00
%      5.271485978057450e+00     4.496560650856099e-01     2.000000000000000e+00
%      5.431227977392524e+00     4.955764155020036e-01     2.000000000000000e+00
%      5.590969976727598e+00     4.913420622962605e-01     2.000000000000000e+00
%      5.750711976062672e+00     4.373815422659806e-01     2.000000000000000e+00
%      5.910453975397747e+00     3.391559181348078e-01     2.000000000000000e+00
%      6.070195974732821e+00     2.066060928841892e-01     2.000000000000000e+00
%      6.229937974067894e+00     5.314674282368336e-02     2.000000000000000e+00
%      6.389679973402969e+00    -1.056913118148119e-01     2.000000000000000e+00
%      6.549421972738044e+00    -2.538329001694201e-01     2.000000000000000e+00
%      6.709163972073117e+00    -3.762853849280690e-01     2.000000000000000e+00
%      6.868905971408191e+00    -4.606559889352061e-01     2.000000000000000e+00
%      7.028647970743266e+00    -4.984060035153750e-01     2.000000000000000e+00
%      7.188389970078340e+00    -4.857149466323550e-01     2.000000000000000e+00
%      7.348131969413414e+00    -4.238672139448358e-01     2.000000000000000e+00
%      7.507873968748489e+00    -3.191220918224096e-01     2.000000000000000e+00
%      7.667615968083562e+00    -1.820802876264119e-01     2.000000000000000e+00
%      7.827357967418637e+00    -2.661108742108984e-02     2.000000000000000e+00
%      7.987099966753712e+00     1.315512821137614e-01     2.000000000000000e+00
%      8.146841966088784e+00     2.764000326805954e-01     2.000000000000000e+00
%      8.306583965423860e+00     3.932757779013211e-01     2.000000000000000e+00
%      8.466325964758934e+00     4.703501333355163e-01     2.000000000000000e+00
%      8.626067964094007e+00     4.998228055617263e-01     2.000000000000000e+00
%      8.785809963429083e+00     4.787110191810028e-01     2.000000000000000e+00
%      8.945551962764156e+00     4.091513879540848e-01     2.000000000000000e+00
%      9.105293962099230e+00     2.981836792692516e-01     2.000000000000000e+00
%      9.265035961434306e+00     1.570383560109743e-01     2.000000000000000e+00
%      9.424777960769379e+00     3.673940397442059e-16     2.000000000000000e+00]
% goalDesired=[path zeros(size(path,1),size(path,2))]';

Error=zeros(4,1);
CommandInput=zeros(4,1);
inc=1;
Cont=1;
ContGoal=0;
GoalPt=1;

%
% Positional Gain Values
kYaw =-1;

% Derivative Gain Values
kD_Yaw =-0.5;

%
Prediction_Horizion=10;


% Ts=.3;saturation=.05;all bounded
Kx1=-0.052701624162968;
Kx2=-5.477869892924388;
Ky1=-0.018687914020970;
Ky2=-7.060834512263481;
Kz1=-1.787280741456883;
Kz2=-1.738212699869965;

Ac=[0   1   0   0   0   0
   0  Kx1  0   0   0   0
   0   0   0   1   0   0
   0   0   0   Ky1 0   0
   0   0   0   0   0   1
   0   0   0   0   0  Kz1];
Bc=[0   0   0
  Kx2  0   0
   0   0   0
   0  Ky2  0
   0   0   0
   0   0  Kz2];


Cc=[1	0	0	0	0	0
0	1	0	0	0	0
0	0	1	0	0	0
0	0	0	1	0	0
0	0	0	0	1	0
0	0	0	0	0	1];
Dc=[0	0	0
0	0	0
0	0	0
0	0	0
0	0	0
0	0	0];


Gd=c2d(ss(Ac,Bc,Cc,Dc),deltaT);
A=Gd.A
B=Gd.B
C=Gd.C
D=Gd.D

u_des=[0;0;0];
%
Ad=[A B ; zeros(size(u_des,1),size(A,2)) zeros(size(u_des,1),size(B,2))];
Bd=[zeros(size(A,1),size(u_des,1));eye(size(u_des,1),size(B,2))];

Cd=[C D];
%


Qx=5*eye(size(A,1));



Qu=[35 0 0 ;0 20 0;0 0 1];

Qx=[Qx zeros(size(Qx,1),size(Qu,1));zeros(size(Qu,1),size(Qx,1)) Qu];






[Qn,K,~,~] = idare(Ad,Bd,Qx,Qu,[],[]);
K=-K;
psi=dlyap((Ad+Bd*K)',eye(size(Ad,1)));

Qv=500*eye(size(Cd,1));

Kpsi1=K(1,:)*(psi\K(1,:)');
Kpsi2=K(2,:)*(psi\K(2,:)');
Kpsi3=K(3,:)*(psi\K(3,:)');





% Ts=.3;saturation=.05;Allbounded
M1=[1	0	0
0	0	0
0	1	0
0	0	0
0	0	1
0	0	0];
M2=[0	0	0
0	0	0
0	0	0];
M1=[M1;M2];

N=[1	0	0
0	0	0
0	1	0
0	0	0
0	0	1
0	0	0];
%% Compact MPC

% Calculating x_pr:
Abar=Ad;
for i=2:Prediction_Horizion
    Abar=[Abar;Ad^(i)];
end



[o, m] = size(Bd);
Bbar = zeros(o * Prediction_Horizion, m * Prediction_Horizion);

for i = 1:Prediction_Horizion
    for j = 1:i
        A_power = Ad^(i-j);

        % Element-wise multiplication using matrix multiplication
        block = A_power * Bd;

        % Calculate the indices for insertion
        row_indices = (i-1)*o + 1 : i*o;
        col_indices = (j-1)*m + 1 : j*m;

        % Insert the block into the appropriate position in Bbar
        Bbar(row_indices, col_indices) = block;
    end
end

% Create Constraints limits using blkdiag
Uconstraint=[.05 .05 .6]';
u_up=[];
for i=1:Prediction_Horizion
    u_up=[u_up;Uconstraint];
end
u_down=-u_up;


% Create QuBar using blkdiag
QuBar = blkdiag(kron(eye(Prediction_Horizion), Qu));
QxBar = blkdiag(kron(eye(Prediction_Horizion), Qx));




M1Bar=M1;
M2Bar=M2;
for i=1:Prediction_Horizion-1
    M1Bar=[M1Bar;M1];
    M2Bar=[M2Bar;M2];
end
%


%
SamplingTime = timer('ExecutionMode','fixedRate','Period',deltaT,'TimerFcn',@(~,~)myfile);
start(SamplingTime);


%



function myfile
% while(1)
tic
global SIASclient Error TimeGlob TG  xhat_p Time  SS  commandArray_Idf CI  DESIRED1 state  inc    xyDesired vertDesired   yawDesired r GoalPt goalDesired deltaT


Position=double([SIASclient.getFrame.RigidBodies(1).x;SIASclient.getFrame.RigidBodies(1).y;SIASclient.getFrame.RigidBodies(1).z]);
q=quaternion( SIASclient.getFrame.RigidBodies(1).qw, SIASclient.getFrame.RigidBodies(1).qx, SIASclient.getFrame.RigidBodies(1).qy, SIASclient.getFrame.RigidBodies(1).qz );
eulerAngles=quat2eul(q,'xyz')*180/pi;
Angle=[eulerAngles(1);eulerAngles(2);eulerAngles(3)];

state=[Position;Angle];
% state=[0;0;0;0;0;0];
Time0=toc;

%% TRACKING 

tt=inc*deltaT;


%  Lemniscate of Bernoulli
Xdesired=.5*sin(.4*pi*tt);
Ydesired=1*sin(.2*pi*tt);
Zdesired=1.5;

%  Lemniscate of Bernoulli INterupt

% Xdesired=.5*sin(.4*pi*tt);
% Ydesired=-1+1*sin(.2*pi*tt);
% Zdesired=1.5;
% if Xdesired>=.1 && Ydesired<=.1
%     Xdesired=.5*sin(.4*pi*tt);
% Ydesired=-1+1*sin(.2*pi*tt);
% Zdesired=1.5;
% end




goalDesired = [ Xdesired ; Ydesired ; Zdesired;0;0;0];
yawDesired= goalDesired(6,GoalPt);


%%



r=[goalDesired(1,GoalPt);0;goalDesired(2,GoalPt);0;goalDesired(3,GoalPt);0];
[errorArray]=ControlCommand;
tic

% ControlCommand;
SS(:,inc)=state;
CI(:,inc)=commandArray_Idf;
DESIRED1(:,inc)=[xyDesired;vertDesired;yawDesired];




Error(:,inc)=errorArray;



inc=inc+1;
Time=toc;
TG(:,inc-1)=[Time0;TimeGlob;Time;sum([Time0;TimeGlob;Time])];

end


function [errorArray]=ControlCommand
tic
global state  inc deltaT Error EstimationError CI  xhat A B C commandArray_Idf  SS yawDesired kYaw kD_Yaw theta theta u xbar ubar V ErrorV NormU ErrorU theta_p u_p p
global Prediction_Horizion Ad Bd TimeGlob  Qn u_des r Qv Abar Bbar QuBar QxBar M1Bar M2Bar psi u_up u_down Kpsi1 Kpsi2 Kpsi3 M1 M2 N Constraint Uconstraint K GoalPt ContGoal goalDesired
Time1=toc;

tic
% Define desired tolerances and gains
if inc == 1
    old_yaw_Error = 0;
else
    old_yaw_Error=Error(3,inc-1);
end

yawActual = deg2rad(state(6));



% Compute the errors
% Yaw Error
yawError = wrapToPi(yawDesired - yawActual);
yawD_Error = (yawError-old_yaw_Error)/deltaT;

% compute the yaw commands
yawCmd = kYaw*yawError+kD_Yaw*yawD_Error;

if abs(yawCmd) > 1.49
    yawCmd = sign(yawCmd)*1.49;
end
Time2=toc;
%%
if inc>1
move(p, 1.1*deltaT, 'RotationSpeed', yawCmd,'VerticalSpeed', CI(4,inc-1),'roll', CI(2,inc-1), 'pitch', CI(1,inc-1));
else
move(p, 1.1*deltaT, 'RotationSpeed', yawCmd); 
end

%% CMPC

yalmip('clear')

theta = sdpvar((size(M1,2)),1,'full');
% theta=r;
u = sdpvar((size(Bd,2))*Prediction_Horizion,1,'full');
xbar=M1*theta;
ubar=M2*theta;
V=N*theta;
errV=V-r;
ErrorV=(errV'*Qv*errV);

norU=ubar-u_des;
NormU= (norU'*norU);

errU = u - M2Bar*theta;
ErrorU = errU' * QuBar * errU;

Constraint=[u<=u_up;u>=u_down];

tic

if inc==1
    x0=double([state(1);0;state(2);0;state(3);0;0;0;0]);
elseif inc>1

        x0=double([state(1);(state(1)-SS(1,inc-1))/deltaT;state(2);(state(2)-SS(2,inc-1))/deltaT;state(3);(state(3)-SS(3,inc-1))/deltaT; CI(1:2,inc-1);CI(4,inc-1)]);
end

x_pr =Abar*x0+Bbar*u;
Time3=toc;

tic
errX = x_pr - M1Bar*theta;
ErrorX = errX' * QxBar * errX;


errTX = x_pr(size(Ad,1) * Prediction_Horizion - (size(Ad,1)-1):size(Ad,1) * Prediction_Horizion) - xbar;
ErrorTX = errTX' * Qn * errTX;






sigma = ErrorV+ErrorU + ErrorX + ErrorTX + NormU;
% sigma =  ErrorV+ErrorX  ;
Time4=toc;
tic
% Terminal Conditions
err=x_pr(size(Ad,1)*Prediction_Horizion-(size(Ad,1)-1):size(Ad,1)*Prediction_Horizion)-xbar;
Lyapan=(err)'*psi*(err);

omegastar=20;
x_pr_Terminal(:,1)=(Ad+Bd*K)*x_pr(size(Ad,1)*Prediction_Horizion-(size(Ad,1)-1):size(Ad,1)*Prediction_Horizion)+(Bd*M2-Bd*K*M1)*theta;
for omega=2:omegastar
    x_pr_Terminal(:,omega)=(Ad+Bd*K)*x_pr_Terminal(:,omega-1)+(Bd*M2-Bd*K*M1)*theta;
end


TConstraints = [M2*theta<=0.98*Uconstraint;M2*theta>=-0.98*Uconstraint];
for omega=1:omegastar
    TConstraints=[TConstraints;M2*theta+K*(x_pr_Terminal(:,omega)-M1*theta)<=Uconstraint;M2*theta+K*(x_pr_Terminal(:,omega)-M1*theta)>=-Uconstraint];
end

Constraints=[Constraint;TConstraints];
Time5=toc;


opt=sdpsettings('cachesolvers',1);

tic
sol=optimize(Constraints,sigma,opt)
Time6=toc;
tic
commandArray=double(u(1:3,1));
theta_p(:,inc)=double(theta);
u_p=double(u(4:end));

%
commandArray_Idf= [commandArray(1); commandArray(2); yawCmd; commandArray(3)];



errorArray = [0;0; yawError; 0];
Time7=toc;
TimeGlob=[Time1;Time2;Time3;Time4;Time5;Time6;Time7];




 positionActual = state(1:2);
 yawActual = deg2rad(state(6)); 
 vertActual = state(3);


 xyDesired(1:2,1) = goalDesired(1:2,GoalPt);
vertDesired = goalDesired(3,GoalPt);  
yawDesired= goalDesired(6,GoalPt); 
yawe1=(yawDesired - yawActual);
 yawError = wrapToPi(yawe1);
 xyError = xyDesired(1:2,1) - positionActual; 
vertError = vertDesired - vertActual; 

 TotalError = norm([xyError; yawError; vertError])
 


if TotalError<=0.1
ContGoal=ContGoal+1;
end

if ContGoal==Inf
    GoalPt=GoalPt+1;
    ContGoal=0;
end

 if GoalPt>size(goalDesired,2)
    land(p)
    save('SSMPCExp5.mat')

 end

end


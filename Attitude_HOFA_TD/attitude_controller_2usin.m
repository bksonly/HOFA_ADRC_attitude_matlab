function [sys,x0,str,ts,simStateCompliance] = attitude_controller_2usin(t,x,u,flag,dt,J,K,b1,b2,b3)
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(dt);
    case 1
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u,dt,b1,b2,b3);
  case 3,
    sys=mdlOutputs(t,x,u,dt,J,K);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(dt)

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 24;%Rd wd dwd z1 z2 z3
sizes.NumOutputs     = 24;% M wd e de
sizes.NumInputs      = 21;%R w Rd
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
 x0  =[1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
str = [];
ts  = [dt 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u,dt,b1,b2,b3)
global Kp Kd;
Rd=reshape(u(13:21),[3,3]);
pre_Rd=reshape(x(1:9),[3,3]);
pre_wd=reshape(x(10:12),[3,1]);

wd=vee(Rd'*(Rd-pre_Rd)/dt);
dwd=(wd-pre_wd)/dt;

w=reshape(u(10:12),[3,1]);
R=reshape(u(1:9),[3,3]);
Rr=Rd'*R;
ew=w-R'*Rd*wd;
hat_ew=hat(ew);
y=vee(Rr-Rr');
dy=vee(Rr*hat_ew+hat_ew*Rr');

z1=reshape(x(16:18),[3,1]);
z2=reshape(x(19:21),[3,1]);
z3=reshape(x(22:24),[3,1]);
M_star=-Kp.*fal(0.4,0.01,y)-Kd.*fal(0.9,0.01,dy);

z3=z3+dt*(-b3*fal(0.25,0.01,z1-y));
z2=z2+dt*(z3-b2*fal(0.5,0.01,z1-y)+M_star);
z1=z1+dt*(z2-b1*(z1-y));

sys = [reshape(Rd,[1,9]),wd',dwd',z1',z2',z3'];

function sys=mdlOutputs(t,x,u,dt,J,K)
global Kp Kd;
R=reshape(u(1:9),[3,3]);
w=reshape(u(10:12),[3,1]);


%%%%%
% R = [1 0 0;0 1 0; 0 0 1];
% w = [0;0;0];
%%%%%%%

Rd=reshape(x(1:9),[3,3]);
wd=reshape(x(10:12),[3,1]);
dwd=reshape(x(13:15),[3,1]);


Rr=Rd'*R;
eR=0.5*vee(Rr-Rr');
ew=w-R'*Rd*wd;
hat_ew=hat(ew);

tmp1=Rr*hat_ew*hat(w)-hat(wd)*Rr*hat_ew-hat(dwd)*Rr;
tmp=tmp1-tmp1';

RB=[Rr(2,2)+Rr(3,3)       -Rr(2,1)          -Rr(3,1);
     -Rr(1,2)         Rr(1,1)+Rr(3,3)       -Rr(3,2);
     -Rr(1,3)           -Rr(2,3)          Rr(1,1)+Rr(2,2)];

e=vee(Rr-Rr');
de=vee(Rr*hat_ew+hat_ew*Rr');

z1=reshape(x(16:18),[3,1]);
z2=reshape(x(19:21),[3,1]);
z3=reshape(x(22:24),[3,1]);
% z3=0*z3;
% e=z1;
% de=z2;

if rank(RB)==3
    B=RB*inv(J);
    invB=J*inv(RB);
    M_star=-Kp.*e-Kd.*de;
%     M_star=-Kp.*fal(0.4,0.01,e)-Kd.*fal(0.9,0.01,de);%-z3;
%     M_star=-[10;10;3].*e-[100;100;30].*de;
    M=invB*(-vee(tmp)+M_star)+cross(w,J*w);
    ul=invB*(M_star);

else
%     M=-8.81*eR-2.54*ew+cross(w,J*w)-J*(hat(w)*Rr'*iwd-Rr'*idwd);
M=[0;0;0];
end


% th=0.5;
% M(1)=deadzone(-th,M(1),th);
% M(2)=deadzone(-th,M(2),th);
% th=0.01;
% M(3)=deadzone(-th,M(3),th);
global R1;
Rr=R1'*R;
eout=vee(Rr-Rr');
sys =[M;wd;eout;de;z1;z2;z3;ul];

% end mdlOutputs


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];



clear all
close all

%define parameters
param.mass=[24;11;11];
param.r1 = 0.1;
param.r2 = 0.1;
param.r3 = 0.1;
param.I1 = 1/2*param.mass(1)*(param.r1)^2;
param.I2 = 1/2*param.mass(2)*(param.r2)^2;
param.I3 = 1/2*param.mass(3)*(param.r3)^2;
param.ks=[1;1;1]*10000;
param.bs=[1;1;1]*100;
param.gg=[0;-9.8];
param.yg = 0;
param.kg = 100000;
param.bg = 100;
param.kk=[1;1;1]*10000;
param.bk=[1;1;1]*100;
param.vdx = 1.0;
param.desiredHeight = 1;
param.desiredHeightGain = 1;
param.actuatorGain = [1;1;1]*100;%torque1,force2,force3

%initial conditions
po12 = [0;1.0];
po13 = po12;
tho12 = 30*pi/180;
tho13 = -30*pi/180;
tho2 = tho12;
tho3 = tho13;
lo = po12(2)*2/sqrt(3);

phi12 = -pi/2;
phi13 = -pi/2;
po2 = po12 +lo*[cos(tho2+phi12); sin(tho2+phi12)];
po3 = po13 +lo*[cos(tho3+phi13); sin(tho3+phi13)];

param.ls2o=lo;
param.ls3o=lo;
param.po12=po12;
param.po13=po13;
param.po2=po2;
param.po3=po3;
param.phi12 = phi12;
param.phi13 = phi13;

param.tipphi12 = pi/2;
param.tipphi13 = pi/2;

%simulator conditions
param.simulator.timeSpan = [0:1e-04:1];
param.simulator.h = 1e-04;
%param.simulator.odeOptions = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

%start simulation
h   = param.simulator.h;
timeSpan   = param.simulator.timeSpan;
initialCondition = [po12; po13; po2; po3; tho12; tho13; tho2; tho3;...
                    0;0;  0;0;  0;0; 0;0; 0;    0; 0; 0];

[y,result]=computeRungeKutta(timeSpan,initialCondition,param,h);

%odeOptions = param.simulator.odeOptions;
%timeSpan = param.simulator.timeSpan;
%initialCondition = param.simulator.initialCondition;
%[time, state] = ode45('dynamice',timeSpan,initialCondition,odeOptions,param);





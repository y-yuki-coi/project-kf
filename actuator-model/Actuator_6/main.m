clear all
close all

%define parameters
param.mass=[48;11;11];
param.r1 = 0.1;
param.I1 = 1/2*param.mass(1)*(param.r1)^2;
param.ks=[1;1;1]*10000;
param.bs=[1;1;1]*100;
param.kk=[1;1;1]*10000;
param.bk=[1;1;1]*100;
param.kc=10000;
param.bc=100;
param.mass=[48;11;11];
param.gg=[0;-9.8];
param.yg = 0;
param.kg = 100000;
param.bg = 100;
param.ks=[1;1;1]*10000;
param.bs=[1;1;1]*100;
param.kk=[1;1;1]*10000;
param.bk=[1;1;1]*100;
param.kc=10000;
param.bc=100;

%initial conditions
po12 = [0;1.0];
po13 = po12;
tho2 =  120*pi/180;
tho3 =   60*pi/180;
lo = po12(2)*2/sqrt(3);
po2 = po12 -lo*[cos(tho2); sin(tho2)];
po3 = po13 -lo*[cos(tho3); sin(tho3)];

param.ls2o=lo;
param.ls3o=lo;
param.po12=po12;
param.po13=po13;
param.po2=po2;
param.po3=po3;

param.phi12 = tho2 + pi;
param.phi13 = tho3 + pi;

%simulator conditions
param.simulator.timeSpan = [0:1e-03:0.3];
param.simulator.h = 1e-03;
%param.simulator.odeOptions = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

%start simulation
h   = param.simulator.h;
timeSpan   = param.simulator.timeSpan;
initialCondition = [po12; po13; po2; po3; 0;    0; ...
                    0;0;  0;0;  0;0; 0;0; 0;    0 ];

[y,result]=computeRungeKutta(timeSpan,initialCondition,param,h);

%odeOptions = param.simulator.odeOptions;
%timeSpan = param.simulator.timeSpan;
%initialCondition = param.simulator.initialCondition;
%[time, state] = ode45('dynamice',timeSpan,initialCondition,odeOptions,param);





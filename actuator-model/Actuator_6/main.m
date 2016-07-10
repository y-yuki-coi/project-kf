clear all
close all

%define parameters
param.mass=[48;11;11];
param.gg=[0;-9.8];
param.ks=[1;1;1]*10000;
param.bs=[1;1;1]*100;
param.kk=[1;1;1]*10000;
param.bk=[1;1;1]*100;
param.kc=10000;
param.bc=100;
param.mass=[48;11;11];
param.gg=[0;-9.8];
param.ks=[1;1;1]*10000;
param.bs=[1;1;1]*100;
param.kk=[1;1;1]*10000;
param.bk=[1;1;1]*100;
param.kc=10000;
param.bc=100;

%initial conditions
po12 = [0;1.0];
po13 = po12;
tho2 =  60*pi/180;
tho3 = 120*pi/180;
lo = po12(2)*2/sqrt(3);
po2 = po12 -lo*[cos(tho2); sin(tho2)];
po3 = po13 -lo*[cos(tho3); sin(tho3)];

param.ls2o=lo;
param.ls3o=lo;
param.po12=po12;
param.po13=po13;
param.po2=po2;
param.po3=po3;

%simulator conditions
param.simulator.timeSpan = [0:0.1:2];
param.simulator.odeOptions = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

%start simulation
computeRungeKutta(param);

%odeOptions = param.simulator.odeOptions;
%timeSpan = param.simulator.timeSpan;
%initialCondition = param.simulator.initialCondition;

%[time, state] = ode45('dynamice',timeSpan,initialCondition,odeOptions,param);




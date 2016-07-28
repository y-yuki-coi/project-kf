% Actuator_Mobility_4_160617.m
% ���q���f���œ񑫈ړ����Č�
% Fujii & Yoshihara, coded by Fujii
clear all; close all; %#ok<CLALL> % 

% Parameters
% Gi = [1000 10000 10000 1000 6000 6000] ; % Feedback gain zeros(6,1);%  
param.gg = [0;-9.8];
param.k_leg21 = 5000; % air
param.k_leg31 = 5000;
param.k_leg22 = 10000; % ground
param.k_leg32 = 10000;
param.b_leg2 = 100;
param.b_leg3 = 100;
param.klim1  = 100000; % �傫�߂ɕύX
param.klim2  = 100000;
param.klim3  = 100000;
param.blim1  = 100;
param.blim2  = 100;
param.blim3  = 100;
param.m1 = 48;
param.m2 = 11;
param.m3 = 11;
param.G1 = 5000; %feedback gain
param.G2 = 5000;
param.G3 = 5000;
param.vdx = 2; % desired speed m/s
param.ckf = 0.5; % ratio of km control to kf control�i�ǉ��j
param.desiredHeight = 1; % now testing
param.desiredHeightGain = 1; % now testing
param.yg = 0.00; % terrain (horizontal)
param.kg = 100000; % �傫�߂ɕύX
param.bg = 100;
param.eps1 = 1e-10; 
param.eps2 = 1e-4; 
param.eps3 = 1e-4; % ��̏��Z������邽�߂̔����� 
param.y01 = 0.92;
param.l0 = param.y01*2/sqrt(3);

param.llim1=param.l0*1.1;
param.llim2=param.l0*1.1;
param.llim3=param.l0*1.1;
param.simulator.h = 1e-03;
param.simulator.maxItr = 5000;

%rename params
h = param.simulator.h;
maxItr = param.simulator.maxItr;

% Initial condition �r�����ւ���A�������t�ɂ��Ă���������ɂȂ邩�m�F
% �����p����������₷���p���ɕς���H�i�Ȃ��������o�����̂ŕۗ��j--------------------------
y_contact = zeros(2,16) ; % xi & initial and contact
x1 = 0 ; y1 = param.y01 ; % initial position for gait
% thi = [0 120/180*pi 60/180*pi] ;
thi = [0 60/180*pi 120/180*pi] ;
th2 = thi(2); th3 = thi(3); 
x2 = x1 - param.l0*cos(th3); % center of segment, CHECK
y2 = y1 - param.l0*sin(th3); 
x3 = x1 - param.l0*cos(th2); 
y3 = y1 - param.l0*sin(th2); 
th1 = 0; %temporalily set to zero, acos((x2-x3)/sqrt((x2-x3)^2+(y2-y3)^2));
xi = [x1 y1 x2 y2 x3 y3] ; % vector


%check foreleg is right and shown in red 
if 0; figure(1); plot(xi(1,[1 3]),xi(1,[2 4]),'ro-',xi(1,[1 5]),xi(1,[2 6]),'bo-',xi(1,[3 5]),xi(1,[4 6]),'ko-'); hold on;
end

%start simulation
if 1
    Y = [xi zeros(1,6)] ; % dt_xi(9)
    tic;
    iter =maxItr; 
    Y = [Y;zeros(iter,12)]; 
    for t = 1:iter
        [Y(t+1,:) result(t,:)] = fun_Actuator(t,Y(t,:),param,h); 
        if isnan(Y(t+1,7)); t
            break 
        end
        if mod(t,1000)==0; 
            t 
        end
    end
    time = h:h:(iter+1)*h ;
    toc;
    xi = Y(:,1:6) ; dt_xi = Y(:,7:12) ;
end

if 1
    plot_result ;
end
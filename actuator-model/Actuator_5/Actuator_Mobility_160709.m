% Actuator_Mobility_3_160628.m
% Fujii & Yoshihara, coded by Fujii
clear all; close all;

% Parameters
param.gg = [0;-9.8];
param.k_leg2 = 10000;
param.k_leg3 = 10000;
param.b_leg2 = 100;
param.b_leg3 = 100;
param.m1 = 48;
param.m2 = 11;
param.m3 = 11;
param.G1 = 1000; %feedback gain
param.G2 = 1000;
param.G3 = 1000;
param.vdx = 1.0; % desired speed m/s
param.desiredHeight = 1.2; % now testing
param.desiredHeightGain = 1; % now testing
param.yg = 0.00; % terrain (horizontal)
param.kg = 100000;
param.bg = 100;
param.eps1 = 1e-10; 
param.eps2 = 1e-4; 
param.eps3 = 1e-4; % 零の除算を避けるための微小量 
param.y01 = 0.92;
param.l0 = param.y01*2/sqrt(3);
param.klim1=100000;
param.klim2=100000;
param.klim3=100000;
param.blim1=100;
param.blim2=100;
param.blim3=100;
param.llim1=param.l0*1.1;
param.llim2=param.l0*1.1;
param.llim3=param.l0*1.1;
param.simulator.h = 1e-04;
param.simulator.maxItr = 20000;
param.zeroLevel = 1e-03;

%rename params
h = param.simulator.h;
maxItr = param.simulator.maxItr;

% Initial condition
y_contact = zeros(2,16) ; % xi & initial and contact
x1 = 0 ; y1 = param.y01 ; % initial position for gait
                          %thi = [0 120/180*pi 60/180*pi] ;
thi = [0 60/180*pi 145/180*pi] ;
thi = [0 145/180*pi 60/180*pi] ;

thi = [0 60/180*pi 120/180*pi] ;
%thi = [0 120/180*pi 60/180*pi] ;

th2 = thi(2); th3 = thi(3); 
x2 = x1 - param.l0*cos(th2); % center of segment, CHECK
y2 = y1 - param.l0*sin(th2); 
x3 = x1 - param.l0*cos(th3); 
y3 = y1 - param.l0*sin(th3); 
th1 = 0; %temporalily set to zero, acos((x2-x3)/sqrt((x2-x3)^2+(y2-y3)^2));
xi = [x1 y1 x2 y2 x3 y3] ; % vector

%check foreleg is right and shown in red 
if 0; figure(1); plot(xi(1,[1 3]),xi(1,[2 4]),'ro-',xi(1,[1 5]),xi(1,[2 6]),'bo-',xi(1,[3 5]),xi(1,[4 6]),'ko-'); hold on;
    % if 0; figure(1); plot(xi(1,[1 4]),xi(1,[2 5]),'ro-',xi(1,[1 7]),xi(1,[2 8]),'bo-',xi(1,[4 7]),xi(1,[5 8]),'ko-'); hold on;
end % center of segment

%start simulation
if 1 
    Y = [xi zeros(1,6)] ; % dt_xi(9)
                          % run simulation
    tic;
    Y = [Y;zeros(maxItr,12)]; %initialize Y matrix
    for t = 1:maxItr
        [Y(t+1,:) result(t,:)] = fun_Actuator(t,Y(t,:),param,h); 
        if mod(t,1000)==0; 
            t 
        end
    end
    time = h:h:(maxItr+1)*h ;  
    toc;
    
    xi = Y(:,1:6) ; dt_xi = Y(:,7:12) ; 
    % jti = calc_joint_pos(xi,li) ;
    delta = time(2)-time(1) ;
    % l_vec = [sqrt((xi(:,3)-xi(:,5)).^2+(xi(:,4)-xi(:,6)).^2) sqrt((xi(:,1)-xi(:,3)).^2+(xi(:,2)-xi(:,4)).^2) ...
    %    sqrt((xi(:,1)-xi(:,5)).^2+(xi(:,2)-xi(:,6)).^2)] ; 
    % l_vec(:,4:6) = diff3p(l_vec(:,1:3),1/h);
    % figure(2); plot(l_vec(:,6),'r'); hold on; plot(dt2_xi(:,6),'b');
end



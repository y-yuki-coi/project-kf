% Particle_Mobility_160612.m
% 粒子モデルで二足移動を再現
% Fujii & Yoshihara（このコードはFujii作成）
clear; close all;
% Parameters
global g h mi li  ki yg y_contact u0 Gi bi % Ii 
g = 9.8 ; h = 10^(-2) ;
mi = [48 11] ; % [kg] 1: HAT 2: leg
li = [0.7 1 1]; % [m] 1: dammy 2: leg 3:between leg 
% Ii = [0 mi(2)*li(2)^2/12 ]; % inertia 1: dammy 2: leg 
bi = [100 100 1000 100] ; % b1 b2 bk bg
ki = [10000 10000 10000] ; % k1 k2 kg
yg = 0.00; % terrain (horizontal)
u0 = 20 ; % constrant neural input 4-8(slow walk--fast run)
Gi = 25 ; % Feedback gain
% Initial condition
l1 = li(2); l1 = li(2);
y_contact = zeros(2,16) ; % xi & initial and contact
x1 = 0 ; y1 = 1 ; % initial position for gait
thi = [0 40/180*pi 100/180*pi] ;
th2 = thi(2); th3 = thi(3); 
x2 = x1 + l1*1*cos(th2) ; % center of segment
y2 = y1 - l1*1*sin(th2) ; 
x3 = x1 + l1*1*cos(th3) ; 
y3 = y1 - l1*1*sin(th3) ; 
th1 = acos((x2-x3)/sqrt((x2-x3)^2+(y2-y3)^2)) ; 
% xi = [x1 y1 th1 x2 y2 th2 x3 y3 th3] ; % vector
xi = [x1 y1 x2 y2 x3 y3] ; % vector
% jti = calc_joint_pos(xi,li) ;
% check: 前足が右足で最初で赤色
if 0; figure(1); plot(xi(1,[1 3]),xi(1,[2 4]),'ro-',xi(1,[1 5]),xi(1,[2 6]),'bo-',xi(1,[3 5]),xi(1,[4 6]),'ko-'); hold on;
% if 0; figure(1); plot(xi(1,[1 4]),xi(1,[2 5]),'ro-',xi(1,[1 7]),xi(1,[2 8]),'bo-',xi(1,[4 7]),xi(1,[5 8]),'ko-'); hold on;
end % center of segment

% シミュレーション
if 1 
Y = [xi zeros(1,6)] ; % dt_xi(9)
% run simulation
tic;
% options = odeset('RelTol',1e-4,'AbsTol',repmat(1e-4,1,52));
% [time,Y] = ode45(@odefun_Taga1991,[0 2],Y0,options) ;
iter = 200 ; 
Y = [Y;zeros(iter,12)]; var1=zeros(iter,6); var2 = zeros(iter,6);
for t = 1:iter
    [Y(t+1,:) var1(t,:) var2(t,:)] = fun_Actuator(Y(t,:)); 
%     [Y(t+1,:) Fgi(t,:) dt2_xi(t,:) Tri(t,:)] = fun_Neural_Taga1991(Y(t,:)); 
    Y_contact(t,:) = [y_contact(1,1:7) y_contact(2,1:7)] ;
    if mod(t,1000)==0; t 
    end
end
time = h:h:(iter+1)*h ;  
toc;

xi = Y(:,1:6) ; dt_xi = Y(:,7:12) ; 
% jti = calc_joint_pos(xi,li) ;
delta = time(2)-time(1) ;
% l_vec = [sqrt((xi(:,3)-xi(:,5)).^2+(xi(:,4)-xi(:,6)).^2) sqrt((xi(:,1)-xi(:,3)).^2+(xi(:,2)-xi(:,4)).^2) ...
%    sqrt((xi(:,1)-xi(:,5)).^2+(xi(:,2)-xi(:,6)).^2)] ; 
% l_vec(:,4:6) = diff3p(l_vec(:,1:3),1/h);
% figure(2); plot(l_vec(:,6),'r'); hold on; plot(dt2_xi(:,6),'b');
end

% check
if 0
figure(2)
set(gcf,'Color',[1 1 1]) ;
subplot 211; plot(time(1:end-1),Fgi(:,1),'m-',time(1:end-1),Fgi(:,2),'r--',time(1:end-1),Fgi(:,3),'c-.',time(1:end-1),Fgi(:,4),'b:')
% subplot 212; plot(time(1:end),xi(:,5),'m-',time(1:end),xi(:,8),'r--',time(1:end),xi(:,11),'c-.',time(1:end),xi(:,14),'b:')
% plot(time(1:end-1),dt2_xi(:,5),'m-');
% plot(time,jti(1:end,8),'r-',time,jti(1:end,10),'b--',time,zeros(length(time),1),'k:'); 
end

% Movie
if 1
figure(1)
set(gcf,'Color',[1 1 1]) ;
skip = 1 ;
nn = 1 ; % mov index
         %v = VideoWriter('Particle_Mobility_160612.mp4','MPEG-4');
v = VideoWriter('Particle_Mobility_160612.avi');
open(v)
for frame = 1:skip:t%length(time) 
    plot(xi(frame,[1 3]),xi(frame,[2 4]),'ro-',xi(frame,[1 5]),xi(frame,[2 6]),'bo-',xi(frame,[3 5]),xi(frame,[4 6]),'ko-'); hold on;
%     plot(xi(frame,[1 4]),xi(frame,[2 5]),'ro-',xi(frame,[1 7]),xi(frame,[2 8]),'bo-',xi(frame,[4 7]),xi(frame,[5 8]),'ko-'); hold on;
    line([-10 10],[0 0],'color','k');
    axis equal
    xlim([-1 3]) ;
    ylim([-0.5 2]) ; 
    hold off ;
    title(sprintf('time = %0.2f (s)',time(frame)),'fontsize',12);
    set(gca,'fontsize',12) ;
    mov(nn)= getframe(gcf); % mov index
    nn = nn+1;
    drawnow
end

writeVideo(v,mov)
close(v)
end
% Actuator_160419.m
% 直動アクチュエータモデルで二足移動を再現
% Fujii & Yoshihara（このコードはFujii作成）
clear; close all;
% Parameters
global g h mi li Ii bi ki yg y_contact
g = 9.8 ; h = 10^(-2) ;
mi = [48 11] ; % [kg] 1: HAT 2: leg
li = [0 1.1]; % [m] 1: dammy 2: leg 
Ii = [0 mi(2)*li(2)^2/12 ]; % inertia 1: dammy 2: leg 
bi = [10 10 1000 1000] ; % b1 b2 bk bg
ki = [10000 11000] ; % kk kg
yg = 0.00; % terrain (horizontal)
% Initial condition
l2 = li(2); 
y_contact = zeros(2,16) ; % xi & initial and contact
x1 = 0 ; y1 = 2;%1.09 ; % initial position for gait 
thi = [0 0.45*pi 0.57*pi] ;
% x1 = 0 ; y1 = 1.11 ; % initial position for static posture  
% thi = [0 pi/2 pi/2 pi/2 pi/2] ;
th2 = thi(2); th3 = thi(3); 
x2 = x1 + (l2/2)*cos(th2) ; % center of segment
y2 = y1 - (l2/2)*sin(th2) ; 
x3 = x1 + (l2/2)*cos(th3) ; 
y3 = y1 - (l2/2)*sin(th3) ; 
xi = [x1 y1 x2 y2 th2 x3 y3 th3] ; % vector
jti = calc_joint_pos(xi,li) ;
% check: 前足が右足で最初で赤色
if 1; figure(1); plot(jti(1,[1 5]),jti(1,[2 6]),'bo-',jti(1,[1 3]),jti(1,[2 4]),'ro-'); hold on;
    plot(x2,y2,'rs','markerfacecolor','r');plot(x3,y3,'bs','markerfacecolor','b'); axis equal; 
end % center of segment

% シミュレーション
if 1 
    % アクチュエータ入力部分を作成（評価関数を作りたいが、まずは受動Dynamicsから--------------------------------------
Y = [xi zeros(1,8)] ; % dt_xi(8)
% run simulation
tic;
% options = odeset('RelTol',1e-4,'AbsTol',repmat(1e-4,1,52));
% [time,Y] = ode45(@odefun_Taga1991,[0 2],Y0,options) ;
iter = 500 ; 
Y = [Y;zeros(iter,16)]; Fgi=zeros(iter,4); dt2_xi = zeros(iter,8);
for t = 1:iter
    [Y(t+1,:) Fgi(t,:) dt2_xi(t,:)] = fun_Actuator(Y(t,:)); 
%     [Y(t+1,:) Fgi(t,:) dt2_xi(t,:) Tri(t,:)] = fun_Neural_Taga1991(Y(t,:)); 
    Y_contact(t,:) = [y_contact(1,1:9) y_contact(2,1:9)] ;
    if mod(t,1000)==0; t 
    end
end
time = h:h:(iter+1)*h ;  
toc;

xi = Y(:,1:8) ; dt_xi = Y(:,9:16) ; 
jti = calc_joint_pos(xi,li) ;
delta = time(2)-time(1) ;
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
skip = 5 ;
nn = 1 ; % mov index

for frame = 1:skip:t%length(time) 
    plot(jti(frame,[1 5]),jti(frame,[2 6]),'bo-',jti(frame,[1 3]),jti(frame,[2 4]),'ro-'); hold on;% 身体
    plot(xi(frame,3),xi(frame,4),'rs','markerfacecolor','r');
    plot(xi(frame,6),xi(frame,7),'bs','markerfacecolor','b'); 
    line([-10 10],[0 0],'color','k');
    xlim([-1 10]) ;
    ylim([-0.5 2]) ; 
%     axis equal
    hold off ;
    title(sprintf('time = %0.2f (s)',time(frame)),'fontsize',12);
    set(gca,'fontsize',12) ;
    mov(nn)= getframe(gcf); % mov index
    nn = nn+1;
    drawnow
end

movie2avi(mov,'Taga1991_151209','compression','None');
end
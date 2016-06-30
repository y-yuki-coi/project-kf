% Actuator_Mobility_3_160628.m
% Fujii & Yoshihara, coded by Fujii
clear; close all;

% Parameters
global g h mi li  ki yg y_contact u0 Gi bi kkf % Ii 
g = 9.8 ; h = 1e-03;
mi = [48 11] ; % [kg] 1: HAT 2: leg
l0 = 1; % [m] 1: dummy 2: leg 3:between leg 
% Ii = [0 mi(2)*li(2)^2/12 ]; % inertia 1: dammy 2: leg 
bi = [1000 1000 1000 1000 1000] ; % b1 b2 bk bg
ki = [100000 50000 100000 1000 10000] ; % k1 k2 kg
yg = 0.00; % terrain (horizontal)
u0 = 3; % constrant neural input 4-8(slow walk--fast run)
Gi = [1000 10000 10000 1000 6000 6000] ; % Feedback gain zeros(6,1);%  
kkf = 1 ; 

param.gg = [0;-9.8];
param.k_leg2 = 100000;
param.k_leg3 = 100000;
param.b_leg2 = 1000;
param.b_leg3 = 1000;
param.l0 = l0;
param.m1 = 48;
param.m2 = 11;
param.m3 = 11;


% Initial condition
y_contact = zeros(2,16) ; % xi & initial and contact
x1 = 0 ; y1 = 0.92 ; % initial position for gait
thi = [0 60/180*pi 120/180*pi] ;
th2 = thi(2); th3 = thi(3); 
x2 = x1 - l0*cos(th3); % center of segment, CHECK
y2 = y1 - l0*sin(th3); 
x3 = x1 - l0*cos(th2); 
y3 = y1 - l0*sin(th2); 
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
iter =4000 ; 
Y = [Y;zeros(iter,12)]; var1=zeros(iter,24); var2 = zeros(iter,22);
for t = 1:iter
    [Y(t+1,:) var1(t,:) var2(t,:)] = fun_Actuator(Y(t,:),param,h); 
    %[Y(t+1,:) Fgi(t,:) dt2_xi(t,:) Tri(t,:)] = fun_Neural_Taga1991(Y(t,:)); 
    %Y_contact(t,:) = [y_contact(1,1:7) y_contact(2,1:7)] ;
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

figure(3)
plot(dt_xi(:,1),'b'); hold on
% plot(var1(:,1),'r');
% Movie
if 1
figure(1)
set(gcf,'Color',[1 1 1]) ;
skip = 100 ;
nn = 1 ; % mov index
% v = VideoWriter('Actuator_Mobility_3_160617.mp4','MPEG-4');
v = VideoWriter('Actuator_Mobility_3_160617.avi');
open(v)
clr_1 = {'k','r','b'} ; clr_2 = {'r','b';'k','b';'k','r'} ;
dev_1 = [0 -0.1; 0.1 0;-0.1 0];
for frame = 1:skip:t%length(time) 
    xit = xi(frame,:); 
    plot(xit([1 3]),xit([2 4]),'ko:',xit([1 5]),xit([2 6]),'ko:',xit([3 5]),xit([4 6]),'ko:'); hold on;
    tmp = var1(frame,7:12) ; vdl(1,1:2) = tmp(1:2); vdl(2,1:2) = tmp(3:4); vdl(3,1:2) = tmp(5:6);
    tmp = var1(frame,13:24) ; vdc(1,1:2,1) = tmp(1:2); vdc(1,1:2,2) = tmp(3:4); vdc(2,1:2,1) = tmp(5:6); vdc(2,1:2,2) = tmp(7:8);
                              vdc(3,1:2,1) = tmp(9:10); vdc(3,1:2,2) = tmp(11:12);
    tmp = var2(frame,13:16) ; Fgi(1,1:2) = tmp(1:2); Fgi(2,1:2) = tmp(3:4); 
    xi2(1,1:2) = (3*xit(3:4)+xit(5:6))/4; xi2(1,3:4) = (xit(3:4)+3*xit(5:6))/4;
    xi2(2,1:2) = (3*xit(3:4)+xit(1:2))/4; xi2(2,3:4) = (xit(3:4)+3*xit(1:2))/4; 
    xi2(3,1:2) = (3*xit(1:2)+xit(5:6))/4; xi2(3,3:4) = (xit(1:2)+3*xit(5:6))/4;
    tmp = var2(frame,10:15)*0.0005 ; Fai(1,1:2) = tmp(1:2) ; Fai(2,1:2) = tmp(3:4) ; Fai(3,1:2) = tmp(5:6) ;
    for i = 1:3
        quiver(xit(2*i-1),xit(2*i),vdl(i,1),vdl(i,2),0.1,'color',clr_1{i})
        quiver(xit(2*i-1),xit(2*i),vdc(i,1,1),vdc(i,2,1),0.1,'color',clr_2{i,1})
        quiver(xit(2*i-1),xit(2*i),vdc(i,1,2),vdc(i,2,2),0.1,'color',clr_2{i,2})
        quiver(xi2(i,1)+dev_1(i,1),xi2(i,2)+dev_1(i,2),Fai(i,1),Fai(i,2),0.1,'color','k')
        quiver(xi2(i,3)+dev_1(i,1),xi2(i,4)+dev_1(i,2),-Fai(i,1),-Fai(i,2),0.1,'color','k')
    end
    line([-10 10],[0 0],'color','k');
    axis equal
    xlim([-1 3]) ;
    ylim([-0.5 2]) ; 
    hold off ;
    title(sprintf('time = %0.2f (s) k1 = %0.2f k2 = %0.2f k3 = %0.2f kf1 = %0.2f kf2 = %0.2f kf3 = %0.2f',time(frame),...
        var2(frame,4),var2(frame,5),var2(frame,6),var2(frame,7),var2(frame,8),var2(frame,9)),'fontsize',8);
%     title(sprintf('time = %0.2f (s) k1 = %0.2f k2 = %0.2f k3 = %0.2f th1 = %d th2 = %d th3 = %d',time(frame),...
%         var2(frame,4),var2(frame,5),var2(frame,6),floor(var2(frame,20)),floor(var2(frame,21)),floor(var2(frame,22))),'fontsize',8);
    set(gca,'fontsize',10) ;
    mov(nn)= getframe(gcf); % mov index
    nn = nn+1;
    drawnow
end

writeVideo(v,mov)
close(v)
% saveas(gcf,'example.pdf');
end

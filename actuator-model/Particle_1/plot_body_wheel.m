function plot_body_wheel(Body,t)

% for debug
% t = 1 ;
global li  hw hwb hwt
l1 = li(1); l2 = li(2); l3 = li(3); l4 = li(4); l5 = li(5); % [m] 1: pelvis width 2: leg length, 3: trunk height 4: shoulder length: 5: head height
[cy(1:2,:),cy(3:4,:),cy(5:6,:)] = cylinder(1,40) ; Circle = cy(1:2:3,:).' ;
[cy100(1:2,:),cy100(3:4,:),cy100(5:6,:)] = cylinder(1,100) ; Circle100 = cy100(1:2:3,:).' ;
x1 = Body.wheel.xi(t,1) ;
x2 = Body.wheel.xi(t,2) ;
xr = Body.wheel.xr(t,1) ;
xl = Body.wheel.xl(t,1) ;

% Wheel
wheel_r = Circle*hw + repmat([xr hw],41,1) ;
wheel_l = Circle*hw + repmat([xl hw],41,1) ;
body_wheel = [xl hw*2; xr hw*2; xr hw*2+hwb; xl hw*2+hwb; xl hw*2] ;
weight = Circle*hwt + repmat([x1 hw*2+hwb+hwt],41,1) ;
th1 = mod(Body.wheel.thi(t,1),2*pi) ;
th2 = mod(Body.wheel.thi(t,2),2*pi) ;
th3 = Body.wheel.thi(t,3) ;
th4 = Body.wheel.thi(t,4) ;
arrow1 = [xr -0.1; xr+ th3*0.1 -0.1];
arrow2 = [xl -0.1; xl+ th4*0.1 -0.1];
wheel_r1 = [xr+hw*sin(th1) hw+hw*cos(th1); xr-hw*sin(th1) hw-hw*cos(th1)] ;
wheel_r2 = [xr+hw*cos(th1) hw-hw*sin(th1); xr-hw*cos(th1) hw+hw*sin(th1)] ;
wheel_l1 = [xl+hw*sin(th2) hw+hw*cos(th2); xl-hw*sin(th2) hw-hw*cos(th2)] ;
wheel_l2 = [xl+hw*cos(th2) hw-hw*sin(th2); xl-hw*cos(th2) hw+hw*sin(th2)] ;

% subplot(4,1,1) % plot
plot(wheel_r(:,1),wheel_r(:,2),'k-') ; hold on ;
plot(wheel_l(:,1),wheel_l(:,2),'k-') ; 
plot(weight(:,1),weight(:,2),'k-','markerfacecolor',[0.2 0.2 0.2]) ; 
plot(body_wheel(:,1),body_wheel(:,2),'k-') ; 
plot(arrow1(:,1),arrow1(:,2),'k-') ;
plot(arrow2(:,1),arrow2(:,2),'k-') ;
plot(wheel_r1(:,1),wheel_r1(:,2),'k-') ;
plot(wheel_l1(:,1),wheel_l1(:,2),'k-') ;
if th3 > 0 ; plot(arrow1(2,1),arrow1(2,2),'k>','markerfacecolor','k','markersize',4) ; 
    plot(wheel_r2(1,1),wheel_r2(1,2),'ko','markerfacecolor','k','markersize',4) ; 
else  plot(arrow1(2,1),arrow1(2,2),'k<','markerfacecolor','k','markersize',4) ; 
    plot(wheel_r2(2,1),wheel_r2(2,2),'ko','markerfacecolor','k','markersize',4) ; 
end
if th4 > 0 ; plot(arrow2(2,1),arrow2(2,2),'k>','markerfacecolor','k','markersize',4) ; 
    plot(wheel_l2(1,1),wheel_l2(1,2),'ko','markerfacecolor','k','markersize',4) ; 
else  plot(arrow2(2,1),arrow2(2,2),'k<','markerfacecolor','k','markersize',4) ; 
    plot(wheel_l2(2,1),wheel_l2(2,2),'ko','markerfacecolor','k','markersize',4) ; 
end
line([-10 10],[0 0],'color','k','linestyle',':');
% axis equal
% set(gca,'xlim',[-2 2],'ylim',[-0.2 0.5]);

% Human
Head = Body.human.Head;
RShoulder = Body.human.RShoulder; RAnkle = Body.human.RAnkle; RToe = Body.human.RToe;
RKnee = Body.human.RKnee; RHip = Body.human.RHip;
LShoulder = Body.human.LShoulder; LAnkle = Body.human.LAnkle; LToe = Body.human.LToe;
LKnee = Body.human.LKnee; LHip = Body.human.LHip;
Leg = [LToe;LAnkle;LKnee;LHip;RHip;RKnee;RAnkle;RToe];
Trunk = [LHip;LShoulder;RShoulder;RHip];
Head_Circle = Circle*l5 + repmat(Head,41,1) ;
% subplot(4,1,2:4) % plot
plot(Leg(:,1),Leg(:,2)-2,'k-') ; hold on ;
plot(Trunk(:,1),Trunk(:,2)-2,'k-') ; 
plot(Head_Circle(:,1),Head_Circle(:,2)-2,'k-') ; 
line([-10 10],[-2 -2],'color','k','linestyle',':');
axis equal
set(gca,'xlim',[-3 3],'ylim',[-2.2 0.5]); % 1.8
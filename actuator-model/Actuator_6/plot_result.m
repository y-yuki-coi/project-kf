close all;

%copy result to matrix
[~,timeRange]=size(result)
for t = 1:1:timeRange
    p12(t,:)=result(t).p12;
    p13(t,:)=result(t).p13;
    p2(t,:)=result(t).p2;
    p3(t,:)=result(t).p3;
    th12(t,:)=result(t).th12;
    th13(t,:)=result(t).th13;
    dth12(t,:)=result(t).dth12;
    dth13(t,:)=result(t).dth13;
    
    p2n(t,:)=result(t).p2n;
    p3n(t,:)=result(t).p3n;
    q12(t,:)=result(t).q12;
    q13(t,:)=result(t).q13;
    ls2(t,:)=result(t).ls2;
    ls3(t,:)=result(t).ls3;
    dls2(t,:)=result(t).dls2;    
    dls3(t,:)=result(t).dls3;
    th12(t,:)=result(t).th12;
    th13(t,:)=result(t).th13;
    Fk12(t,:)=result(t).Fk12;
    Fk13(t,:)=result(t).Fk13;    
    Tk12(t,:)=result(t).Tk12;
    Tk13(t,:)=result(t).Tk13;

    Fc11(t,:)=result(t).Fc11;
end

time=((1:1:timeRange)*result(1).param.simulator.h)';

%now start plotting
figure
hold on
plot(p12(:,1),p12(:,2),'r-');
plot(p13(:,1),p13(:,2),'g-');
plot(p2(:,1),p2(:,2),'b-');
plot(p3(:,1),p3(:,2),'k-');
axis equal

%figure
%hold on
%plot(xx02(:,1),xx02(:,2),'g-');
%plot(xx03(:,1),xx03(:,2),'b-');
%axis equal
%title('xx0');

figure
hold on
plot(time,ls2,'g-');
plot(time,ls3,'b-');
plot(time,dls2,'g-.');
plot(time,dls3,'b-.');
title('ls2, ls3');

figure
subplot(3,1,1);
hold on
plot(time,th12*180/pi,'g-');
plot(time,th13*180/pi,'b-');
title('th12, th13');

subplot(3,1,2);
hold on
plot(time,dth12*180/pi,'g-');
plot(time,dth13*180/pi,'b-');
title('dth12, dth13');

subplot(3,1,3);
hold on
plot(time,Tk12,'g-');
plot(time,Tk13,'b-');
title('Tk12, Tk13');

figure
hold on
plot(time,Fk12,'g-');
plot(time,Fk13,'b-');
title('Fk12, Fk13');



if 0
figure
hold on
plot(time,vd(:,1),'r-.');
plot(time,vd(:,2),'k-.');
plot(time,dxx1(:,1),'r-');
plot(time,dxx1(:,2),'k-');
title('vd, v1');

figure
hold on
plot(time,Flim1(:,1).^2 + Flim1(:,1).^2,'r-');
plot(time,Flim2(:,1).^2 + Flim2(:,1).^2,'g-');
plot(time,Flim3(:,1).^2 + Flim3(:,1).^2,'b-');
title('Flim');

figure
hold on
plot(time,Fg2(:,1),'g-');
plot(time,Fg2(:,2),'g-.');
plot(time,Fg3(:,1),'b-');
plot(time,Fg3(:,2),'b-.');
title('Fg');

figure
hold on
plot(time,Fa1(:,1).^2 + Fa1(:,1).^2,'r-');
plot(time,Fa2(:,1).^2 + Fa2(:,1).^2,'g-');
plot(time,Fa3(:,1).^2 + Fa3(:,1).^2,'b-');
title('Fa');

figure
hold on
plot(time,km(:,1),'r-');
plot(time,km(:,2),'g-');
plot(time,km(:,3),'b-');
title('km');

figure
hold on
plot(time,dt_l1,'r-');
plot(time,dt_l2,'g-');
plot(time,dt_l3,'b-');
plot(time,dt_ld1,'r-.');
plot(time,dt_ld2,'g-.');
plot(time,dt_ld3,'b-.');
title('dtl, dtld');


figure
hold on
plot(time,l1,'r-');
plot(time,l2,'g-');
plot(time,l3,'b-');
title('l');

end


figure(1)
set(gcf,'Color',[1 1 1]) ;
videoPointer = VideoWriter('test.avi');
open(videoPointer)
for t=1:10:timeRange
    hold on
    line( [p12(t,1) p13(t,1)], [p12(t,2) p13(t,2)], 'Color', 'r');
    line( [p13(t,1) p3(t,1)], [p13(t,2) p3(t,2)], 'Color', 'b');
    line( [p12(t,1) p2(t,1)], [p12(t,2) p2(t,2)], 'Color', 'g');    
    
    %line( [p13(t,1) q13(t,1)], [p13(t,2) q13(t,2)], 'Color', 'b');
    %line( [p12(t,1) q12(t,1)], [p12(t,2) q12(t,2)], 'Color', 'g');    
    
    %plot( p2n(t,1), p2n(t,2), 'gx');
    %plot( p3n(t,1), p3n(t,2), 'bx');    
    
    plot( q12(t,1), q12(t,2), 'gx');
    plot( q13(t,1), q13(t,2), 'bx');    
    
    plot( p2(t,1), p2(t,2), 'go');
    plot( p3(t,1), p3(t,2), 'bo');
                    
    line( [p12(t,1) p12(t,1)+Fc11(t,1)], [p12(t,2) p12(t,2)+Fc11(t,2)], ...
          'Color', 'k');
    line( [p13(t,1) p13(t,1)-Fc11(t,1)], [p13(t,2) p13(t,2)-Fc11(t,2)], ...
          'Color', 'k');    
    
    line( [q12(t,1) q12(t,1)+Fk12(t,1)], [q12(t,2) q12(t,2)+Fk12(t,2)], ...
          'Color', 'k');
    
    line( [q13(t,1) q13(t,1)+Fk13(t,1)], [q13(t,2) q13(t,2)+Fk13(t,2)], ...
          'Color', 'k');
    
    
    thDistance=0.1;
    th12direction = thDistance*[cos(th12(t));sin(th12(t))];
    th13direction = thDistance*[cos(th13(t));sin(th13(t))];
    line( [p12(t,1) p12(t,1)+th12direction(1)], [p12(t,2) p12(t,2)+th12direction(2)], ...
          'Color', 'k');
    line( [p13(t,1) p13(t,1)+th13direction(1)], [p13(t,2) p13(t,2)+th13direction(2)], ...
          'Color', 'k');
    
    
    %line( [p1(t,1) p1(t,1)+vd(t,1)], [p1(t,2) p1(t,2)+vd(t,2)], ...
    %      'Color', 'k');     
    %line( [xx1(t,1) xx1(t,1)+vdl1(t,1)], [xx1(t,2) xx1(t,2)+vdl1(t,2)], ...
    %      'Color','r');    
    %line( [xx1(t,1) xx1(t,1)+vdl2(t,1)], [xx1(t,2) xx1(t,2)+vdl2(t,2)], ...
    %      'Color','g');    
    %line( [xx1(t,1) xx1(t,1)+vdl3(t,1)], [xx1(t,2) xx1(t,2)+vdl3(t,2)], ...
    %      'Color','b');     
        
    line( [-10 10], [0 0]);  %gournd level
    axis equal
    
    xlim([-2 4]);
    ylim([-1 2]);
    
    %if t > 2
    drawnow;
    frameList = getframe(gcf);    
    writeVideo(videoPointer,frameList);    
    clf;
    %end
end

close(videoPointer);


if 0
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
end
%close all;

%copy result to matrix
[timeRange,n]=size(result);
for t = 1:1:timeRange
    xx1(t,:)=result(t).xx1;
    xx2(t,:)=result(t).xx2;
    xx3(t,:)=result(t).xx3;
    %xx02(t,:)=result(t).xx02;
    %xx03(t,:)=result(t).xx03;
    Flim1(t,:)=result(t).Flim1;
    Flim2(t,:)=result(t).Flim2;
    Flim3(t,:)=result(t).Flim3;
    Fg2(t,:)=result(t).Fg2;
    Fg3(t,:)=result(t).Fg3;
    Fa1(t,:)=result(t).Fa1;
    Fa2(t,:)=result(t).Fa2;
    Fa3(t,:)=result(t).Fa3;
    vd(t,:)=result(t).vd;
    vdl1(t,:)=result(t).vdl(:,1);
    vdl2(t,:)=result(t).vdl(:,2);
    vdl3(t,:)=result(t).vdl(:,3);
    ex1(t,:)=result(t).ex1;
    ex2(t,:)=result(t).ex2;
    ex3(t,:)=result(t).ex3;
    l1(t,:)=result(t).l1;
    l2(t,:)=result(t).l2;
    l3(t,:)=result(t).l3;
    dt_l1(t,:)=result(t).dt_l1;
    dt_l2(t,:)=result(t).dt_l2;
    dt_l3(t,:)=result(t).dt_l3;
    dt_ld1(t,:)=result(t).dt_ld(:,1);
    dt_ld2(t,:)=result(t).dt_ld(:,2);
    dt_ld3(t,:)=result(t).dt_ld(:,3);
    
    km(t,:)=result(t).km';    
end
time=((1:1:timeRange)*result(1).param.simulator.h)';

%now start plotting
figure
hold on
plot(xx1(:,1),xx1(:,2),'r-');
plot(xx2(:,1),xx2(:,2),'g-');
plot(xx3(:,1),xx3(:,2),'b-');
axis equal

%figure
%hold on
%plot(xx02(:,1),xx02(:,2),'g-');
%plot(xx03(:,1),xx03(:,2),'b-');
%axis equal
%title('xx0');

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
title('dt_l');

figure
hold on
plot(time,dt_ld1,'r-');
plot(time,dt_ld2,'g-');
plot(time,dt_ld3,'b-');
title('dt_ld');


figure
hold on
plot(time,l1,'r-');
plot(time,l2,'g-');
plot(time,l3,'b-');
title('l');

figure
videoPointer = VideoWriter('test.avi');
open(videoPointer)
for t=1:100:timeRange
    hold on
    line( [xx1(t,1) xx2(t,1)], [xx1(t,2) xx2(t,2)], 'Color', 'g');
    line( [xx2(t,1) xx3(t,1)], [xx2(t,2) xx3(t,2)], 'Color', 'r');
    line( [xx3(t,1) xx1(t,1)], [xx3(t,2) xx1(t,2)], 'Color', 'b');    
    line( [xx1(t,1) xx1(t,1)+vd(t,1)], [xx1(t,2) xx1(t,2)+vd(t,2)], ...
          'Color', 'k');    

    %line( [xx1(t,1) xx1(t,1)+ex2(t,1)], [xx1(t,2) xx1(t,2)+ex2(t,2)], ...
    %      'Color','g');    
    %line( [xx1(t,1) xx1(t,1)+ex3(t,1)], [xx1(t,2) xx1(t,2)+ex3(t,2)], ...
    %      'Color','b');     
    
    line( [xx1(t,1) xx1(t,1)+vdl1(t,1)], [xx1(t,2) xx1(t,2)+vdl1(t,2)], ...
          'Color','r');    
    line( [xx1(t,1) xx1(t,1)+vdl2(t,1)], [xx1(t,2) xx1(t,2)+vdl2(t,2)], ...
          'Color','g');    
    line( [xx1(t,1) xx1(t,1)+vdl3(t,1)], [xx1(t,2) xx1(t,2)+vdl3(t,2)], ...
          'Color','b');     
        
    line( [-10 10], [0 0]);  %gournd level
    axis equal
      
    xlim([-2 4]);
    ylim([-1 2]);
    
    frame = getframe(gcf);
    writeVideo(videoPointer,frame);                    
    clf;
end
close(videoPointer);

figure;




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
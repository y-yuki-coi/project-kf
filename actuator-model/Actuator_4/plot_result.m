close all;

%copy result to matrix
[timeRange,n]=size(result);
for t = 1:1:timeRange
    xx1(t,:)=result(t).xx1;
    xx2(t,:)=result(t).xx2;
    xx3(t,:)=result(t).xx3;
    Flim1(t,:)=result(t).Flim1;
    Flim2(t,:)=result(t).Flim2;
    Flim3(t,:)=result(t).Flim3;
%     Flimb(t,:)=result(t).Flimb;
    Fai(t,:)=result(t).Fai;
    Fg2(t,:)=result(t).Fg2;
    Fg3(t,:)=result(t).Fg3;
    Fa1(t,:)=result(t).Fa1;
    Fa2(t,:)=result(t).Fa2;
    Fa3(t,:)=result(t).Fa3;
    vd(t,:)=result(t).vd;
    vdl1(t,:)=result(t).vdl(:,1);
    vdl2(t,:)=result(t).vdl(:,2);
    vdl3(t,:)=result(t).vdl(:,3);
    vdci(:,:,:,t)=cat(3,result(t).vdc(:,2:3,1),result(t).vdc(:,1:2:3,2),result(t).vdc(:,1:2,3));
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
    vd_childa1(t,:)=result(t).vd_childa(:,1);
    vd_childa2(t,:)=result(t).vd_childa(:,2);
    vd_childa3(t,:)=result(t).vd_childa(:,3);
    
    km(t,:)=result(t).km';   
%     kf(t,:)=result(t).kf';   
end
time=((1:1:timeRange)*result(1).param.simulator.h)';

%now start plotting
int = 1:2000 ;
if 0
figure
hold on
plot(xx1(:,1),xx1(:,2),'r-');
plot(xx2(:,1),xx2(:,2),'g-');
plot(xx3(:,1),xx3(:,2),'b-');
axis equal

% figure
% hold on
% plot(xx02(:,1),'g-');
% plot(xx03(:,1),'b-');

figure
hold on
plot(time,Flim1(:,1).^2 + Flim1(:,1).^2,'r-');
plot(time,Flim2(:,1).^2 + Flim2(:,1).^2,'g-');
plot(time,Flim3(:,1).^2 + Flim3(:,1).^2,'b-');
title('Flim');

figure
hold on
plot(time(int),Fg2(int,1).^2 + Fg2(int,1).^2,'g-');
plot(time(int),Fg3(int,1).^2 + Fg3(int,1).^2,'b-');
title('Fg');

figure
hold on
% plot(time,sqrt(Fa1(:,1).^2 + Fa1(:,2).^2),'r-');
% plot(time,sqrt(Fa2(:,1).^2 + Fa2(:,1).^2),'g-');
% plot(time,sqrt(Fa3(:,1).^2 + Fa3(:,1).^2),'b-');
plot(time,Fai(:,1),'r-');
plot(time,Fai(:,2),'g-');
plot(time,Fai(:,3),'b-');
title('Fa');

figure
hold on
plot(time,km(:,1),'r-');
plot(time,km(:,2),'g-');
plot(time,km(:,3),'b-');
title('km');

figure
hold on
plot(time,kf(:,1),'r-');
plot(time,kf(:,2),'g-');
plot(time,kf(:,3),'b-');
title('kf');

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
plot(time,l2,'g-')
plot(time,l3,'b-');
title('l');

figure
hold on
plot(time,vd_childa1(:,1),'r-');
plot(time,vd_childa2(:,1),'g-');
plot(time,vd_childa3(:,1),'b-');
title('vd_childa');

figure
hold on
% plot(time,ex1,'r-');
% plot(time,ex2,'g-');
plot(time,ex3(:,1),'b-');
plot(time,ex3(:,2),'b:');
title('ex');
end

if 0
figure
videoPointer = VideoWriter('test.avi');
open(videoPointer)
for t=1:1:timeRange
    hold on
    line( [xx1(t,1) xx2(t,1)], [xx1(t,2) xx2(t,2)]);
    line( [xx2(t,1) xx3(t,1)], [xx2(t,2) xx3(t,2)]);
    line( [xx3(t,1) xx1(t,1)], [xx3(t,2) xx1(t,2)]);    
    line( [xx1(t,1) xx1(t,1)+vd(t,1)], [xx1(t,2) xx1(t,2)+vd(t,2)]);    

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
        
    line( [-10 10], [0 0]);  %ground level
    axis equal
      
    xlim([-2 4]);
    ylim([-1 2]);
    
    frame = getframe(gcf);
    writeVideo(videoPointer,frame);                    
    clf;
end
close(videoPointer);
end

figure;


% Movie
if 1
figure(1)
set(gcf,'Color',[1 1 1]) ;
skip = 100 ;
nn = 1 ; % mov index
videoPointer = VideoWriter('test.avi');
open(videoPointer)

clr_1 = {'k','r','b'} ; clr_2 = {'r','b';'k','b';'k','r'} ;
dev_1 = [0 -0.1; 0.1 0;-0.1 0];
a_f = 0.0005 ;
for t = 1:skip:length(time) %7%
    xit = xi(t,:); 
    plot(xit([1 3]),xit([2 4]),'ko:',xit([1 5]),xit([2 6]),'ko:',xit([3 5]),xit([4 6]),'ko:'); hold on;
    vdl(1,:) = vdl1(t,:); vdl(2,:) = vdl2(t,:); vdl(3,:) = vdl3(t,:);
    vdc = vdci(:,:,:,t); 
    Fgi(1,:) = Fg2(t,:); Fgi(2,:) = Fg3(t,:);
    xi2(1,1:2) = (3*xit(3:4)+xit(5:6))/4; xi2(1,3:4) = (xit(3:4)+3*xit(5:6))/4;
    xi2(2,1:2) = (3*xit(3:4)+xit(1:2))/4; xi2(2,3:4) = (xit(3:4)+3*xit(1:2))/4; 
    xi2(3,1:2) = (3*xit(1:2)+xit(5:6))/4; xi2(3,3:4) = (xit(1:2)+3*xit(5:6))/4;
    Fait(1,:) = a_f*Fa1(t,:) ; Fait(2,:) = a_f*Fa2(t,:) ; Fait(3,:) = a_f*Fa3(t,:) ;
    for i = 1:3
        quiver(xit(2*i-1),xit(2*i),vdl(i,1),vdl(i,2),0.1,'color',clr_1{i})
%         quiver(xit(2*i-1),xit(2*i),vdc(1,1,i),vdc(2,1,i),0.1,'color',clr_2{i,1})
%         quiver(xit(2*i-1),xit(2*i),vdc(1,2,i),vdc(2,2,i),0.1,'color',clr_2{i,2})
        quiver(xi2(i,1)+dev_1(i,1),xi2(i,2)+dev_1(i,2),-Fait(i,1),-Fait(i,2),0.1,'color','k')
        quiver(xi2(i,3)+dev_1(i,1),xi2(i,4)+dev_1(i,2),Fait(i,1),Fait(i,2),0.1,'color','k')
    end
    line([-10 10],[0 0],'color','k');
    axis equal
%     xlim([-1 3]) ;
    ylim([-0.5 2]) ; 
    xlim([-5 5]) ; 
    hold off ;
    title(sprintf('time = %0.2f (s) km1 = %0.2f km2 = %0.2f km3 = %0.2f ',time(t),...
        km(t,1),km(t,2),km(t,3)),'fontsize',7); % kf1 = %0.2f kf2 = %0.2f kf3 = %0.2f ,kf(t,1),kf(t,2),kf(t,3)
    set(gca,'fontsize',10) ;
    frame = getframe(gcf);
    writeVideo(videoPointer,frame);                    
    clf;
end

close(videoPointer)
% saveas(gcf,'example.pdf');
end
% Wheel_Mobility_160603.m
% 車輪で二足移動をモデル化
% Fujii & Yoshihara（このコードはFujii作成）
clear; close all;
% Parameters
global g h mi li  Gi mu mu0 hw hwb hwt ws
g = 9.8 ; h = 10^(-2) ;
mi = [48 11] ; % [kg] 1: HAT 2: leg
li = [0.5 0.7 0.7 0.6 0.1] ; % [m] 1: pelvis width 2: leg length, 3: trunk height 4: shoulder length: 5: head height
u0 = 10 ; % constrant neural input +:right
mu = 0.5 ; % dynamic friction coefficient
mu0 = 0.6 ; % maximal static friction coefficient
Gi = [0.08 1] ; % Feedback gain
hw = 0.1 ; % wheel height
hwb = 0.1 ; % wheel body height
hwt = 0.1 ; % weight height
ws = 2 ; % steering frequency
% Initial condition
x1 = 0 ; x2 = 0 ; % x1: HAT x2: center of pelvis
th1 = 10^(-2) ; th2 = 0 ; % th1: right leg phase, th2: left leg phase
th3 = 1 ; th4 = 1 ; % th3: right leg direction, th4: left leg direction
dt_x1 = 0 ; dt_x2 = 0 ; 
dt_th1 = 0 ; dt_th2 = 0 ; 
dt_th3 = 0 ; dt_th4 = 0 ; 
% x2 = x1 + (l2/2)*cos(th2) ; % center of segment
% y2 = y1 - (l2/2)*sin(th2) ; 
% x3 = x1 + (l2/2)*cos(th3) ; 
% y3 = y1 - (l2/2)*sin(th3) ; 
xi = [x1 x2 th1 th2 th3 th4 dt_x1 dt_x2 dt_th1 dt_th2 dt_th3 dt_th4] ; % vector
Body = calc_body_wheel(xi) ;
% check: 前足が右足で最初で赤色
if 0; figure(1); plot_body_wheel(Body,1)
end % center of segment

% run simulation
turn = 1 ;
th_vel = 1 ; % 切り返し指令を出す時の平均速度
int_sim_s = 0.2 ; % 切り返しシミュレーションの時間幅
int_sim_frame = int_sim_s/h ;
ut = -10 ; % 切り返し入力
th_x_turn = -1 ; % 切り返した後の目標到達距離

if 1
    tic;
    iter = 200 ;
    Y = [xi;zeros(iter,12)]; Fgi=zeros(iter,4); dt2_xi = zeros(iter,6);
    time = h:h:(10000+1)*h ;
    for t = 1:iter
        [Y(t+1,:) Fgi(t,:) dt2_xi(t,:)] = fun_Wheel(Y(t,:),u0);
        if mod(t,1000)==0; t
        end
        if turn == 1
            if t>int_sim_frame
                if Y(t-int_sim_frame,8) >= th_vel ;
                    t_th_vel = t ;
                    break
                end
            end
        end
    end
    if turn == 1
        n = 1 ;
        for st = t_th_vel-int_sim_frame:t_th_vel+1%int_sim_frame
            Yt{n,1}(1:st,:) = Y(1:st,:) ;
            x_st(n,1) = Yt{n,1}(st,2) ;
            Fgit{n,1}(1:st-1,:) = Fgi(1:st-1,:) ;
            dt2_xit{n,1}(1:st-1,:) = dt2_xi(1:st-1,:) ;
            t = st ;
            while x_st(n,1) - Yt{n,1}(t,2) < -th_x_turn
                [Yt{n,1}(t+1,:) Fgit{n,1}(t,:) dt2_xit{n,1}(t,:)] = fun_Wheel(Yt{n,1}(t,:),ut);
                t = t + 1 ;
            end
            if mod(n,10)==0; n
            end
            RT(n,1) = t - (t_th_vel-int_sim_frame) ;
            n = n + 1 ;
        end
    end
    toc;
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
v = VideoWriter('Wheel_Mobility_160603.mp4','MPEG-4');
open(v)
if turn == 0
    tmp_Y = Y ;
else n = 1 ; 
    tmp_Y = Yt{n} ; % int_sim_frame+
end
n_time = size(tmp_Y,1)-1 ;
    
for frame = 1:skip:n_time%length(time)
    subplot 411
    plot(time(1:n_time),tmp_Y(1:n_time,8)) ; hold on
    line([time(frame) time(frame)],[-3 3]) ;
    line([time(t_th_vel) time(t_th_vel)],[-3 3],'color','k','linestyle',':') ;
    title(['n = ',num2str(n),' RT = ',num2str(RT(n)*h*1000),'ms']);
    hold off
    xlim([0 time(n_time)]); ylim([-3 3]); 
    
    subplot(4,1,2:4)
    Body = calc_body_wheel(tmp_Y(frame,:)) ;
    plot_body_wheel(Body,1);
    line([tmp_Y(t_th_vel,2) tmp_Y(t_th_vel,2)],[-2.2 0.5],'color','k','linestyle',':') ;hold on
    hold off ;
    title(sprintf('time = %0.2f (s)',time(frame)),'fontsize',12);
    set(gca,'fontsize',12) ;
    mov(nn)= getframe(gcf); % mov index
    nn = nn+1;
    drawnow
end
writeVideo(v,mov)
close(v)
%movie2avi(mov,'Wheel_Mobility_160603','compression','None');
end

RT_Both1 = [283;282;442;389;371;354;345;339;333;330;328;324;324;321;505;423;399;388;377;371;365;361] ;
RT_Alt1 = [360;362;365;368;370;373;375;378;380;383;385;388;390;393;395;398;400;402;405;407;410;412] ;
RT_Both2 = [367;356;348;342;337;333;330;327;324;322;319;317;314;312;311;310;311;315;328;591;483;441] ;
RT_Alt2 = [365;369;397;423;386;367;353;343;335;329;325;321;318;317;318;321;333;439;429;440;515;446] ;
xx = cat(2,RT_Both2, RT_Alt2) ;
m = mean(xx,1); 
[hp,p,ci,stat] = ttest(RT_Both2,RT_Alt2) ;
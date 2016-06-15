function [dy var1 var2] = RKfun(y0,isRefresh) % Fgi dt2_xi l_vec th

global g mi li ki yg y_contact u0 Gi bi % Ii

xi = y0(1:6) ; dt_xi = y0(7:12) ; 

% for debug 
% t =30; % first contact h = 10^(-3) 
% t = 4296;% t = 4299 % too large GRF  h = 10^(-4) 
% t = 5246;% t = 4299 % too large GRF  h = 10^(-4) 
% xi = Y(t,1:14) ;dt_xi = Y(t,15:28) ; Y0 = Y(1:t,:); 

m1 = mi(1); m2 = mi(2); l01 = li(1); l02 = li(2); l03 = li(3); k1 = ki(1) ; k2 = ki(2) ;  kg = ki(3) ;
b1 = bi(1) ; b2 = bi(2) ; bk = bi(3) ; bg = bi(4) ; % I2 = Ii(2); 
% x1 = xi(1) ; y1 = xi(2) ; th1 = xi(3) ;x2 = xi(4) ; y2 = xi(5) ; th2 = xi(6) ; 
% x3 = xi(7) ; y3 = xi(8) ; th3 = xi(9) ;
% dt_x1 = dt_xi(1) ; dt_y1 = dt_xi(2) ; dt_th1 = dt_xi(3) ; dt_x2 = dt_xi(4) ; dt_y2 = dt_xi(5) ; dt_th2 = dt_xi(6) ; 
% dt_x3 = dt_xi(7) ; dt_y3 = dt_xi(8) ; dt_th3 = dt_xi(9) ;
x1 = xi(1) ; y1 = xi(2) ; x2 = xi(3) ; y2 = xi(4) ; x3 = xi(5) ; y3 = xi(6) ; 
dt_x1 = dt_xi(1) ; dt_y1 = dt_xi(2) ; dt_x2 = dt_xi(3) ; dt_y2 = dt_xi(4) ; dt_x3 = dt_xi(5) ; dt_y3 = dt_xi(6) ; 
l1 = sqrt((x2-x3)^2+(y2-y3)^2) ; 
l2 = sqrt((x1-x2)^2+(y1-y2)^2) ; 
l3 = sqrt((x1-x3)^2+(y1-y3)^2) ;
dt_l1 = 1/2*l1^(-1/2)*(2*(x2-x3)*(dt_x2-dt_x3)+2*(y2-y3)*(dt_y2-dt_y3)) ; % 微分データから検証済み
dt_l2 = 1/2*l2^(-1/2)*(2*(x2-x1)*(dt_x2-dt_x1)+2*(y2-y1)*(dt_y2-dt_y1)) ; 
dt_l3 = 1/2*l3^(-1/2)*(2*(x1-x3)*(dt_x1-dt_x3)+2*(y1-y3)*(dt_y1-dt_y3)) ; 
l_vec = [l1 l2 l3 dt_l1 dt_l2 dt_l3] ;
th1 = acos((x3-x2)/l1) ; 
th2 = acos((x2-x1)/l2) ; 
th3 = acos((x3-x1)/l3) ; 
th = [th1 th2 th3]/pi*180 ;
% calculate ground reaction force (GRF)-----------------------------------------------------------------
% ankle position
xr = x2 ;
yr = y2 ; 
xl = x3 ;
yl = y3 ; 
dt_xr = dt_x2 ;
dt_yr = dt_y2 ;
dt_xl = dt_x3 ;
dt_yl = dt_y3 ;

% Ground reaction force
Fgi = zeros(4,1) ; 
if 1 % 1: GRF exist 0: no GRF
    if yr < yg % contact
        if y_contact(1,7) ~= 0 % not initial & keep contact
            yr0 = yg ; % Y0(t_contact,10)-(l3/2)*sin(Y0(t_contact,11)) ; % ground y
            xr0 = y_contact(1,3) ; % ground x
        else % first contact in one cycle
            if isRefresh == 1 % Update
                y_contact(1,1:7) = [xi(1:6) 1] ;
            end
            yr0 = yr ; xr0 = xr ;
        end
        Fgi(1) = -kg*(xr-xr0) - bg*dt_xr ;
        Fgi(2) = -kg*(yr-yr0) + bg*f_max(-dt_yr) ;
    else Fgi(1:2) = 0 ; % non-contact
        y_contact(1,1:13) = zeros(1,13) ;
    end
    
    if yl < yg % contact
        if y_contact(2,7) ~= 0 % not initial & keep contact
            yl0 = yg ; % ground y
            xl0 = y_contact(2,5) ; % ground x
        else % first contact in one cycle
            if isRefresh == 1 % Update
                y_contact(2,1:7) = [xi(1:6) 1] ;
            end
            yl0 = yl ; xl0 = xl ;
        end
        Fgi(3) = -kg*(xl-xl0) - bg*dt_xl ;
        Fgi(4) = -kg*(yl-yl0) + bg*f_max(-dt_yl) ;
    else Fgi(3:4) = 0 ; % non-contact
        y_contact(2,1:7) = zeros(1,7) ;
    end
    
end

% Actuator force -----------------------------------------------------------------------------------
% Mobility 
% u0がx方向の目標速度とする
eps1 = 10^(-10) ; eps2 = 10^(-4) ; % 零の除算を避けるための微小量 
vd = [u0 0] ; % 目標速度
if 1 % 1: Mobility control
    ex1 = [-cos(th2) sin(th2)];
    ex2 = [-cos(th3) sin(th3)];
    ex3 = [-cos(th1) sin(th1)];
    % right leg
    vdl(1,1:2) = dot(ex1,vd)*ex1 ; % 局所速度ベクトル（本来寄与できるベクトル）
    vdr(1,1:2) = vd - vdl(1,1:2) ; % 本来寄与できない速度ベクトル
    vdc(1,1:2,2) = dot(ex2,vdr(1,1:2))*ex2 ; % 左に対して右が寄与できる配分
    vdc(1,1:2,3) = dot(ex3,vdr(1,1:2))*ex3 ; % 上に対して右が寄与できる配分
    vi(1) = dot([dt_x2,dt_y2],ex1); % 脚に沿った現在速度
    if sum(Fgi(1:2)) == 0 % non-contact 
       k(1) = 0 ; 
    else % contact
        k(1) = exp(-4*log2(norm(vdl(1,1:2)-vi(1)*ex1)^2+eps1)/(norm(vdl(1,1:2))^2+eps2)) ; % vdlとviの符号が逆だと1を超える
        if k(1) >= 1 ; k(1) = 1; end
    end
    % left leg
    vdl(2,1:2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    vdr(2,1:2) = vd - vdl(2,1:2) ; % 本来寄与できない速度ベクトル
    vdc(2,1:2,1) = dot(ex1,vdr(2,1:2))*ex1 ; % 右に対して左が寄与できる配分
    vdc(2,1:2,3) = dot(ex3,vdr(2,1:2))*ex3 ; % 上に対して左が寄与できる配分
    vi(2) = dot([dt_x3,dt_y3],ex2); % 脚に沿った現在速度
%     if sum(Fgi(3:4)) == 0 % non-contact left
%        k(2) = 0 ;
%     else Fgi(3:4) = 0 ; % contact
        k(2) = exp(-4*log2(norm(vdl(2,1:2)-vi(2)*ex2)^2+eps1)/(norm(vdl(2,1:2))^2+eps2)) ; % 伸縮速度dt_l3でなく現在速度を入れた
        if k(2) >= 1 ; k(2) = 1; end
%     end
    % Top
    vdl(3,1:2) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    vdr(3,1:2) = vd - vdl(3,1:2) ; % 本来寄与できない速度ベクトル
    vdc(3,1:2,1) = dot(ex1,vdr(3,1:2))*ex2 ; % 右に対して上が寄与できる配分
    vdc(3,1:2,2) = dot(ex2,vdr(3,1:2))*ex3 ; % 左に対して上が寄与できる配分
    vi(3) = dot([dt_x1,dt_y1],ex3); % 脚間に沿った現在速度
    k(3) = exp(-4*log2(norm(vdl(3,1:2)-vi(3)*ex3)^2+eps1)/(norm(vdl(3,1:2))^2+eps2)) ; % 伸縮速度dt_l1でなく現在速度を入れた
    if k(3) >= 1 ; k(3) = 1; end
    vd_childa(1,1:2) = (1-k(2))*(1-k(3))*vdl(1,1:2) + k(2)*vdc(2,1:2,1) + k(3)*vdc(3,1:2,1) ; % 右に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(2,1:2) = (1-k(1))*(1-k(3))*vdl(2,1:2) + k(1)*vdc(1,1:2,2) + k(3)*vdc(3,1:2,2); % 左に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(3,1:2) = (1-k(1))*(1-k(2))*vdl(3,1:2) + k(1)*vdc(1,1:2,3) + k(2)*vdc(2,1:2,3) ;
    % 出力へ変換 
    vdi(1) = dot(vd_childa(1,1:2),ex1) ; % 出力方向に変換
    vdi(2) = dot(vd_childa(2,1:2),ex2) ; % 出力方向に変換
    vdi(3) = dot(vd_childa(3,1:2),ex3) ;
    for i = 1:3
        tmp = Gi*(vdi(i)-vi(i)) ;
%         Fai(i) = tmp ;
        if abs(tmp) <= 10*Gi 
            Fai(i) = tmp ;
        else Fai(i) = sign(tmp)*10*Gi ;
        end
    end
else % どのようにアクチュエータの入力を作るか未定の場合
    Fai = zeros(3,1);
    Fai = ones(3,1)*Gi;%zeros(3,1);
    vdl = zeros(3,2);
    vi = zeros(1,3); k = zeros(1,3);
end

Fa2 = Fai(1) ; Fa3 = Fai(2) ; Fa1 = Fai(3) ;
var1 = [vdl(1,1:2) vdl(2,1:2) vdl(3,1:2)];
var2 = [vi k];
% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

% differential equantion and output
% このダイナミクスは合っていそう（確認済）
dy = zeros(1,12) ;
dy(1:6)  = dt_xi ;
dy(7)  = (- Fa1*cos(th1) ..., % HAT - Fa2*cos(th2) - Fa3*cos(th3)
                - k1*(l02-l2)*cos(th2) - k1*(l03-l3)*cos(th3) + b1*dt_l2*cos(th2) + b1*dt_l3*cos(th3))/m1 ;
dy(8)  = (- m1 * g + Fa1*sin(th1) .... % + Fa2*sin(th2) + Fa3*sin(th3) 
                + k1*(l02-l2)*sin(th2) + k1*(l03-l3)*sin(th3) - b1*dt_l2*sin(th2) - b1*dt_l3*sin(th3))/m1 ;
dy(9)  = (+ Fgx2 + Fa2*cos(th2) ...
                + k1*(l02-l2)*cos(th2) - k2*(l01-l1)*cos(th1) - b1*dt_l2*cos(th2) + b2*dt_l1*cos(th1))/m2 ; % Leg_R
dy(10) = (+ Fgy2 -m2 * g - Fa2*sin(th2) ...
                - k1*(l02-l2)*sin(th2) + k2*(l01-l1)*sin(th1) + b1*dt_l2*sin(th2) - b2*dt_l1*sin(th1))/m2 ;
dy(11) = (+ Fgx3 + Fa3*cos(th3) ...
                + k1*(l03-l3)*cos(th3) + k2*(l01-l1)*cos(th1) - b1*dt_l3*cos(th3) - b2*dt_l1*cos(th1))/m2 ; % Leg_L
dy(12) = (+ Fgy3 -m2 * g - Fa3*sin(th3) ...
                - k1*(l03-l3)*sin(th3) - k2*(l01-l1)*sin(th1) + b1*dt_l3*sin(th3) + b2*dt_l1*sin(th1))/m2 ;
dt2_xi = dy(7:12) ;
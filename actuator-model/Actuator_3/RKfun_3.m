function [dy var1 var2] = RKfun_3(y0,isRefresh) % Fgi dt2_xi l_vec th
% 20160624作成
% th1の角度の定義、左右脚の地面反力による分岐を直して、
% 動きやすさkの定義を改良したがver2よりもさらに走れなくなった
global g mi li ki yg y_contact u0 Gi bi kkf% Ii

xi = y0(1:6) ; dt_xi = y0(7:12) ; 

% for debug 
% t =30; % first contact h = 10^(-3) 
% t = 4296;% t = 4299 % too large GRF  h = 10^(-4) 
% t = 5246;% t = 4299 % too large GRF  h = 10^(-4) 
% y0 = Y(t,:); xi = y0(1:6) ;dt_xi = Y(t,7:12) ; isRefresh = 0 ;

m1 = mi(1); m2 = mi(2); l01 = li(1); l02 = li(2); l03 = li(3); k1 = ki(1) ; k2 = ki(2) ;  k3 = ki(3) ;  k4 = ki(4) ; kg = ki(5) ;
b1 = bi(1) ; b2 = bi(2) ; b3 = bi(3) ; b4 = bi(4) ; bg = bi(5) ; % I2 = Ii(2); 
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
if x2 >= x3
    if y2 >= y3
        th1 = acos((x3-x2)/l1) ;
    else th1 = 2*pi-acos((x3-x2)/l1) ;
    end
else
    if y2 >= y3
        th1 = acos((x2-x3)/l1) ;
    else th1 = 2*pi-acos((x2-x3)/l1) ;
    end
end
th2 = acos((x2-x1)/l2) ; % x1がx2よりも上にあることが前提
th3 = acos((x3-x1)/l3) ; % x1がx3よりも上にあることが前提 
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
% Fは伸展方向が正
eps1 = 10^(-10) ; eps2 = 10^(-4) ; eps3 = 10^(-4) ; % 零の除算を避けるための微小量 
vd = [u0 0] ; % 目標速度
if 1 % 1: Mobility control
    ex2 = [-cos(th2) sin(th2)];
    ex3 = [-cos(th3) sin(th3)];
    tmp1 = calc_ang_triang(l1+0.01,l2,l3) ; % l1が微小量伸びた時の角度
    if Fgi(2) ~= 0 && Fgi(4) == 0% 右脚が着地
        tmp2 = x2+l2*[cos(pi-th1+tmp1(2)) cos(pi-th1+tmp1(2))] -x1 ; % 微小量l1が伸びた時のx1の変位ベクトル
    elseif Fgi(4) ~= 0 && Fgi(2) == 0% 左脚が着地
        tmp2 = x3+l3*[cos(pi-th1+tmp1(3)) cos(pi-th1+tmp1(3))] -x1 ; % 微小量l1が伸びた時のx1の変位ベクトル
    else 
        if x2 >= x3
             tmp2 = x3+0.01*[ cos(th1) -sin(th1)]+l3*[cos(pi-th1+tmp1(3)) cos(pi-th1+tmp1(3))] -x1 ; % 微小量l1が伸びた時のx1の変位ベクトル
        else tmp2 = x2+0.01*[ cos(th1) -sin(th1)]+l2*[cos(pi-th1+tmp1(2)) cos(pi-th1+tmp1(2))] -x1 ; % 微小量l1が伸びた時のx1の変位ベクトル
        end
    end
    ex1 = tmp2./norm(tmp2);
    % right leg
    % vdl(2,1:2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(2) ~= 0 % 右脚が着地
        vdl(2,1:2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    else vdl(2,1:2) = dot(ex2,vd/2)*ex2 ; % 浮いているので寄与は半分
    end
    vi(2) = dt_l2 ;
    vdr(2,1:2) = vd - vdl(1,1:2) ; % この関節が生成できない速度ベクトル
    vdc(2,1:2,3) = dot(ex3,vdr(1,1:2))*ex3 ; % 左が右に寄与できる配分
    vdc(2,1:2,1) = dot(ex1,vdr(1,1:2))*ex1 ; % 下が右に寄与できる配分
    k(2) = exp(-4*log(2)*(norm(vdl(2,1:2)-dt_l2*ex2)^2+eps1)/(norm(vdl(2,1:2))^2+eps2)) ; % 1を超えないように設計されている
    % figure(1); plot(0:0.01:1,exp(-4*log(2).*[0:0.01:1])) % check
    % left leg
%     vdl(3,1:2) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(3) ~= 0 % 左脚が着地
        vdl(3,1:2) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    else vdl(3,1:2) = dot(ex3,vd/2)*ex3 ; % 浮いているので寄与は半分
    end
    vi(3) = dt_l3 ;
    vdr(3,1:2) = vd - vdl(3,1:2) ; % この関節が生成できない速度ベクトル
    vdc(3,1:2,2) = dot(ex2,vdr(2,1:2))*ex2 ; % 右が左に寄与できる配分
    vdc(3,1:2,1) = dot(ex1,vdr(2,1:2))*ex1 ; % 下が左に寄与できる配分
    k(3) = exp(-4*log(2)*(norm(vdl(3,1:2)-dt_l2*ex3)^2+eps1)/(norm(vdl(3,1:2))^2+eps2)) ; %
    % Hip
    vi(1) = dt_l1; % 脚間に沿った現在速度
%     vdl(1,1:2) = dot(ex1,vd)*ex1 ; %
%     k(1) = exp(-4*log(2)*(norm(vdl(1,1:2)-dt_l1*ex1)^2+eps1)/(norm(vdl(1,1:2))^2+eps2)) ; %
%     vdr(1,1:2) = vd - vdl(1,1:2) ; % この関節が生成できない速度ベクトル
    if Fgi(2) == 0 || Fgi(4) == 0% どちらかの脚が離地
        vdl(1,1:2) = dot(ex1,vd)*ex1 ; %
        k(1) = exp(-4*log(2)*(norm(vdl(1,1:2)-dt_l1*ex1)^2+eps1)/(norm(vdl(1,1:2))^2+eps2)) ; %
        vdr(1,1:2) = vd - vdl(1,1:2) ; % この関節が生成できない速度ベクトル
    else % 両脚支持期
        vdl(1,1:2) = [0 0] ; %
        k(1) = exp(-4*log(2)*(norm(dt_l1*ex1)^2+eps1)/(eps2)) ;
        vdr(1,1:2) = [0 0] ; % この関節が生成できない速度ベクトル
    end
    vdc(1,1:2,2) = dot(ex2,vdr(1,1:2))*ex2 ; % 右が下に寄与できる配分
    vdc(1,1:2,3) = dot(ex3,vdr(1,1:2))*ex3 ; % 左が下に寄与できる配分

    % 全体の相互作用
    vd_childa(1,1:2) = (1-k(2))*(1-k(3))*vdl(1,1:2) + k(2)*vdc(2,1:2,1) + k(3)*vdc(3,1:2,1) ; % 下に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(2,1:2) = (1-k(1))*(1-k(3))*vdl(2,1:2) + k(1)*vdc(1,1:2,2) + k(3)*vdc(3,1:2,2) ; % 右に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(3,1:2) = (1-k(1))*(1-k(2))*vdl(3,1:2) + k(1)*vdc(1,1:2,3) + k(2)*vdc(2,1:2,3) ;
    % 出力へ変換 
    vdi(1) = dot(vd_childa(1,1:2),ex1) ; % 出力方向に変換
    vdi(2) = dot(vd_childa(2,1:2),ex2) ; % 出力方向に変換
    vdi(3) = dot(vd_childa(3,1:2),ex3) ;
    for i = 1:3
        tmp = Gi(i)*(vdi(i)-vi(i)) ;
        Fai(i) = tmp ;
    end
    % （前のめりにならないための）倒れやすさ制御------------------------------------------------
    % kfは0が倒れやすい、1が倒れにくい→1が倒れやすい、0が倒れにくい
    % right leg
    % tmp = (dt_x1-dt_x2) + sqrt(g/l2)*(x1-x2) ; % Kagawa & Uno 2010
    % tmp = (dt_x1-dt_x2)^2 + (g/l2)*(x1-x2)^2 ; % 原点と状態点の距離　Kagawa & Uno 2010
    tmp = abs(dt_x1-dt_x2 + (g/l2)*(x1-x2))/sqrt(2); % 状態点と安定多様体との距離　Kagawa & Uno 2010 y = -x に対して、abs(y0-1*x0+0)/sqrt(1+1^2)
    kf(2) = exp(-1/abs(tmp)) ; % 0.5に相当する値は何か？ vdl,vdcは単位ベクトルなので一緒でいい？（目標が倒れないなので設定しにくい）
    % left leg
    % tmp = (dt_x1-dt_x3) + sqrt(g/l3)*(x3-x3) ; % Kagawa & Uno 2010 
    % tmp = (dt_x1-dt_x3)^2 + (g/l3)*(x1-x3)^2 ;
    tmp = abs(dt_x1-dt_x3 + (g/l2)*(x1-x3))/sqrt(2); % 原点と安定多様体との距離　Kagawa & Uno 2010
    % kf(3) = sign(tmp)*exp(-1/abs(tmp)) ;
    kf(3) = exp(-1/abs(tmp)) ;
    % Hip: 自身の倒れやすさはゼロ
    kf(1) = 0 ;
    % 二脚を助ける成分：接地離地で分類する必要あり？
%     if Fgi(2) == 0 && Fgi(4) ~= 0% 左脚が着地 右脚が離地
%         kf2vdc1 = 0 ;
%         kf3vdc1 = kf(3)*vdc(3,1:2,1) ; % x2>x3でも逆でも、kf(3)>0でも逆でも
%     elseif Fgi(4) == 0 && Fgi(2) ~= 0% 右脚が着地 左脚が離地
%         kf3vdc1 = 0 ;
%         kf2vdc1 = kf(2)*vdc(2,1:2,1) ; % x2>x3でも逆でも、kf(3)>0でも逆でも
    if Fgi(2) == 0 || Fgi(4) == 0% どちらかの脚が離地
        kf2vdc1 = kf(2)*vdc(2,1:2,1) ;
        kf3vdc1 = kf(3)*vdc(3,1:2,1) ; % x2>x3でも逆でも、kf(3)>0でも逆でも
    else % 両脚支持期
        kf2vdc1 = 0 ; kf3vdc1 = 0 ;
    end
    % 全体の相互作用
%     vdf_childa(1,1:2) = (1-kf(2))*vdc(2,1:2,1) + (1-kf(3))*vdc(3,1:2,1) ; % 自分は倒れることに直接関係しないので、助けるのみ 動きやすさと係数が逆でOK？
%     vdf_childa(2,1:2) = kf(3)*vdl(2,1:2) + (1-kf(3))*vdc(3,1:2,2) ; % この行第一項が脚を上げる成分（正しいかはわからない）
%     vdf_childa(3,1:2) = kf(2)*vdl(2,1:1) + (1-kf(2))*vdc(2,1:2,3) ; % この行第一項が脚を上げる成分（正しいかはわからない）
%     vdf_childa(1,1:2) = kf(2)*vdc(2,1:2,1) + kf(3)*vdc(3,1:2,1) ; % 自分は倒れることに直接関係しないので、助けるのみ
    vdf_childa(1,1:2) = kf2vdc1 + kf3vdc1 ; % 自分は倒れることに直接関係しないので、助けるのみ
%     vdf_childa(2,1:2) = (1-kf(3))*vdl(2,1:2) + kf(3)*vdc(3,1:2,2) ; % この行第一項が脚を上げる成分（正しいかはわからない）
%     vdf_childa(3,1:2) = (1-kf(2))*vdl(2,1:1) + kf(2)*vdc(2,1:2,3) ; % この行第一項が脚を上げる成分（正しいかはわからない）
    amp = 5 ;
    if abs(kf(2)-kf(3)) < 0.3
        if sign(u0)*sign(x2-x3) == 1 % 前脚が、2の脚
            vdf_childa(2,1:2) =   amp*(1-kf(3))*vdl(2,1:2) + 1/amp*kf(3)*vdc(3,1:2,2) ; % 
            vdf_childa(3,1:2) = 1/amp*(1-kf(2))*vdl(2,1:1) +   amp*kf(2)*vdc(2,1:2,3) ; % 
        else
            vdf_childa(2,1:2) = 1/amp*(1-kf(3))*vdl(2,1:2) +   amp*kf(3)*vdc(3,1:2,2) ; % 
            vdf_childa(3,1:2) =   amp*(1-kf(2))*vdl(2,1:1) + 1/amp*kf(2)*vdc(2,1:2,3) ; % 
        end
    else
        vdf_childa(2,1:2) = (1-kf(3))*vdl(2,1:2) + kf(3)*vdc(3,1:2,2) ; %
        vdf_childa(3,1:2) = (1-kf(2))*vdl(2,1:1) + kf(2)*vdc(2,1:2,3) ; %
    end
    % 出力へ変換 
    vdfi(1) = dot(kkf*vdf_childa(1,1:2),ex1) ; % 出力方向に変換
    vdfi(2) = dot(kkf*vdf_childa(2,1:2),ex2) ; % 出力方向に変換
    vdfi(3) = dot(kkf*vdf_childa(3,1:2),ex3) ;
    for i = 1:3
        tmp = Gi(i+3)*(vdfi(i)-vi(i)) ;
        Fai(i) = tmp + Fai(i);
    end
else % どのようにアクチュエータの入力を作るか未定の場合
    Fai = zeros(3,1);
    Fai = ones(3,1)*Gi;%zeros(3,1);
    vdl = zeros(3,2);
    vi = zeros(1,3); k = zeros(1,3); kf = zeros(1,3);
end

Fa1 = Fai(1) ;%0;%
Fa2 = Fai(2) ; Fa3 = Fai(3) ;
var1 = [l_vec vdl(1,1:2) vdl(2,1:2) vdl(3,1:2) vdc(1,1:2,2) vdc(1,1:2,3) vdc(2,1:2,1) vdc(2,1:2,3) vdc(3,1:2,1) vdc(3,1:2,2)]; 

% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

% バネ定数を接地離地・短縮進展で変更する
if Fgi(2) ~= 0 % 右脚が着地
    if l02-l2 < 0 ; k_leg1 = k1 ; b_leg1 = b1 ; % 右脚が着地
    else k_leg1 = k2 ; b_leg1 = b2 ;
    end
else 
    if l02-l2 < 0 ; k_leg1 = k3 ; b_leg1 = b3 ; % 右脚が離地
    else k_leg1 = k4 ; b_leg1 = b4 ;
    end
end

if Fgi(4) ~= 0% 左脚が着地
    if l03-l3 < 0 ; k_leg2 = k1 ; b_leg2 = b1 ; % 左脚が着地
    else k_leg2 = k2 ; b_leg2 = b2 ;
    end
else 
    if l03-l3 < 0 ; k_leg2 = k3 ; b_leg2 = b3 ; % 左脚が離地
    else k_leg2 = k4 ; b_leg2 = b4 ;
    end    
end

if l01-l1 < 0 ; k_hip = k3 ; b_hip = b3 ;
else k_hip = k4 ; b_hip = b4 ;
end

% differential equantion and output
% このダイナミクスは合っていそう（確認済）
if x2>=x3
     costh1 =  cos(th1) ; sinth1 =  sin(th1) ;
else costh1 = -cos(th1) ; sinth1 = -sin(th1) ;
end
var2 = [vi k kf -Fa1*costh1 Fa1*sinth1 -Fa2*cos(th2) Fa2*sin(th2) -Fa3*cos(th3) Fa3*sin(th3) Fgi' th];
dy = zeros(1,12) ;
dy(1:6)  = dt_xi ;
dy(7)  = (- Fa2*cos(th2) - Fa3*cos(th3) ..., % HAT 
                - k_leg1*(l02-l2)*cos(th2) - k_leg2*(l03-l3)*cos(th3) + b_leg1*dt_l2*cos(th2) + b_leg2*dt_l3*cos(th3))/m1 ;
dy(8)  = (- m1 * g + Fa2*sin(th2) + Fa3*sin(th3)  .... % 
                + k_leg1*(l02-l2)*sin(th2) + k_leg2*(l03-l3)*sin(th3) - b_leg1*dt_l2*sin(th2) - b_leg2*dt_l3*sin(th3))/m1 ;
dy(9)  = (+ Fgx2 - Fa1*costh1 + Fa2*cos(th2) ...
                + k_leg1*(l02-l2)*cos(th2) - k_hip*(l01-l1)*costh1 - b_leg1*dt_l2*cos(th2) + b_hip*dt_l1*costh1)/m2 ; % Leg_R
dy(10) = (+ Fgy2 -m2 * g + Fa1*sinth1 - Fa2*sin(th2) ...
                - k_leg1*(l02-l2)*sin(th2) + k_hip*(l01-l1)*sinth1 + b_leg1*dt_l2*sin(th2) - b_hip*dt_l1*sinth1)/m2 ;
dy(11) = (+ Fgx3 + Fa1*costh1 + Fa3*cos(th3) ...
                + k_leg2*(l03-l3)*cos(th3) + k_hip*(l01-l1)*costh1 - b_leg2*dt_l3*cos(th3) - b_hip*dt_l1*costh1)/m2 ; % Leg_L
dy(12) = (+ Fgy3 -m2 * g - Fa1*sinth1- Fa3*sin(th3) ...
                - k_leg2*(l03-l3)*sin(th3) - k_hip*(l01-l1)*sinth1 + b_leg2*dt_l3*sin(th3) + b_hip*dt_l1*sinth1)/m2 ;
dt2_xi = dy(7:12) ;
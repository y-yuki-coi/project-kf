function [dy result] = RKfun_4(time,y0,param,isRefresh) 
% 20160702作成
% 吉原さんの修正から、自分で解読して再修正 

% for debug 
% y0 = Y(t,:); isRefresh = 0 ; isFirstContact2 = false; isFirstContact3 = false; xx02 = []; xx03 = []; 

%rename params
k_leg21=param.k_leg21;
k_leg31=param.k_leg31;
k_leg22=param.k_leg22;
k_leg32=param.k_leg32;
b_leg2=param.b_leg2;
b_leg3=param.b_leg3;
gg = param.gg;
l0 = param.l0;
m1 = param.m1;
m2 = param.m2;
m3 = param.m3;
klim1 = param.klim1;
klim2 = param.klim2;
klim3 = param.klim3;
blim1 = param.blim1;
blim2 = param.blim2;
blim3 = param.blim3;
llim1 = param.llim1;
llim2 = param.llim2;
llim3 = param.llim3;
kg = param.kg;
bg = param.bg; 
G1 = param.G1; 
G2 = param.G2; 
G3 = param.G3; 
Gi = [G1; G2; G3];
vdx = param.vdx;
desiredHeight=param.desiredHeight;
eps1 = param.eps1;
eps2 = param.eps2;
eps3 = param.eps3;
yg = param.yg;
ckf = param.ckf;
xi = y0(1:6) ; dt_xi = y0(7:12) ; 

l01 = l0; l02 = l0; l03 = l0; 
x1 = xi(1) ; y1 = xi(2) ; 
x2 = xi(3) ; y2 = xi(4) ; 
x3 = xi(5) ; y3 = xi(6) ; 
dt_x1 = dt_xi(1); dt_y1 = dt_xi(2); 
dt_x2 = dt_xi(3); dt_y2 = dt_xi(4); 
dt_x3 = dt_xi(5); dt_y3 = dt_xi(6); 
xx1 = [x1; y1];%top position
xx2 = [x2; y2];%foretip
xx3 = [x3; y3];%hindtip
dxx1 = [dt_x1; dt_y1];%top position
dxx2 = [dt_x2; dt_y2];%foretip
dxx3 = [dt_x3; dt_y3];%hindtip

d23=(x2-x3)^2+(y2-y3)^2;
d12=(x1-x2)^2+(y1-y2)^2; 
d31=(x1-x3)^2+(y1-y3)^2;
l1 = sqrt(d23); 
l2 = sqrt(d12); 
l3 = sqrt(d31);
%CHECK
dt_l1 = 1/2*d23^(-1/2)*(2*(x2-x3)*(dt_x2-dt_x3)+2*(y2-y3)*(dt_y2-dt_y3)); 
dt_l2 = 1/2*d12^(-1/2)*(2*(x2-x1)*(dt_x2-dt_x1)+2*(y2-y1)*(dt_y2-dt_y1)); 
dt_l3 = 1/2*d31^(-1/2)*(2*(x1-x3)*(dt_x1-dt_x3)+2*(y1-y3)*(dt_y1-dt_y3)); 

% calculate ground reaction force (GRF)-----------------------------------------------------------------

Fgi = zeros(4,1) ; 
persistent isFirstContact2;
persistent isFirstContact3;
persistent xx02; % デバッグはどのようにして行う？ 
persistent xx03;

if isempty(isFirstContact2)
    isFirstContact2 = true;
end

if isempty(isFirstContact3)
    isFirstContact3 = true;
end

if xx2(2) < yg %contact
    if isempty(xx02) || (isFirstContact2 && isRefresh)
        xx02 = xx2;
        isFirstContact2 = false;
    else
        %do nothing
    end
else
    isFirstContact2 = true;
end

if xx3(2) < yg %contact
    if isempty(xx03) || (isFirstContact3 && isRefresh)
        xx03 = xx3;
        isFirstContact3 = false;
    else
        %do nothing
    end
else
    isFirstContact3 = true;
end

Fg2 = [0;0];
if xx2(2) < yg
    Fg2(1) = -kg * (xx2(1) - xx02(1)) -    bg*dxx2(1);
    Fg2(2) = -kg * (xx2(2) - yg ) -min(bg*dxx2(2),0); 
end

Fg3 = [0;0];
if xx3(2) < yg
    Fg3(1) = -kg * (xx3(1) - xx03(1)) -    bg*dxx3(1);
    Fg3(2) = -kg * (xx3(2) - yg ) -min(bg*dxx3(2),0);     
end
Fgi(1:2)=Fg2;
Fgi(3:4)=Fg3;

%--- limit force -------------------------------------------------------------------------------
Flim1 = 0;
if dt_l1 > 0 && l1 > llim1
    Flim1 = klim1*( l1 - llim1 ) - blim1*dt_l1;
end

Flim2 = 0;
if dt_l2 > 0 && l2 > llim2
    Flim2 = klim2*( l2 - llim2 ) - blim2*dt_l2;
end

Flim3 = 0;
if dt_l3 > 0 && l3 > llim3
    Flim3 = klim3*( l3 - llim3 ) - blim3*dt_l3;
end

% Actuator force -----------------------------------------------------------------------------------
% Mobility 
% u0がx方向の目標速度とする
% Fは伸展方向が正
deltaL = 0.01;
vd = [vdx; 0] ; % 目標速度 y: desiredHeight-xx1(2)はつぶれてしまうため、あまり良くならない

if 1 % 1: Mobility control
    %copute effective direction of 3 muscles, ex1, ex2, ex3
    [ex1_pr,ex2,ex3] = calc_ex_1(xi); % ex1も脚間に沿って計算
    [ex1,ex2,ex3] = calc_ex_2(xi,Fgi,deltaL); % ex1は脚長不変を仮定して三角形の角度から計算
    ex1_pr =real(ex1_pr);ex1 =real(ex1);ex2 =real(ex2);ex3 =real(ex3); % なぜか実行できない時の対処法（完全ではない）
    % right leg, muscle 2
    % vdl(:,2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(2) ~= 0 % 右脚が着地
        vdl(:,2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    else vdl(:,2) = dot(ex2,vd/2)*ex2 ; % 浮いているので寄与は半分
    end
    vi(:,2) =  dt_l2*ex2;
    vdr(:,2) = vd - vdl(:,2) ; % この関節が生成できない速度ベクトル
    vdc(:,2,3) = dot(ex3,vdr(:,2))*ex3 ; % 左が右に寄与できる配分
    vdc(:,2,1) = dot(ex1,vdr(:,2))*ex1 ; % 下が右に寄与できる配分
    km(2) = exp(-4*log(2)*(norm(vdl(:,2)-vi(:,2))^2+eps1)/(norm(vdl(:,2))^2+eps2)) ; % 1を超えないように設計されている
    % figure(1); plot(0:0.01:1,exp(-4*log(2).*[0:0.01:1])) % check
    
    % left leg, muscle 3
    % vdl(3,1:2) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(4) ~= 0 % 左脚が着地
        vdl(:,3) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    else vdl(:,3) = dot(ex3,vd/2)*ex3 ; % 浮いているので寄与は半分
    end
    vi(:,3) =  dt_l3*ex3;
    vdr(:,3) = vd - vdl(:,3) ; % この関節が生成できない速度ベクトル
    vdc(:,3,2) = dot(ex2,vdr(:,3))*ex2 ; % 右が左に寄与できる配分
    vdc(:,3,1) = dot(ex1,vdr(:,3))*ex1 ; % 下が左に寄与できる配分
    km(3) = exp(-4*log(2)*(norm(vdl(:,3)-vi(:,3))^2+eps1)/(norm(vdl(:,3))^2+eps2)) ; %
    
    % Hip, muscle 1
    vi(:,1) = dt_l1*ex1; % 脚間に沿った現在速度
    if Fgi(2) == 0 || Fgi(4) == 0% どちらかの脚が離地
        vdl(:,1) = dot(ex1,vd)*ex1 ; %
    else % 両脚支持期
        vdl(:,1) = [0 0] ; %
    end
    vdr(:,1) = vd - vdl(:,1) ; % この関節が生成できない速度ベクトル
    vdc(:,1,2) = dot(ex2,vdr(:,1))*ex2 ; % 右が下に寄与できる配分
    vdc(:,1,3) = dot(ex3,vdr(:,1))*ex3 ; % 左が下に寄与できる配分
    km(1) = exp(-4*log(2)*(norm(vdl(:,1)-vi(:,1))^2+eps1)/(norm(vdl(1,1:2))^2+eps2)) ; %
    
    %km = km*0;
    % 全体の相互作用
    vd_childa(:,1) = (1-km(2))*(1-km(3))*vdl(:,1) + km(2)*vdc(:,2,1) + km(3)*vdc(:,3,1) ; % 下に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(:,2) = (1-km(1))*(1-km(3))*vdl(:,2) + km(1)*vdc(:,1,2) + km(3)*vdc(:,3,2) ; % 右に対するVdの配分 この時点でスカラーにするべき？
    vd_childa(:,3) = (1-km(1))*(1-km(2))*vdl(:,3) + km(1)*vdc(:,1,3) + km(2)*vdc(:,2,3) ;

    % （前のめりにならないための）倒れやすさ制御------------------------------------------------
    if 1
    % 1が倒れやすい、0が倒れにくい
    g = -gg(2) ;
    % right leg
    tmp = abs(dt_x1-dt_x2 + sqrt(g/l2)*(x1-x2))/sqrt(2); % 状態点と安定多様体との距離　Kagawa & Uno 2010 y = -x に対して、abs(y0-1*x0+0)/sqrt(1+1^2)
    kf(2) = exp(-1/abs(tmp)) ; % 0.5に相当する値は何か？ vdl,vdcは単位ベクトルなので一緒でいい？（目標が倒れないなので設定しにくい）
    % left leg
    tmp = abs(dt_x1-dt_x3 + sqrt(g/l2)*(x1-x3))/sqrt(2); % 原点と安定多様体との距離　Kagawa & Uno 2010
    kf(3) = exp(-1/abs(tmp)) ;
    % Hip: 自身の倒れやすさはゼロ
    kf(1) = 0 ;
    % 二脚を助ける成分：接地離地で分類する必要あり
    if Fgi(2) == 0 || Fgi(4) == 0% どちらかの脚が離地
        kf2vdc1 = kf(2)*vdc(:,2,1) ;
        kf3vdc1 = kf(3)*vdc(:,3,1) ; % x2>x3でも逆でも、kf(3)>0でも逆でも
    else % 両脚支持期
        kf2vdc1 = 0 ; kf3vdc1 = 0 ;
    end
    % 全体の相互作用
    vdf_childa(:,1) = kf2vdc1 + kf3vdc1 ; % 自分は倒れることに直接関係しないので、助けるのみ
%     amp = 5 ; % 脚を上げるために非対称性を強くする→いまはなぜか非対称性が強く前脚を上げてくれるので、陽には入れない
%     if abs(kf(2)-kf(3)) < 0.3
%         if sign(vdx)*sign(x2-x3) == 1 % 前脚が、2の脚
%             vdf_childa(:,2) =   amp*(1-kf(3))*vdl(:,2) + 1/amp*kf(3)*vdc(:,3,2) ; %
%             vdf_childa(:,3) = 1/amp*(1-kf(2))*vdl(:,3) +   amp*kf(2)*vdc(:,2,3) ; %
%         else
%             vdf_childa(:,2) = 1/amp*(1-kf(3))*vdl(:,2) +   amp*kf(3)*vdc(:,3,2) ; %
%             vdf_childa(:,3) =   amp*(1-kf(2))*vdl(:,3) + 1/amp*kf(2)*vdc(:,2,3) ; %
%         end
%     else
        vdf_childa(:,2) = (1-kf(3))*vdl(:,2) + kf(3)*vdc(:,3,2) ; % エラー？：代入の右辺の次元が、大きさが 1 でない次元の添字より多くなっています
        vdf_childa(:,3) = (1-kf(2))*vdl(:,3) + kf(2)*vdc(:,2,3) ; % デバッグできない
%     end
%     else kf = zeros(1,3) ; vdf_childa = zeros(2,3) ;
    end
    % 出力へ変換 
    dt_ld(1) = -dot(vd_childa(:,1)+ckf*vdf_childa(:,1),ex1_pr) ; % なぜ負？
    dt_ld(2) = dot(vd_childa(:,2)+ckf*vdf_childa(:,2),ex2) ; 
    dt_ld(3) = dot(vd_childa(:,3)+ckf*vdf_childa(:,3),ex3) ;
    
    dt_l(1)=dt_l1;
    dt_l(2)=dt_l2;
    dt_l(3)=dt_l3;
    for i = 1:3
        Fai(i) = -Gi(i)*(dt_ld(i)-dt_l(i)); % なぜ負？
    end
    
    Fa1 = Fai(1)*ex1_pr;
    Fa2 = Fai(2)*ex2;
    Fa3 = Fai(3)*ex3;
end

% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
% e2=(xx1-xx2)/norm(xx1-xx2); % ex2と同じ
% e3=(xx1-xx3)/norm(xx1-xx3);    

% バネ定数を接地離地・短縮進展で変更する
if Fgi(2) ~= 0 % 右脚が着地
    k_leg2 = k_leg22 ;
else 
    k_leg2 = k_leg21 ;
end

if Fgi(3) ~= 0 % 右脚が着地
    k_leg3 = k_leg32 ;
else 
    k_leg3 = k_leg31 ;
end

Fs2 = k_leg2 * ( (xx1-xx2) -ex2*l02);
Fs3 = k_leg3 * ( (xx1-xx3) -ex3*l03);
Fb2 = -b_leg2 * ( dt_l2 * ex2 );
Fb3 = -b_leg3 * ( dt_l3 * ex3 );
Flim1 = Flim1*ex1_pr ; % 追加
Flim2 = Flim2*ex2 ; % 追加 
Flim3 = Flim3*ex3 ; % 追加
dy = zeros(1,12);
dy(1:6)  = dt_xi;
dy( 7: 8)= gg + (-Fs2-Fs3+Fb2+Fb3     -Fa2-Fa3        -Flim2-Flim3)/m1;
dy( 9:10)= gg + ( Fs2    -Fb2    +Fg2 +Fa2-Fa1 -Flim1 +Flim2      )/m2;
dy(11:12)= gg + (     Fs3    -Fb3+Fg3 +Fa3+Fa1 +Flim1       +Flim3)/m3;

%save results 
result.vd = vd;
result.vd_childa = vd_childa;
result.vdl = vdl;
result.vdr = vdr;
result.vdc = vdc;
result.ex1 = ex1;
result.ex2 = ex2;
result.ex3 = ex3;
result.dt_l1 = dt_l1;
result.dt_l2 = dt_l2;
result.dt_l3 = dt_l3;
result.dt_ld = dt_ld;
result.dt_l = dt_l;
result.l1 = l1;
result.l2 = l2;
result.l3 = l3;
result.km = km;
result.kf = kf;
result.xx1 = xx1;
result.xx2 = xx2;
result.xx3 = xx3;
result.Fai = Fai;
result.Fs2 = Fs2;
result.Fs3 = Fs3;
result.Fb2 = Fb2;
result.Fb3 = Fb3;
result.Fa1 = Fa1;
result.Fa2 = Fa2;
result.Fa3 = Fa3;
result.Fg2 = Fg2;
result.Fg3 = Fg3;
result.Flim1 = Flim1;
result.Flim2 = Flim2;
result.Flim3 = Flim3;
result.param = param;


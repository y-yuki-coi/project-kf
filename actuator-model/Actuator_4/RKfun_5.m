function [dy result] = RKfun_5(time,y0,param,isRefresh) 
% 20160702作成
% 吉原さんの修正から、自分で解読して再修正 

% for debug 
% y0 = Y(t,:); isRefresh = 0 ; 

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
desiredHeightGain=param.desiredHeightGain;
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
persistent xx02; % デバッグは直前の行をプリントアウトして行う
persistent xx03;

if xx2(2) < yg %contact
    if isempty(xx02) && isRefresh
        xx02 = xx2 ;
    else
    end
else
    xx02 = [];
end

if xx3(2) < yg %contact
    if isempty(xx03) && isRefresh
        xx03 = xx3;
    else
    end
else
    xx03 = [];
end

Fg2 = [0;0];
if ~isempty(xx02)
    Fg2(1) = -kg * (xx2(1) - xx02(1)) -    bg*dxx2(1);
    Fg2(2) = -kg * (xx2(2) - yg ) -min(bg*dxx2(2),0); 
end

Fg3 = [0;0];
if ~isempty(xx03)
    Fg3(1) = -kg * (xx3(1) - xx03(1)) -    bg*dxx3(1);
    Fg3(2) = -kg * (xx3(2) - yg ) -min(bg*dxx3(2),0);     
end
Fgi(1:2)=Fg2;
Fgi(3:4)=Fg3;

%--- limit force -------------------------------------------------------------------------------
% 脚が伸びきらない力
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

% バランスを崩さない力
Flimb = 0;
klim4 = 10000 ;
lb = 0.1 ;
if abs(x2-x3) > 0.2
    if x2 > x3
        if abs(x2-x1) < lb
            Flimb = klim4*( x2-x1 - lb );% - blim4*(dt_x2-dt_x1);
        elseif abs(x1-x3) < lb
            Flimb = - klim4*( x1-x3 - lb );
        end
    else
        if abs(x3-x1) < lb
            Flimb = klim4*( x3-x1 - lb );% - blim4*(dt_x3-dt_x1);
        elseif abs(x1-x2) < lb
            Flimb = - klim4*( x1-x2 - lb );
        end
    end
end

% 跳びすぎない力
Flimj = 0;
klim5 = 1000 ;
blim5 = 1000;
lj = 1.5;
if y1 < lj && dt_y1 > 0
    Flimj = - klim5*( y1 - lj ) - blim5*(dt_y1);
end



% Actuator force -----------------------------------------------------------------------------------
% Mobility 
% u0がx方向の目標速度とする
% Fは伸展方向が正（収縮方向が正の時もあるのはなぜ？）
deltaL = 0.01;
vd = [vdx; 0] ; % 目標速度 y: desiredHeightGain*(desiredHeight-xx1(2))はつぶれてしまうため、あまり良くならない
g = -gg(2) ;

if 1 % 1: Mobility control
    %copute effective direction of 3 muscles, ex1, ex2, ex3
    [ex1, ex2, ex3, em1, em2, em3] = calc_ex_1(xi,Fg2,Fg3); % ex1も脚間に沿って計算
    
    % right leg, muscle 2
    % vdl(:,2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(2) ~= 0 % 右脚が着地
        vdl(:,2) = dot(ex2,vd)*ex2 ; % 局所速度ベクトル（本来寄与できるベクトル）
        vdr(:,2) = vd - vdl(:,2) ; % この関節が生成できない速度ベクトル
    else 
        if (vdx >= 0 && x2 >= x3) || (vdx < 0 && x2 < x3) % 前脚の時
            vdl(:,2) = dot(ex2,vd/2)*ex2 ; % 浮いているので寄与は半分
            vdr(:,2) = vd - vdl(:,2) ; % 
            if abs(x2-x1) < 0.2 % 前脚のバランスに関する修正
                vdl(:,2) = -vdl(:,2) ;% ;%*2
                vdr(:,2) = -vd + vdl(:,2) ; % この関節が生成できない速度ベクトル
            end
        else % 後脚の時
            if l2 < 1
                vdl(:,2) = dot(ex2,vd/2)*ex2 ;
                vdr(:,2) = vd - vdl(:,2) ; % 
            else
                vdl(:,2) = -dot(ex2,vd)*ex2 ; %
                vdr(:,2) = - vd + vdl(:,2) ; % 
            end
        end
    end
   
    vi(:,2) =  dt_l2*ex2;
    vdc(:,2,3) = dot(ex3,vdr(:,2))*ex3 ; % 左が右に寄与できる配分
    vdc(:,2,1) = dot(ex1,vdr(:,2))*ex1 ; % 下が右に寄与できる配分
    
    % left leg, muscle 3
    % vdl(3,1:2) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
    if Fgi(4) ~= 0 % 左脚が着地
        vdl(:,3) = dot(ex3,vd)*ex3 ; % 局所速度ベクトル（本来寄与できるベクトル）
        vdr(:,3) = vd - vdl(:,3) ; % この関節が生成できない速度ベクトル
    else
        if (vdx >= 0 && x2 <= x3) || (vdx < 0 && x2 > x3) % 前脚の時
            vdl(:,3) = dot(ex3,vd/2)*ex3 ; % 浮いているので寄与は半分
            vdr(:,3) = vd - vdl(:,3) ; % この関節が生成できない速度ベクトル
            if abs(x3-x1) < 0.2 % 前脚のバランスに関する修正
                vdl(:,3) = -vdl(:,3) ;% ;%*2
                vdr(:,3) = -vd + vdl(:,3) ; % この関節が生成できない速度ベクトル
            end
        else
            if l3 < 1
                vdl(:,3) = dot(ex3,vd/2)*ex3 ;
                vdr(:,3) = vd - vdl(:,3) ; % この関節が生成できない速度ベクトル
            else
                vdl(:,3) = -dot(ex3,vd)*ex3 ; % 浮いているので寄与は半分
                vdr(:,3) = - vd + vdl(:,3) ; % この関節が生成できない速度ベクトル
            end
        end
    end
    
    vi(:,3) =  dt_l3*ex3;
    vdc(:,3,2) = dot(ex2,vdr(:,3))*ex2 ; % 右が左に寄与できる配分
    vdc(:,3,1) = dot(ex1,vdr(:,3))*ex1 ; % 下が左に寄与できる配分
    
    
    % Hip, muscle 1
    vi(:,1) = dt_l1*ex1; % 脚間に沿った現在速度
    eps4 = 0.03 ; 
    if y2 > yg+eps4 && y3 > yg+eps4 % 空中期 Fgi(2) == 0 && Fgi(4) == 0
        if l3 > 0.5
            vdl(:,1) = -dot(ex1,vd)*ex1 ; %
            vdr(:,1) = -vd + vdl(:,1) ; % この関節が生成できない速度ベクトル
        else
            vdl(:,1) = dot(ex1,vd)*ex1 ; %
            vdr(:,1) = vd - vdl(:,1) ; % この関節が生成できない速度ベクトル
        end
    elseif y2 <= yg+eps4 && y3 > yg+eps4 % 右脚支持期
        vdl(:,1) = -dot(ex1,vd)*ex1 ; %
        vdr(:,1) = -vd + vdl(:,1) ; % この関節が生成できない速度ベクトル
    elseif y2 > yg+eps4 && y3 <= yg+eps4% 左脚支持期    
        vdl(:,1) = dot(ex1,vd)*ex1 ; %
        vdr(:,1) = vd - vdl(:,1) ; % この関節が生成できない速度ベクトル        
    else % 両脚支持期
        vdl(:,1) = [0 0];
%         vdl(:,1) = dot(ex1,vd)*ex1 ; %
        vdr(:,1) = vd - vdl(:,1) ; % この関節が生成できない速度ベクトル     
    end
    
    vdc(:,1,2) = dot(ex2,vdr(:,1))*ex2 ; % 右が下に寄与できる配分
    vdc(:,1,3) = dot(ex3,vdr(:,1))*ex3 ; % 左が下に寄与できる配分
    
    % 動きやすさ
    km(1) = calcMobilityByLengthVelocity(l1,l01,dt_l1,vdx) ; 
    km(2) = calcMobilityByLengthVelocity(l2,l02,dt_l2,vdx) ; 
    km(3) = calcMobilityByLengthVelocity(l3,l03,dt_l3,vdx) ; 

    if 0
        km(2) = exp(-4*log(2)*(norm(vdl(:,2)-vi(:,2))^2+eps1)/(norm(vdl(:,2))^2+eps2)) ; % 1を超えないように設計されている       
        km(3) = exp(-4*log(2)*(norm(vdl(:,3)-vi(:,3))^2+eps1)/(norm(vdl(:,3))^2+eps2)) ; %
        km(1) = exp(-4*log(2)*(norm(vdl(:,1)-vi(:,1))^2+eps1)/(norm(vdl(:,1))^2+eps2)) ; %
    end
    if (vdx >= 0 && x2 >= x3) || (vdx < 0 && x2 < x3) % 右が前脚の時
        if abs(x2-x1) < 0.2 % 前脚のバランスに関する修正
            km(2) = km(2)+0.2 ;
        end
    else 
        if abs(x3-x1) < 0.2 % 前脚のバランスに関する修正
            km(3) = km(3)+0.2 ;
        end
    end
    tmpk = sum(km); % 動きやすさの割合で再配分
    km = km/(tmpk+eps2);
    %km = km*0;
    
    % 全体の相互作用
    vd_childa(:,1) = (1-km(2))*(1-km(3))*vdl(:,1) + km(2)*vdc(:,2,1) + km(3)*vdc(:,3,1) ; % 下に対するVdの配分  
    vd_childa(:,2) = (1-km(1))*(1-km(3))*vdl(:,2) + km(1)*vdc(:,1,2) + km(3)*vdc(:,3,2) ; % 右に対するVdの配分 
    vd_childa(:,3) = (1-km(1))*(1-km(2))*vdl(:,3) + km(1)*vdc(:,1,3) + km(2)*vdc(:,2,3) ;

    % 出力へ変換 
    dt_ld(1) = dot(vd_childa(:,1),em1) ;
    dt_ld(2) = dot(vd_childa(:,2),em2) ; 
    dt_ld(3) = dot(vd_childa(:,3),em3) ;
    
    Fa1 = -G1*(dt_ld(1)-dt_l1)*em1; % dt_ld<0, dt_l1>0, em>1, Fa1>0
    Fa2 = -G2*(dt_ld(2)-dt_l2)*em2;
    Fa3 = -G3*(dt_ld(3)-dt_l3)*em3;
    
    Fai(2) = -G2*(dt_ld(2)-dt_l2) ;
    Fai(1) = -G1*(dt_ld(1)-dt_l1) ;
    Fai(3) = -G3*(dt_ld(3)-dt_l3) ;

end
% Fa1 = zeros(2,1); %Fa2 = zeros(2,1); Fa3 = zeros(2,1); 
% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
% e2=(xx1-xx2)/norm(xx1-xx2); 
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

Fs2 = k_leg2 * ( (xx1-xx2) -em2*l02);
Fs3 = k_leg3 * ( (xx1-xx3) -em3*l03);
Fb2 = -b_leg2 * ( dt_l2 * em2 );
Fb3 = -b_leg3 * ( dt_l3 * em3 );
Flim1 = Flim1*em1 ; % 追加
Flim2 = Flim2*em2 ; % 追加 
Flim3 = Flim3*em3 ; % 追加
dy = zeros(1,12);
dy(1:6)  = dt_xi;
dy( 7: 8)= gg + (-Fs2-Fs3+Fb2+Fb3     -Fa2-Fa3 +Flimb +Flimj -Flim2-Flim3)/m1; % Flimb Flimjを追加
dy( 9:10)= gg + ( Fs2    -Fb2    +Fg2 +Fa2-Fa1 -Flim1        +Flim2      )/m2; % +Flim1では？
dy(11:12)= gg + (     Fs3    -Fb3+Fg3 +Fa3+Fa1 +Flim1              +Flim3)/m3; % -Flim1では？

if isnan(dy(9))
%     vi
end
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
% result.dt_l = dt_l;
result.l1 = l1;
result.l2 = l2;
result.l3 = l3;
result.km = km;
result.xx1 = xx1;
result.xx2 = xx2;
result.xx3 = xx3;
result.xx02 = xx02;
result.xx03 = xx03;
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
result.Flimb = Flimb;
result.param = param;


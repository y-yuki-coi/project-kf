function [dy result] = RKfun_3(time,y0,param,isRefresh)
% 20160628 review

%rename params
k_leg2=param.k_leg2;
k_leg3=param.k_leg3;
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
l_vec = [l1 l2 l3 dt_l1 dt_l2 dt_l3] ;

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

persistent isFirstContact2;
persistent isFirstContact3;
persistent xx02;
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
% vdxがx方向の目標速度とする
% Fは伸展方向が正

vd = [vdx; desiredHeight-xx1(2)] ; % 目標速度
if 1 % 1: Mobility control
     %copute effective direction of 3 muscles, ex1, ex2, ex3
    ex1=(xx2-xx3)/norm(xx2-xx3);
    ex2=(xx1-xx2)/norm(xx1-xx2);
    ex3=(xx1-xx3)/norm(xx1-xx3);    
    if x2 > x3
        xxfore = xx2;
        xxhind = xx3;
    else
        xxfore = xx3;
        xxhind = xx2;
    end
    ex1 = (xxfore-xxhind)/norm(xxfore-xxhind);        
    %if xx2(2) < yg && xx3(2) < yg
    %ex1 = ex1 * 0;
    %end
    
    % right leg, muscle 2
    vdl(:,2) = dot(ex2,vd)*ex2;
    vi(:,2) =  dt_l2*ex2;
    vdr(2,:) = vd - vdl(:,2) ; % この関節が生成できない速度ベクトル
    vdc(:,2,3) = dot(ex3,vdr(:,2))*ex3 ; % 左が右に寄与できる配分
    vdc(:,2,1) = dot(ex1,vdr(:,2))*ex1 ; % 下が右に寄与できる配分
    km(2) = exp(-4*log(2)*(norm(vdl(:,2)-vi(:,2))^2+eps1)/(norm(vdl(:,2))^2+eps2)) ; % 1を超えないように設計されている
    
    % left leg, muscle 3
    vdl(:,3) = dot(ex3,vd)*ex3; % 局所速度ベクトル（本来寄与できるベクトル）
    vi(:,3) =  dt_l3*ex3;
    vdr(:,3) = vd - vdl(:,3); % この関節が生成できない速度ベクトル
    vdc(:,3,2) = dot(ex2,vdr(:,3))*ex2 ; % 右が左に寄与できる配分
    vdc(:,3,1) = dot(ex1,vdr(:,3))*ex1 ; % 下が左に寄与できる配分
    km(3) = exp(-4*log(2)*(norm(vdl(:,3)-vi(:,3))^2+eps1)/(norm(vdl(:,3))^2+eps2)); %
    
    % Hip, muscle 1
    vdl(:,1) = dot(ex1,vd)*ex1; % 局所速度ベクトル（本来寄与できるベクトル）    
    vi(:,1) =  dt_l1*ex1;    
    vdr(:,1) = vd - vdl(:,1); % この関節が生成できない速度ベクトル
    vdc(:,1,2) = dot(ex2,vdr(:,1))*ex2 ; 
    vdc(:,1,3) = dot(ex3,vdr(:,1))*ex3 ; 
    km(1) = exp(-4*log(2)*(norm(vdl(:,1)-vi(:,1))^2+eps1)/(norm(vdl(:,1))^2+eps2)); %    
    
    %km = km*0;
    % 全体の相互作用
    vd_childa(:,1) = (1-km(2))*(1-km(3))*vdl(:,1) + km(2)*vdc(:,2,1) + km(3)*vdc(:,3,1) ; % 下に対するVdの配分
    vd_childa(:,2) = (1-km(1))*(1-km(3))*vdl(:,2) + km(1)*vdc(:,1,2) + km(3)*vdc(:,3,2) ; % 右に対するVdの配分
    vd_childa(:,3) = (1-km(1))*(1-km(2))*vdl(:,3) + km(1)*vdc(:,1,3) + km(2)*vdc(:,2,3) ;
    
    % 出力へ変換
    dt_ld(1) = -dot(vd_childa(:,1),ex1) ; % 出力方向に変換
    dt_ld(2) = dot(vd_childa(:,2),ex2) ; 
    dt_ld(3) = dot(vd_childa(:,3),ex3) ;
    
    dt_l(1)=dt_l1;
    dt_l(2)=dt_l2;
    dt_l(3)=dt_l3;
    for i = 1:3
        Fai(i) = -Gi(i)*(dt_ld(i)-dt_l(i));
    end
        
    Fa1 = Fai(1)*ex1;
    Fa2 = Fai(2)*ex2;
    Fa3 = Fai(3)*ex3;    
end

% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

e2=(xx1-xx2)/norm(xx1-xx2);
e3=(xx1-xx3)/norm(xx1-xx3);    

Fs2 = k_leg2 * ( (xx1-xx2) -e2*l02);
Fs3 = k_leg3 * ( (xx1-xx3) -e3*l03);
Fb2 = -b_leg2 * ( dt_l2 * e2 );
Fb3 = -b_leg3 * ( dt_l3 * e3 );

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
result.xx1 = xx1;
result.xx2 = xx2;
result.xx3 = xx3;
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





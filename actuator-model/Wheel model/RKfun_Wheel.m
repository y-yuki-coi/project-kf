function [dy Fgi dt2_xi] = RKfun_Wheel(y0,u,isRefresh)

global g mi li Gi mu mu0  hw hwb hwt ws

xi = y0(1:6) ; dt_xi = y0(7:12) ; 

% for debug 
% t =30; % first contact h = 10^(-3) 
% t = 4296;% t = 4299 % too large GRF  h = 10^(-4) 
% t = 5246;% t = 4299 % too large GRF  h = 10^(-4) 
% xi = Y(t,1:14) ;dt_xi = Y(t,15:28) ; Y0 = Y(1:t,:); 

m1 = mi(1); m2 = mi(2); 
G1 = Gi(1); G2 = Gi(2); l1 = li(1);
x1 = xi(1) ; x2 = xi(2) ; th1 = xi(3) ; th2 = xi(4) ; th3 = xi(5) ; th4 = xi(6) ; 
dt_x1 = dt_xi(1) ; dt_x2 = dt_xi(2) ; dt_th1 = dt_xi(3) ; dt_th2 = dt_xi(4) ; dt_th3 = dt_xi(5) ; dt_th4 = dt_xi(6) ;

vd = u ; % 目標速度
% calculate ground reaction force (GRF)-----------------------------------------------------------------
% wheel position 
xr = x2+(l1/2); 
xl = x2-(l1/2); 

% Ground reaction force
Fgi = zeros(4,1) ; 

lr = abs(l1/2 + x2 - x1) ; % てこの長さ（右側）
ll = abs(l1/2 - x2 + x1) ; % てこの長さ（左側）
mr = ll/l1*m1 + m2 ; 
ml = lr/l1*m1 + m2 ; 
mmax = m1 + m2 ;

Fzr = mr * g ; % Fz 
Fzl = ml * g ; % Fz 



if 0% dt_x1 == 0 % static
    % 振動子制御
    
    
    
    
else  % dynamic
    F1 = G1*sin(th1); 
    if mod(th1,2*pi) < pi % right leg contact
        %Fgi(1) = -th3*mu*Fzr ;
        Fgi(1) = -sign(vd)*sin(th1)*th3*Fzr/mmax*G2*(vd-dt_x2)/2 ;
    else Fgi(1) = 0 ; % non-contact
    end
    if mod(th2,2*pi) < pi % left leg contact
        % Fgi(3) = -th4*mu*Fzl ;
        Fgi(3) = -sign(vd)*sin(th2)*th4*Fzl/mmax*G2*(vd-dt_x2)/2 ;
    else Fgi(3) = 0 ; % non-contact
    end
end
Fxr = Fgi(1) ; Fgi(2) = Fzr ;
Fxl = Fgi(3) ; Fgi(4) = Fzl ;

% Mobility  -----------------------------------------------------------------------------------
% u0がx方向の目標速度とする
eps1 = 10^(-5) ; eps2 = 10^(-5) ; % 零の除算を避けるための微小量 

if 0 % 1: Mobility control
else % どのようにアクチュエータの入力を作るか未定の場合
    
end


% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
dt2_xi = zeros(1,6) ;

% differential equantion and output
dy = zeros(1,12) ;
dy(1)  = dt_xi(1) ;
% dy(1)  = dt_xi(1)+1*sin(dt_xi(2)/hw*2);%
dy(2)  = dt_xi(2) ;
dy(3)  = dt_xi(2)/hw ;
dy(4)  = dt_xi(2)/hw ;
if sign(vd) < 0 
    dy(5)  = -ws ;
    if th3 < -1 ; dy(5)  = 0 ;
    end
else dy(5)  = 0 ;
end
if sign(vd) < 0 
    dy(6)  = -ws ;
    if th4 < -1 ; dy(6)  = 0 ;
    end
else dy(6)  = 0 ;
end
dy(8)  = (- Fxr - Fxl)/m2 ; %  - F1/m1
% dy(7)  = (- Fxr - Fxl)/m2; % -mu*g  F1/m1 
dy(7)  = G1*sin(2*th3) +(- Fxr - Fxl)/m2; % -mu*g  F1/m1 
dy(9)  = 0 ;
dy(10) = 0 ;
dy(11) = 0 ;
dy(12) = 0 ;
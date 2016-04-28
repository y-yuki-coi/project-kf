% Appendix_Eq_Motion_Const_5Link.m
% Appendix of Equations of Motion & Kinematic constraint
% 直接実行しないが、式の確認用(Appendix)
% Symbolic Mathで数式の検算をしたかったが、equationsToMatrixは2011aには入っていない

clear ;
% Equations of Motion -------------------------------------------
% dt2_xi = P*F + Q 
% Taga1991との相違点 Taga1995の表記に合わせる（セグメント番号に合うように）
% x1 = x1; y1 = x2 ; % HAT 
% x2 = x3; y2 = x4 ; th2 = x5; x3 = x6; y3 = x7 ; th3 = x8; % LR Thigh
% x4 = x9; y4 = x10 ; th4 = x11; x5 = x12; y5 = x13 ; th5 = x14; % LR Shank 
% Fx2 = F1 ; Fy2 = F3 ; Fx3 = F4 ; Fy3 = F5 ; % LR Thigh
% Fx4 = F6 ; Fy4 = F7 ; Fx5 = F8 ; Fy5 = F9 ; % LR Shank
% Fgx1 = Fg1 ; Fgy1 = Fg2 ; Fgx2 = Fg3 ; Fgy2 = Fg4 ; % LR GRF
% l2 = l1 ; I2 = I1 ; % Thigh 
% l3 = l3 ; I3 = I3 ; % Shank 
if 0 % equationsToMatrixがあれば1
    syms m1 m2 m3 l2 l3 I2 I3 b1 b2 bk kk g ...
        dt2_x1 dt2_x2 dt2_x3 dt2_x4 dt2_x5 dt2_y1 dt2_y2 dt2_y3 dt2_y4 dt2_y5...
        dt2_th2 dt2_th3 dt2_th4 dt2_th5  ... 
        Fx2 Fx3 Fx4 Fx5 Fy2 Fy3 Fy4 Fy5 Fgx1 Fgx2 Fgy1 Fgy2 ...
        th2 th3 th4 th5 dt_th2 dt_th3 dt_th4 dt_th5 ...
        Tr1 Tr2 Tr3 Tr4 Tr5 Tr6 Tr7 Tr8
[P,Q] = equationsToMatrix([...
m1 * dt2_x1  ==   Fx2 + Fx3, .... % HAT x (1)
m1 * dt2_y1  ==   Fy2 + Fy3 -m1 * g, .... % HAT y (2)
m2 * dt2_x2  == - Fx2 + Fx4 - Fa1*cos(th2), .... % Thigh_R x (3) Actuator
m2 * dt2_y2  == - Fy2 + Fy4 -m2 * g + Fa1*sin(th2), .... % Thigh_R y (4)
I2 * dt2_th2 == - Fx2*(l2/2)*sin(th2) - Fy2*(l2/2)*cos(th2) ... % Thigh_R theta (5)
                - Fx4*(l2/2)*sin(th2) - Fy4*(l2/2)*cos(th2) ... % from force
                - b1*abs(th2-pi/2)*dt_th2 ... % viscosity 1 (hip)
                - (b2+bk*f_max(th2-th4))*(dt_th2-dt_th4) ...  % viscosity 2 (knee)
                - kk*h_sgn(th2-th4), ... % elasticity
m2 * dt2_x3  == - Fx3 + Fx5 - Fa2*cos(th3), .... % Thigh_L x (6)
m2 * dt2_y3  == - Fy3 + Fy5 -m2 * g + Fa2*sin(th3), .... % Thigh_L y (7)
I2 * dt2_th3 == - Fx3*(l2/2)*sin(th3) - Fy3*(l2/2)*cos(th3) ... % Thigh_L theta (8) 
                - Fx5*(l2/2)*sin(th3) - Fy5*(l2/2)*cos(th3) ...
                - b1*abs(th3-pi/2)*dt_th3 ...
                - (b2+bk*f_max(th3-th5))*(dt_th3-dt_th5) ...
                - kk*h_sgn(th3-th5), ...          
m3 * dt2_x4  == - Fx4 + Fgx1 + Fa1*cos(th2), .... % Shank_R x (9)
m3 * dt2_y4  == - Fy4 + Fgy1 -m3 * g - Fa1*sin(th2), .... % Shank_R y (10) 
I3 * dt2_th4 == - Fx4*(l3/2)*sin(th4) - Fy4*(l3/2)*cos(th4) ... % Shank_R theta (11)
                - Fgx1*(l3/2)*sin(th4) - Fgy1*(l3/2)*cos(th4) ... 
                - (b2+bk*f_max(th2-th4))*(dt_th4-dt_th2) ... 
                + kk*h_sgn(th2-th4), ... 
m3 * dt2_x5  == - Fx5 + Fgx2 + Fa2*cos(th3), .... % Shank_L x (12)
m3 * dt2_y5  == - Fy5 + Fgy2 -m3 * g - Fa2*sin(th3), .... % Shank_L y (13)
I3 * dt2_th5 == - Fx5*(l3/2)*sin(th5) - Fy5*(l3/2)*cos(th5) ... % Shank_L theta (14)
                - Fgx2*(l3/2)*sin(th5) - Fgy2*(l3/2)*cos(th5) ...
                - (b2+bk*f_max(th3-th5))*(dt_th5-dt_th3) ...
                + kk*h_sgn(th3-th5), ...
                ],[Fx2;Fy2;Fx3;Fy3;Fx4;Fy4;Fx5;Fy5]);
else 
end 

% 手計算でPとQを求める
P = [1/m1 0 1/m1 0 0 0 0 0 ; % 14*8 matrix 
     0 1/m1 0 1/m1 0 0 0 0 ;
     -1/m2 0 0 0 1/m2 0 0 0 ;
     0 -1/m2 0 0 0 1/m2 0 0 ;
     -(l2/2/I2)*sin(th2) -(l2/2/I2)*cos(th2) 0 0 -(l2/2/I2)*sin(th2) -(l2/2/I2)*cos(th2) 0 0;
     0 0 -1/m2 0 0 0 1/m2 0 ;
     0 0 0 -1/m2 0 0 0 1/m2 ;
     0 0 -(l2/2/I2)*sin(th3) -(l2/2/I2)*cos(th3) 0 0 -(l2/2/I2)*sin(th3) -(l2/2/I2)*cos(th3);
     0 0 0 0 -1/m3 0 0 0 ;
     0 0 0 0 0 -1/m3 0 0 ;
     0 0 0 0 -(l3/2/I3)*sin(th4) -(l3/2/I3)*cos(th4) 0 0;
     0 0 0 0 0 0 -1/m3 0 ;
     0 0 0 0 0 0 0 -1/m3 ;
     0 0 0 0 0 0 -(l3/2/I3)*sin(th5) -(l3/2/I3)*cos(th5)
     ] ;
Q = [0;-g; % (1-2) total:8*1 vector
     - Fa1*cos(th2)/m2; + Fa1*sin(th2)/m2 - g; % (3-4)
     (- b1*abs(th2-pi/2)*dt_th2 - (b2+bk*f_max(th2-th4))*(dt_th2-dt_th4) - kk*h_sgn(th2-th4) + Tr1 + Tr3)/I2 ; % (5)  
     - Fa2*cos(th3)/m2; + Fa2*sin(th3)/m2 - g; % (6-7)
     (- b1*abs(th3-pi/2)*dt_th3 - (b2+bk*f_max(th3-th5))*(dt_th3-dt_th5) - kk*h_sgn(th3-th5) + Tr2 + Tr4)/I2 ; % (8)
     (Fgx1 + Fa1*cos(th2))/m3; (Fgy1 - Fa1*sin(th2))/m3-g; % (9-10)
     (- (b2+bk*f_max(th2-th4))*(dt_th4-dt_th2) + kk*h_sgn(th2-th4) - Tr3 - Tr5)/I3 ; % (11)
     (Fgx2 + Fa2*cos(th3))/m3; (Fgy2 - Fa2*sin(th3))/m3-g; % (12-13)
     (- (b2+bk*f_max(th3-th5))*(dt_th5-dt_th3) + kk*h_sgn(th3-th5) - Tr4 - Tr6)/I3 % (14)
     ] ;

% Kinematic constraint----------------------------------------------
% D = C*dt2_xi
if 0 % equationsToMatrixがあれば1
    syms
    D = equationsToMatrix([...
        dt2_x1 - dt2_x2 - (l2/2)*sin(th2)*dt2_th2 ==  (l2/2)*cos(th2)*dt_th2^2, ... % HAT & Thigh
        dt2_y1 - dt2_y2 - (l2/2)*cos(th2)*dt2_th2 == -(l2/2)*sin(th2)*dt_th2^2, ...
        dt2_x1 - dt2_x3 - (l2/2)*sin(th3)*dt2_th3 ==  (l2/2)*cos(th3)*dt_th3^2, ...
        dt2_y1 - dt2_y3 - (l2/2)*cos(th3)*dt2_th3 == -(l2/2)*sin(th3)*dt_th3^2, ...
        dt2_x2 - (l2/2)*sin(th2)*dt2_th2 - dt2_x4 - (l3/2)*sin(th4)*dt2_th4 == ... % Thigh & Shank
               + (l2/2)*cos(th2)*dt_th2^2 + (l3/2)*cos(th4)*dt_th4^2, ... 
        dt2_y2 - (l2/2)*cos(th2)*dt2_th2 - dt2_y4 - (l3/2)*cos(th4)*dt2_th4 == ...
               - (l2/2)*sin(th2)*dt_th2^2 - (l3/2)*sin(th4)*dt_th4^2, ...
        dt2_x3 - (l2/2)*sin(th3)*dt2_th3 - dt2_x5 - (l3/2)*sin(th5)*dt2_th5 == ...
               + (l2/2)*cos(th3)*dt_th3^2 + (l3/2)*cos(th5)*dt_th5^2, ...
        dt2_y3 - (l2/2)*cos(th3)*dt2_th3 - dt2_y5 - (l3/2)*cos(th5)*dt2_th5 == ...
               - (l2/2)*sin(th3)*dt_th3^2 - (l3/2)*sin(th5)*dt_th5^2, ...
        ],[dt2_x1;dt2_y1;dt2_x2;dt2_y2;dt2_th2;dt2_x3;dt2_y3;dt2_th3;dt2_x4;dt2_y4;dt2_th4;dt2_x5;dt2_y5;dt2_th5]);
end
% 手計算でCとDを求める
C = [1 0 -1 0 -(l2/2)*sin(th2) 0 0 0 0 0 0 0 0 0 ; % 8*14 matrix 
     0 1 0 -1 -(l2/2)*cos(th2) 0 0 0 0 0 0 0 0 0 ; 
     1 0 0 0 0 -1 0 -(l2/2)*sin(th3) 0 0 0 0 0 0 ; 
     0 1 0 0 0 0 -1 -(l2/2)*cos(th3) 0 0 0 0 0 0 ;     
     0 0 1 0 -(l2/2)*sin(th2) 0 0 0 -1 0 -(l3/2)*sin(th4) 0 0 0; 
     0 0 0 1 -(l2/2)*cos(th2) 0 0 0 0 -1 -(l3/2)*cos(th4) 0 0 0; 
     0 0 0 0 0 1 0 -(l2/2)*sin(th3) 0 0 0 -1 0 -(l3/2)*sin(th5) ; 
     0 0 0 0 0 0 1 -(l2/2)*cos(th3) 0 0 0 0 -1 -(l3/2)*cos(th5) 
     ] ;
D = [ (l2/2)*cos(th2)*dt_th2^2; -(l2/2)*sin(th2)*dt_th2^2; % 8*1 vector
      (l2/2)*cos(th3)*dt_th3^2; -(l2/2)*sin(th3)*dt_th3^2; 
     +(l2/2)*cos(th2)*dt_th2^2 + (l3/2)*cos(th4)*dt_th4^2 ; 
     -(l2/2)*sin(th2)*dt_th2^2 - (l3/2)*sin(th4)*dt_th4^2;
     +(l2/2)*cos(th3)*dt_th3^2 + (l3/2)*cos(th5)*dt_th5^2 ; 
     -(l2/2)*sin(th3)*dt_th3^2 - (l3/2)*sin(th5)*dt_th5^2;
     ] ;
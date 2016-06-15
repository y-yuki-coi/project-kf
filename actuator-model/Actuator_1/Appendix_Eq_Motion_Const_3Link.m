% Appendix_Eq_Motion_Const_3Link.m
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
    %%%%%%%%%%%%%%%%%%%%% アクチュエーターの運動方程式？？%%%%%%%%%%%%%%%%%%%%
m1 * dt2_x1  ==   Fx2 + Fx3 - Fa2*cos(th2) - Fa3*cos(th3), .... % HAT 
m1 * dt2_y1  ==   Fy2 + Fy3 -m1 * g + Fa2*sin(th2) + Fa3*sin(th3), .... % 
m2 * dt2_x2  == - Fx2 + Fgx2 + Fa2*cos(th2), .... % Leg_R
m2 * dt2_y2  == - Fy2 + Fgy2 -m2 * g - Fa2*sin(th2), ....
I2 * dt2_th2 == - Fx2*(l2/2)*sin(th2) - Fy2*(l2/2)*cos(th2) ... 
                - Fgx2*(l2/2)*sin(th2) - Fgy2*(l2/2)*cos(th2) ... % from ground
                - b1*abs(th2-pi/2)*dt_th2 ... % viscosity 1 (hip) 果たして角度に依存するので良いのか？
m2 * dt2_x3  == - Fx3 + Fgx3 + Fa3*cos(th3), .... % Leg_L
m2 * dt2_y3  == - Fy3 + Fgy3 -m2 * g - Fa3*sin(th3), ....
I2 * dt2_th3 == - Fx3*(l2/2)*sin(th3) - Fy3*(l2/2)*cos(th3) ... 
                - Fgx3*(l2/2)*sin(th3) - Fgy3*(l2/2)*cos(th3) ...
                - b1*abs(th3-pi/2)*dt_th3 ...         
                ],[Fx2;Fy2;Fx3;Fy3]);
else 
end 

% 手計算でPとQを求める
P = [1/m1 0 1/m1 0 ; % total: 8*4 matrix 
     0 1/m1 0 1/m1 ;
     -1/m2 0 0 0 ;
     0 -1/m2 0 0 ;
     -(l2/2/I2)*sin(th2) -(l2/2/I2)*cos(th2) 0 0 ;
     0 0 -1/m2 0 ;
     0 0 0 -1/m2 ;
     0 0 -(l2/2/I2)*sin(th3) -(l2/2/I2)*cos(th3) ;
     ] ;
Q = [(- Fa2*cos(th2) - Fa3*cos(th3))/m1;-g; % (1-2) total:8*1 vector
     (Fgx2 + Fa2*cos(th2))/m2;(Fgy2- Fa2*sin(th2))/m2-g; % (3-4)
     (- b1*abs(th2-pi/2)*dt_th2)/I2 ; % (5)  
     (Fgx3 + Fa3*cos(th3))/m2;(Fgy3- Fa3*sin(th3))/m2-g; % (6-7)
     (- b1*abs(th3-pi/2)*dt_th3)/I2 ; % (8)
     ] ;

% Kinematic constraint----------------------------------------------
% D = C*dt2_xi
if 0 % equationsToMatrixがあれば1
    syms
    D = equationsToMatrix([...
        dt2_x1 - dt2_x2 - (l2/2)*sin(th2)*dt2_th2 ==  (l2/2)*cos(th2)*dt_th2^2, ... % HAT & Leg
        dt2_y1 - dt2_y2 - (l2/2)*cos(th2)*dt2_th2 == -(l2/2)*sin(th2)*dt_th2^2, ...
        dt2_x1 - dt2_x3 - (l2/2)*sin(th3)*dt2_th3 ==  (l2/2)*cos(th3)*dt_th3^2, ...
        dt2_y1 - dt2_y3 - (l2/2)*cos(th3)*dt2_th3 == -(l2/2)*sin(th3)*dt_th3^2, ...
        ],[dt2_x1;dt2_y1;dt2_x2;dt2_y2;dt2_th2;dt2_x3;dt2_y3;dt2_th3]);
end
% 手計算でCとDを求める
C = [1 0 -1 0 -(l2/2)*sin(th2) 0 0 0 ; % 4*8 matrix 
     0 1 0 -1 -(l2/2)*cos(th2) 0 0 0 ; 
     1 0 0 0 0 -1 0 -(l2/2)*sin(th3) ; 
     0 1 0 0 0 0 -1 -(l2/2)*cos(th3) ;     
     ] ;
D = [ (l2/2)*cos(th2)*dt_th2^2; -(l2/2)*sin(th2)*dt_th2^2; % 4*1 vector
      (l2/2)*cos(th3)*dt_th3^2; -(l2/2)*sin(th3)*dt_th3^2; 
     ] ;
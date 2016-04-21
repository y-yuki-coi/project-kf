function [dy Fgi dt2_xi] = RKfun(y0,isRefresh)

global g mi li Ii bi ki yg y_contact

xi = y0(1:8) ; dt_xi = y0(9:16) ; 

% for debug 
% t =30; % first contact h = 10^(-3) 
% t = 4296;% t = 4299 % too large GRF  h = 10^(-4) 
% t = 5246;% t = 4299 % too large GRF  h = 10^(-4) 
% xi = Y(t,1:14) ;dt_xi = Y(t,15:28) ; Y0 = Y(1:t,:); 

m1 = mi(1); m2 = mi(2); 
l2 = li(2); I2 = Ii(2); 
b1 = bi(1) ; b2 = bi(2) ; bk = bi(3) ; kk = ki(1) ; bg = bi(4) ; kg = ki(2) ;
x1 = xi(1) ; y1 = xi(2) ; x2 = xi(3) ; y2 = xi(4) ; th2 = xi(5) ; 
x3 = xi(6) ; y3 = xi(7) ; th3 = xi(8) ;
dt_x1 = dt_xi(1) ; dt_y1 = dt_xi(2) ; dt_x2 = dt_xi(3) ; dt_y2 = dt_xi(4) ; dt_th2 = dt_xi(5) ; 
dt_x3 = dt_xi(6) ; dt_y3 = dt_xi(7) ; dt_th3 = dt_xi(8) ;
% calculate ground reaction force (GRF)-----------------------------------------------------------------
% ankle position
xr = x2+(l2/2)*cos(th2) ;
yr = y2-(l2/2)*sin(th2) ; 
xl = x3+(l2/2)*cos(th3) ;
yl = y3-(l2/2)*sin(th3) ; 
dt_xr = dt_x2 - (l2/2)*sin(th2)*dt_th2 ;
dt_yr = dt_y2 - (l2/2)*cos(th2)*dt_th2 ;
dt_xl = dt_x3 - (l2/2)*sin(th3)*dt_th3 ;
dt_yl = dt_y3 - (l2/2)*cos(th3)*dt_th3 ;

% Ground reaction force
Fgi = zeros(4,1) ; 
if 1 % 1: GRF exist 0: no GRF
    if yr < yg % contact
        if y_contact(1,9) ~= 0 % not initial & keep contact
            yr0 = yg ; % Y0(t_contact,10)-(l3/2)*sin(Y0(t_contact,11)) ; % ground y
            xr0 = y_contact(1,3)+(l2/2)*cos(y_contact(1,5)) ; % ground x
            Fgi(1) = -kg*(xr-xr0) - bg*dt_xr ;
            Fgi(2) = -kg*(yr-yr0) + bg*f_max(-dt_yr) ;
        else % first contact in one cycle
            if isRefresh == 1 % Update
                y_contact(1,1:9) = [xi(1:8) 1] ;
            end
            Fgi(1:2) = 0 ;
        end
    else Fgi(1:2) = 0 ; % non-contact
        y_contact(1,1:15) = zeros(1,15) ;
    end
    
    if yl < yg % contact 
        if y_contact(2,9) ~= 0 % not initial & keep contact
            yl0 = yg ; % ground y
            xl0 = y_contact(2,6)+(l2/2)*cos(y_contact(2,8)) ; % ground x
            Fgi(3) = -kg*(xl-xl0) - bg*dt_xl ;
            Fgi(4) = -kg*(yl-yl0) + bg*f_max(-dt_yl) ;
        else % first contact in one cycle
            if isRefresh == 1 % Update 
                y_contact(2,1:9) = [xi(1:8) 1] ;
            end
            Fgi(3:4) = 0 ;
        end
    else Fgi(3:4) = 0 ; % non-contact
        y_contact(2,1:9) = zeros(1,9) ;
    end
    
end

% Actuator force -----------------------------------------------------------------------------------
Fai = zeros(2,1);% どのようにアクチュエータの入力を作るかは未定
Fa2 = Fai(1) ; Fa3 = Fai(2) ;

% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

% Equations of Motion 
% dt2_xi = P*F + Q 
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

% Kinematic constraint
% D = C*dt2_xi
C = [1 0 -1 0 -(l2/2)*sin(th2) 0 0 0 ; % 4*8 matrix 
     0 1 0 -1 -(l2/2)*cos(th2) 0 0 0 ; 
     1 0 0 0 0 -1 0 -(l2/2)*sin(th3) ; 
     0 1 0 0 0 0 -1 -(l2/2)*cos(th3) ;     
     ] ;
D = [ (l2/2)*cos(th2)*dt_th2^2; -(l2/2)*sin(th2)*dt_th2^2; % 4*1 vector
      (l2/2)*cos(th3)*dt_th3^2; -(l2/2)*sin(th3)*dt_th3^2; 
     ] ;
 
% Equation of motion with constraint
% dt2_xi = P*(inv(C*P)*(D-C*Q)) + Q ;
dt2_xi = P*((C*P)\(D-C*Q)) + Q ; % Gaussian elimination
% dt2_xi = P*(pinv(C*P)*(D-C*Q)) + Q ; % 擬似逆行列
% dt2_xi = zeros(1,14);

% differential equantion and output
dy = zeros(1,16) ;
dy(1:8)  = dt_xi ;
dy(9:16) = dt2_xi ;
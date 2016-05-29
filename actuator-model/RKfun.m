function [dy Fgi dt2_xi] = RKfun(y0,isRefresh)

global g mi li Ii bi ki yg y_contact u0 Gi

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
        else % first contact in one cycle
            if isRefresh == 1 % Update
                y_contact(1,1:9) = [xi(1:8) 1] ;
            end
            yr0 = yr ; xr0 = xr ;
        end
        Fgi(1) = -kg*(xr-xr0) - bg*dt_xr ;
        Fgi(2) = -kg*(yr-yr0) + bg*f_max(-dt_yr) ;
    else Fgi(1:2) = 0 ; % non-contact
        y_contact(1,1:15) = zeros(1,15) ;
    end
    
    if yl < yg % contact
        if y_contact(2,9) ~= 0 % not initial & keep contact
            yl0 = yg ; % ground y
            xl0 = y_contact(2,6)+(l2/2)*cos(y_contact(2,8)) ; % ground x
        else % first contact in one cycle
            if isRefresh == 1 % Update
                y_contact(2,1:9) = [xi(1:8) 1] ;
            end
            yl0 = yl ; xl0 = xl ;
        end
        Fgi(3) = -kg*(xl-xl0) - bg*dt_xl ;
        Fgi(4) = -kg*(yl-yl0) + bg*f_max(-dt_yl) ;
    else Fgi(3:4) = 0 ; % non-contact
        y_contact(2,1:9) = zeros(1,9) ;
    end
    
end

% Actuator force -----------------------------------------------------------------------------------
% Mobility 
% u0��x�����̖ڕW���x�Ƃ���
eps1 = 10^(-5) ; eps2 = 10^(-5) ; % ��̏��Z������邽�߂̔����� 
vd = [u0 0] ; % �ڕW���x
if 1 % 1: Mobility control
    ex1 = [-cos(th2) sin(th2)];
    ex2 = [-cos(th3) sin(th3)];
    % right leg
    vdl(1,1:2) = dot(ex1,vd)*ex1 ; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j
    vdr(1,1:2) = vd - vdl(1,1:2) ; % �{����^�ł��Ȃ����x�x�N�g��
    vdc(2,1:2) = dot(ex2,vdr(1,1:2))*ex2 ; % ���ɑ΂��ĉE����^�ł���z��
    if sum(Fgi(1:2)) == 0 % non-contact 
        k(1) = 0 ; 
    else % contact
        k(1) = exp(-4*log2(norm(vdl(1,1:2)-vd)+eps1)/(norm(vdl(1,1:2))+eps2)) ; % �����₷���w�W
    end
    % left leg
    vdl(2,1:2) = dot(ex2,vd)*ex2 ; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j
    vdr(2,1:2) = vd - vdl(2,1:2) ; % �{����^�ł��Ȃ����x�x�N�g��
    vdc(1,1:2) = dot(ex1,vdr(2,1:2))*ex1 ; % �E�ɑ΂��č�����^�ł���z��
    if sum(Fgi(3:4)) == 0 % non-contact left
        k(2) = 0 ;
        Fai(2) = 0 ;
    else Fgi(3:4) = 0 ; % contact
        k(2) = exp(-4*log2(norm(vdl(2,1:2)-vd)+eps1)/(norm(vdl(2,1:2))+eps2)) ; % �����₷���w�W
    end
    vd_childa(1,1:2) = (1-k(2))*vdl(1,1:2) + k(2)*vdc(1,1:2); % �E�ɑ΂���Vd�̔z��
    vd_childa(2,1:2) = (1-k(1))*vdl(2,1:2) + k(1)*vdc(2,1:2); % ���ɑ΂���Vd�̔z��
    % �o�͂֕ϊ� 
    % �͂�1���������A���x��2�����Ȃ̂ŁA�s�ǐݒ���H�F�������͂Ƒ��x�̕����͓����Ȃ̂Ő�Βl�Ōv�Z
    vi(1) = sqrt(dt_x2^2+dt_y2^2) ; 
    vi(2) = sqrt(dt_x3^2+dt_y3^2) ; 
    for i = 1:2
        if k(i) ~= 0  
            Fai(i) = Gi*(norm(vd_childa(i,1:2))-vi(i)) ;
        else
            Fai(i) = 0 ;
        end
    end
else % �ǂ̂悤�ɃA�N�`���G�[�^�̓��͂���邩����̏ꍇ
    Fai = zeros(2,1);
end

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
% dt2_xi = P*(pinv(C*P)*(D-C*Q)) + Q ; % �[���t�s��
% dt2_xi = zeros(1,14);

% differential equantion and output
dy = zeros(1,16) ;
dy(1:8)  = dt_xi ;
dy(9:16) = dt2_xi ;
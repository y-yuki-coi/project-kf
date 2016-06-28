function [dy var1 var2] = RKfun_2(y0,isRefresh) % Fgi dt2_xi l_vec th
% 20160624�쐬
% th1�̊p�x�̒�`�A���E�r�̒n�ʔ��͂ɂ�镪��𒼂������A����Ȃ��Ȃ���
% �����₷��k�̒�`�������������̂܂܂ɂ��Ă���iver3�ŏC���j
global g mi li ki yg y_contact u0 Gi bi % Ii

xi = y0(1:6) ; dt_xi = y0(7:12) ; 

% for debug 
% t =30; % first contact h = 10^(-3) 
% t = 4296;% t = 4299 % too large GRF  h = 10^(-4) 
% t = 5246;% t = 4299 % too large GRF  h = 10^(-4) 
% xi = Y(t,1:14) ;dt_xi = Y(t,15:28) ; Y0 = Y(1:t,:); 

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
dt_l1 = 1/2*l1^(-1/2)*(2*(x2-x3)*(dt_x2-dt_x3)+2*(y2-y3)*(dt_y2-dt_y3)) ; % �����f�[�^���猟�؍ς�
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
% u0��x�����̖ڕW���x�Ƃ���
% F�͐L�W��������
eps1 = 10^(-10) ; eps2 = 10^(-4) ; eps3 = 10^(-4) ; % ��̏��Z������邽�߂̔����� 
vd = [u0 0] ; % �ڕW���x
if 1 % 1: Mobility control
    if sum(Fgi(2)) ~= 0 && sum(Fgi(4)) == 0% �E�r�����n: ex1���t�ɂȂ邱�Ƃɒ���
        ex1 = [cos(th1) sin(th1)];
    elseif sum(Fgi(4)) ~= 0 && sum(Fgi(2)) == 0% ���r�����n
        ex1 = [-cos(th1) sin(th1)];
    else % ex1 = [0 0];
        tmp_v1 = [(x2+x3)/2 (y2+y3)/2] - [x1 y1]; % x1�Ƒ��̒��_�����ԃx�N�g���i�r�̒����͕s�ςƉ���j
        ex1 = tmp_v1./norm(tmp_v1); 
    end
    ex2 = [-cos(th2) sin(th2)];
    ex3 = [-cos(th3) sin(th3)];
    % right leg
    vdl(2,1:2) = dot(ex2,vd)*ex2 ; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j
    vi(2) = dt_l2 ;
    vdr(2,1:2) = vd - vdl(1,1:2) ; % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(2,1:2,3) = dot(ex3,vdr(1,1:2))*ex3 ; % �����E�Ɋ�^�ł���z��
    vdc(2,1:2,1) = dot(ex1,vdr(1,1:2))*ex1 ; % �����E�Ɋ�^�ł���z��
%     if sum(Fgi(1:2)) == 0 % non-contact 
%        k(1) = 0 ; 
%     else % contact
        k(2) = exp(-4*log(2)*(norm(vdl(2,1:2)-dt_l2*ex2)^2+eps1)/(norm(vdl(2,1:2))^2+eps2)) ; % 1�𒴂��Ȃ��悤�ɐ݌v����Ă���
% end
    % figure(1); plot(0:0.01:1,exp(-4*log(2).*[0:0.01:1])) % check
    % �|��₷�� �������Ɠ|��ɂ����@�傫���Ɠ|��₷��
    kf(2) = sqrt(g/l2)*(x2-x1)/(dt_x1+eps3) ; % Kagawa & Uno 2010
%     kf(2) = sign(kf(2))*exp(-4*log(2)*kf(2)) ;
    if abs(kf(2)) > 1 ; kf(2) = sign(kf(2)) ; end
    % left leg
    vdl(3,1:2) = dot(ex3,vd)*ex3 ; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j
    vi(3) = dt_l3 ;
    vdr(3,1:2) = vd - vdl(3,1:2) ; % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(3,1:2,2) = dot(ex2,vdr(2,1:2))*ex2 ; % �E�����Ɋ�^�ł���z��
    vdc(3,1:2,1) = dot(ex1,vdr(2,1:2))*ex1 ; % �������Ɋ�^�ł���z��
%     if sum(Fgi(3:4)) == 0 % non-contact left
%        k(2) = 0 ;
%     else Fgi(3:4) = 0 ; % contact
        k(3) = exp(-4*log(2)*(norm(vdl(3,1:2)-dt_l2*ex3)^2+eps1)/(norm(vdl(3,1:2))^2+eps2)) ; % 
%     end
    % �|��₷��
    kf(3) = sqrt(g/l3)*(x3-x1)/(dt_x1+eps3) ; % Kagawa & Uno 2010
%     kf(3) = sign(kf(3))*exp(-4*log(2)*kf(3)) ;
    if abs(kf(3)) > 1 ; kf(3) = sign(kf(3)) ; end
    % Hip
    vi(1) = dt_l1; % �r�Ԃɉ��������ݑ��x
    vdl(1,1:2) = dot(ex1,vd)*ex1 ; %
    k(1) = exp(-4*log(2)*(norm(vdl(1,1:2)-dt_l1*ex1)^2+eps1)/(norm(vdl(1,1:2))^2+eps2)) ; %
    vdr(1,1:2) = vd - vdl(1,1:2) ; % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(1,1:2,2) = dot(ex2,vdr(1,1:2))*ex2 ; % �E�����Ɋ�^�ł���z��
    vdc(1,1:2,3) = dot(ex3,vdr(1,1:2))*ex3 ; % �������Ɋ�^�ł���z��
    % ���g�̓|��₷���̓[��
    kf(1) = 0 ;
    % ��r�������鐬���F�ڒn���n�ŕ��ނ���
    if sum(Fgi(2)) == 0 && sum(Fgi(4)) ~= 0% �E�r�����n: ex1���t�ɂȂ邱�Ƃɒ���
        kf2vdc1 = 0 ; 
        kf3vdc1 = kf(3)*vdc(3,1:2,1) ; % x2>x3�ł��t�ł��Akf(3)>0�ł��t�ł�
    elseif sum(Fgi(4)) == 0 && sum(Fgi(2)) ~= 0% ���r�����n
        kf3vdc1 = 0 ; 
        kf2vdc1 = kf(2)*vdc(2,1:2,1) ; % x2>x3�ł��t�ł��Akf(3)>0�ł��t�ł�
    else 
        kf2vdc1 = 0 ; kf3vdc1 = 0 ; 
    end
    % �S�̂̑��ݍ�p
    vd_childa(1,1:2) = (1-k(2))*(1-k(3))*vdl(1,1:2) + k(2)*vdc(2,1:2,1) + k(3)*vdc(3,1:2,1) + ... % ���ɑ΂���Vd�̔z�� ���̎��_�ŃX�J���[�ɂ���ׂ��H
                       kf2vdc1 + kf3vdc1 ; % �����͓|��邱�Ƃɒ��ڊ֌W���Ȃ��̂ŁA������̂�
    vd_childa(2,1:2) = (1-k(1))*(1-k(3))*vdl(2,1:2) + k(1)*vdc(1,1:2,2) + k(3)*vdc(3,1:2,2) + ...; % �E�ɑ΂���Vd�̔z�� ���̎��_�ŃX�J���[�ɂ���ׂ��H
                       (1-kf(3))*vdl(2,1:2) + kf(3)*vdc(3,1:2,2) ; % ���̍s��ꍀ���r���グ�鐬���i���������͂킩��Ȃ��j
    vd_childa(3,1:2) = (1-k(1))*(1-k(2))*vdl(3,1:2) + k(1)*vdc(1,1:2,3) + k(2)*vdc(2,1:2,3) + ...;
                       (1-kf(2))*vdl(2,1:1) + kf(2)*vdc(2,1:2,3) ; % ���̍s��ꍀ���r���グ�鐬���i���������͂킩��Ȃ��j
    % �o�͂֕ϊ� 
    vdi(1) = dot(vd_childa(1,1:2),ex1) ; % �o�͕����ɕϊ�
    vdi(2) = dot(vd_childa(2,1:2),ex2) ; % �o�͕����ɕϊ�
    vdi(3) = dot(vd_childa(3,1:2),ex3) ;
    for i = 1:3
        tmp = Gi(i)*(vdi(i)-vi(i)) ;
        Fai(i) = tmp ;
%         if abs(tmp) <= 10*Gi(1) 
%             Fai(i) = tmp ;
%         else Fai(i) = sign(tmp)*10*Gi(1) ;
%         end
    end
else % �ǂ̂悤�ɃA�N�`���G�[�^�̓��͂���邩����̏ꍇ
    Fai = zeros(3,1);
    Fai = ones(3,1)*Gi;%zeros(3,1);
    vdl = zeros(3,2);
    vi = zeros(1,3); k = zeros(1,3);
end

Fa1 = (1) ;%0;%
Fa2 = Fai(2) ; Fa3 = Fai(3) ;
var1 = [l_vec vdl(1,1:2) vdl(2,1:2) vdl(3,1:2) vdc(1,1:2,2) vdc(1,1:2,3) vdc(2,1:2,1) vdc(2,1:2,3) vdc(3,1:2,1) vdc(3,1:2,2)]; 
% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

if l01-l1 < 0 ; k_hip = k3 ; b_hip = b3 ; 
else k_hip = k4 ; b_hip = b4 ; 
end
if l02-l2 < 0 ; k_leg1 = k1 ; b_leg1 = b1 ; 
else k_leg1 = k2 ; b_leg1 = b2 ; 
end
if l03-l3 < 0 ; k_leg2 = k1 ; b_leg2 = b1 ; 
else k_leg2 = k2 ; b_leg2 = b2 ; 
end
% differential equantion and output
% ���̃_�C�i�~�N�X�͍����Ă������i�m�F�ρj
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
function [dy var1 var2] = RKfun_3(y0,param,isRefresh) % Fgi dt2_xi l_vec th
% 20160628 review
% th1�̊p�x�̒�`�A���E�r�̒n�ʔ��͂ɂ�镪��𒼂��āA
% �����₷��k�̒�`�����ǂ�����ver2��������ɑ���Ȃ��Ȃ���
global g mi li ki yg y_contact u0 Gi bi kkf % Ii

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

xi = y0(1:6) ; dt_xi = y0(7:12) ; 

m1  = mi(1); m2 = mi(2); 
l01 = l0; l02 = l0; l03 = l0; 
k1  = ki(1); k2 = ki(2) ;  k3 = ki(3) ;  k4 = ki(4) ; kg = ki(5);
b1 = bi(1) ; b2 = bi(2) ; b3 = bi(3) ; b4 = bi(4) ; bg = bi(5) ; % I2 = Ii(2); 
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
th2 = acos((x2-x1)/l2) ; % x1��x2������ɂ��邱�Ƃ��O��
th3 = acos((x3-x1)/l3) ; % x1��x3������ɂ��邱�Ƃ��O�� 
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

% Actuator force -----------------------------------------------------------------------------------
% Mobility 
% u0��x�����̖ڕW���x�Ƃ���
% F�͐L�W��������
eps1 = 1e-10; eps2 = 1e-4; eps3 = 1e-4; % ��̏��Z������邽�߂̔����� 
deltaL = 0.01;

vd = [u0; 0] ; % �ڕW���x
if 1 % 1: Mobility control
     %copute effective direction of 3 muscles, ex1, ex2, ex3
    ex2 = (xx1-xx2)/norm(xx1-xx2); 
    ex3 = (xx1-xx3)/norm(xx1-xx3);     
    if x2 > x3
        xxfore = xx2;
        xxhind = xx3;
    else
        xxfore = xx3;
        xxhind = xx2;
    end
    ex1 = (xxfore-xxhind)/norm(xxfore-xxhind); %tmp2./norm(tmp2);
    
    % right leg, muscle 2
    vdl(:,2) = dot(ex2,vd)*ex2;
    vi(2) = dt_l2 ;
    vdr(2,:) = vd - vdl(:,2) ; % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(:,2,3) = dot(ex3,vdr(:,2))*ex3 ; % �����E�Ɋ�^�ł���z��
    vdc(:,2,1) = dot(ex1,vdr(:,2))*ex1 ; % �����E�Ɋ�^�ł���z��
    k(2) = exp(-4*log(2)*(norm(vdl(:,2)-dt_l2*ex2)^2+eps1)/(norm(vdl(:,2))^2+eps2)) ; % 1�𒴂��Ȃ��悤�ɐ݌v����Ă���
    
    % left leg, muscle 3
    vdl(:,3) = dot(ex3,vd)*ex3; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j
    vi(3) = dt_l3;
    vdr(:,3) = vd - vdl(:,3); % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(:,3,2) = dot(ex2,vdr(:,3))*ex2 ; % �E�����Ɋ�^�ł���z��
    vdc(:,3,1) = dot(ex1,vdr(:,3))*ex1 ; % �������Ɋ�^�ł���z��
    k(3) = exp(-4*log(2)*(norm(vdl(:,3)-dt_l3*ex3)^2+eps1)/(norm(vdl(:,3))^2+eps2)); %
    
    % Hip
    vdl(:,1) = dot(ex1,vd)*ex1; % �Ǐ����x�x�N�g���i�{����^�ł���x�N�g���j    
    vi(1) = dt_l1; % �r�Ԃɉ��������ݑ��x
    vdr(:,1) = vd - vdl(:,1); % ���̊֐߂������ł��Ȃ����x�x�N�g��
    vdc(:,1,2) = dot(ex2,vdr(:,1))*ex2 ; 
    vdc(:,1,3) = dot(ex3,vdr(:,1))*ex3 ; 
    k(1) = exp(-4*log(2)*(norm(vdl(:,1)-dt_l1*ex1)^2+eps1)/(norm(vdl(:,1))^2+eps2)); %    
    
    % �S�̂̑��ݍ�p
    vd_childa(:,1) = (1-k(2))*(1-k(3))*vdl(:,1) + k(2)*vdc(:,2,1) + k(3)*vdc(:,3,1) ; % ���ɑ΂���Vd�̔z��
    vd_childa(:,2) = (1-k(1))*(1-k(3))*vdl(:,2) + k(1)*vdc(:,1,2) + k(3)*vdc(:,3,2) ; % �E�ɑ΂���Vd�̔z��
    vd_childa(:,3) = (1-k(1))*(1-k(2))*vdl(:,3) + k(1)*vdc(:,1,3) + k(2)*vdc(:,2,3) ;
    
    % �o�͂֕ϊ� 
    vdi(1) = dot(vd_childa(:,1),ex1) ; % �o�͕����ɕϊ�
    vdi(2) = dot(vd_childa(:,2),ex2) ; % �o�͕����ɕϊ�
    vdi(3) = dot(vd_childa(:,3),ex3) ;
    
    for i = 1:3
        Fai(i) = Gi(i)*(vdi(i)-vi(i));
    end
end
Fa1 = Fai(1);
Fa2 = Fai(2); 
Fa3 = Fai(3);
var1 = [l_vec vdl(:,1)' vdl(:,2)' vdl(:,3)' vdc(:,1,2)' vdc(:,1,3)' vdc(:,2,1)' vdc(:,2,3)' vdc(:,3,1)' vdc(:,3,2)']; 

% bipedal_model -----------------------------------------------------------------------------------

% Generated Torques
Fgx2 = Fgi(1) ; Fgy2 = Fgi(2) ; Fgx3 = Fgi(3) ; Fgy3 = Fgi(4) ; 

% differential equantion and output
% ���̃_�C�i�~�N�X�͍����Ă������i�m�F�ρj
if x2>=x3
    costh1 =  cos(th1) ; sinth1 =  sin(th1) ;
else 
    costh1 = -cos(th1) ; sinth1 = -sin(th1) ;
end
kf = [0 0 0];
var2 = [vi k kf -Fa1*costh1 Fa1*sinth1 -Fa2*cos(th2) Fa2*sin(th2) -Fa3*cos(th3) Fa3*sin(th3) Fgi' th];

Fs2 = k_leg2 * ( (xx1-xx2) -(xx1-xx2)/norm(xx1-xx2)*l02);
Fs3 = k_leg3 * ( (xx1-xx3) -(xx1-xx3)/norm(xx1-xx3)*l03);
Fb2 = -b_leg2 * ( dt_l2 * (xx1-xx2)/norm(xx1-xx2) );
Fb3 = -b_leg3 * ( dt_l3 * (xx1-xx3)/norm(xx1-xx3) );

dy = zeros(1,12);
dy(1:6)  = dt_xi;
dy( 7: 8)= gg + (-Fs2-Fs3+Fb2+Fb3    )/m1;
dy( 9:10)= gg + ( Fs2    -Fb2    +Fg2)/m2; 
dy(11:12)= gg + (    Fs3     -Fb3+Fg3)/m3; 

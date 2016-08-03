function km = calcMobilityByLengthVelocity(l,l0,dt_l,vdx) 

% for debug
% l = 0.8 ; l0 = 1 ;
% x = norminv([0.05 0.95],l0,0.3) ; 

% force-length relationship
sigma1 = 0.2 ; % 95%����0.5�{
y1 = sqrt(2*pi*sigma1^2)*normpdf((l-l0)/l0,0,sigma1) ; % �ő傪1:�L�т����Ă��鎞

% force-velocity relationship
sigma2 = 5; % �W�������ꂽ���x�̍���-1,1�ŁA�͂�0�ɂȂ�
dt_l0 = vdx; % ��ƂȂ�L�k���x
y2 = (1/ (1+exp(-sigma2*(-dt_l/dt_l0)) ) ) ; % �ő傪1:�L�W���x�������� 

% for debug
% x = -1:0.01:1;
% y = amp*(1./ (1+exp(-sigma2*(x)) ) )  ;
% plot(x,y)
km = y1*(1-y2) ; %  1-

function km = calcMobilityByLengthVelocity(l,l0,dt_l,vdx) 

% for debug
% l = 0.8 ; l0 = 1 ;
% x = norminv([0.05 0.95],l0,0.3) ; 

% force-length relationship
sigma1 = 0.2 ; % 95%が約0.5倍
y1 = sqrt(2*pi*sigma1^2)*normpdf((l-l0)/l0,0,sigma1) ; % 最大が1:伸びきっている時

% force-velocity relationship
sigma2 = 5; % 標準化された速度の差が-1,1で、力が0になる
dt_l0 = vdx; % 基準となる伸縮速度
y2 = (1/ (1+exp(-sigma2*(-dt_l/dt_l0)) ) ) ; % 最大が1:伸展速度が速い時 

% for debug
% x = -1:0.01:1;
% y = amp*(1./ (1+exp(-sigma2*(x)) ) )  ;
% plot(x,y)
km = y1*(1-y2) ; %  1-

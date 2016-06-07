function [Y Fgi dt2_xi] = fun_Wheel(y0,u)

global h
% for debug 
% y0 = Y(t,:)

% Runge-Kutta
[h1 Fgi dt2_xi] = RKfun_Wheel(y0,u,0) ; % isRefresh = 0 ;
[h2 , ~, ~] = RKfun_Wheel(y0+h1*h/2,u,0) ; 
[h3 , ~, ~] = RKfun_Wheel(y0+h2*h/2,u,0) ;
[h4 , ~, ~] = RKfun_Wheel(y0+h3*h,u,1) ; % isRefresh = 1 ;
h5 = (h1+h2*2+h3*2+h4)/6 ; 
Y = y0+h5*h ; 


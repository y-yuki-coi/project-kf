function [Y Fgi dt2_xi] = fun_Actuator(y0)

global h
% for debug 
% y0 = Y(t,:)

% Runge-Kutta
[h1 Fgi dt2_xi] = RKfun(y0,0) ; % isRefresh = 0 ;
[h2 , ~, ~] = RKfun(y0+h1*h/2,0) ; 
[h3 , ~, ~] = RKfun(y0+h2*h/2,0) ;
[h4 , ~, ~] = RKfun(y0+h3*h,1) ; % isRefresh = 1 ;
h5 = (h1+h2*2+h3*2+h4)/6 ; 
Y = y0+h5*h ; 


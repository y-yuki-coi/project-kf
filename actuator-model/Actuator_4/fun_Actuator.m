function [Y result] = fun_Actuator(t,y0,param,h)

% for debug 
% y0 = Y(t,:)

% Runge-Kutta
[h1 result] = RKfun_4(t,    y0,       param,1) ; % isRefresh = 0 ;
[h2 ~]      = RKfun_4(t+h/2,y0+h1*h/2,param,0) ; 
[h3 ~]      = RKfun_4(t+h/2,y0+h2*h/2,param,0) ;
[h4 ~]      = RKfun_4(t+h,  y0+h3*h,  param,0) ; % isRefresh = 1 ;
h5 = (h1+h2*2+h3*2+h4)/6 ; 

Y = y0+h5*h ; 

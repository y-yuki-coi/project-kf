function [Y var1 var2] = fun_Actuator(y0,h)

% for debug 
% y0 = Y(t,:)

% Runge-Kutta
[h1 v11 v21] = RKfun_3(y0,0) ; % isRefresh = 0 ;
[h2 v12 v22] = RKfun_3(y0+h1*h/2,0) ; 
[h3 v13 v23] = RKfun_3(y0+h2*h/2,0) ;
[h4 v14 v24] = RKfun_3(y0+h3*h,1) ; % isRefresh = 1 ;
h5 = (h1+h2*2+h3*2+h4)/6 ; 
var1 = (v11+v12*2+v13*2+v14)/6 ; 
var2 = (v21+v22*2+v23*2+v24)/6 ; 
Y = y0+h5*h ; 


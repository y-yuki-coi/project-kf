function [yOut,result]=computeRungeKutta(timeSpan,y0,param,h)
    
    maxItr = length(timeSpan);
    yOut = [zeros(maxItr, length(y0))]; %initialize Y matrix
    
    y = y0;
    itr = 1;
    
    yOut(itr,:) = y0;
    itr = itr + 1;
    
    for t = timeSpan
        t
        [yNext result] = computeRungeKuttaStep(t,y,param,h)
        
        yOut(itr,:) = yNext;        
        itr = itr+1;
        
        y = yNext;
        if mod(t,1000)==0; 
            t 
        end
    end
    
end

function [y,result]=computeRungeKuttaStep(t,y0,param,h)
% Runge-Kutta

    [h1 result] = dynamics(t,     y0,      param,1) ; % isRefresh = 0 ;
    
    [h2 ~]      = dynamics(t+h/2,y0+h1*h/2,param,0) ; 
    [h3 ~]      = dynamics(t+h/2,y0+h2*h/2,param,0) ;
    [h4 ~]      = dynamics(t+h,  y0+h3*h,  param,0) ; % isRefresh = 1 ;
    h5 = (h1+h2*2+h3*2+h4)/6 ;     
    
    y = y0+h5*h ; 
end



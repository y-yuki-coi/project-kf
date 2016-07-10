function [y,result]=computeRungeKutta(timeSpan,y0,param,h)
    
    for t = 1:maxItr
        [Y(t+1,:) result(t,:)] = computeRungeKuttaStep(t,Y(t,:),param,h); 
        if mod(t,1000)==0; 
            t 
        end
    end
    

    
    
    tic;
    Y = [Y;zeros(maxItr,12)]; %initialize Y matrix
    for t = 1:maxItr
        [Y(t+1,:) result(t,:)] = fun_Actuator(t,Y(t,:),param,h); 
        if mod(t,1000)==0; 
            t 
        end
    end
    time = h:h:(maxItr+1)*h ;  
    toc;
    
    xi = Y(:,1:6) ; dt_xi = Y(:,7:12) ; 
    % jti = calc_joint_pos(xi,li) ;
    delta = time(2)-time(1) ;
    % l_vec = [sqrt((xi(:,3)-xi(:,5)).^2+(xi(:,4)-xi(:,6)).^2) sqrt((xi(:,1)-xi(:,3)).^2+(xi(:,2)-xi(:,4)).^2) ...
    %    sqrt((xi(:,1)-xi(:,5)).^2+(xi(:,2)-xi(:,6)).^2)] ; 
    % l_vec(:,4:6) = diff3p(l_vec(:,1:3),1/h);
    % figure(2); plot(l_vec(:,6),'r'); hold on; plot(dt2_xi(:,6),'b');
        
    for t = 1:maxItr
        [Y(t+1,:) result(t,:)] = fun_Actuator(t,Y(t,:),param,h); 
        if mod(t,1000)==0; 
            t 
        end
    end
end

function [y,result]=computeRungeKuttaStep(t,y0,param,h)
    % Runge-Kutta
    [h1 result] = dynamics(t,     y0,       param,1) ; % isRefresh = 0 ;
    [h2 ~]      = dynamics(t+h/2,y0+h1*h/2,param,0) ; 
    [h3 ~]      = dynamics(t+h/2,y0+h2*h/2,param,0) ;
    [h4 ~]      = dynamics(t+h,  y0+h3*h,  param,0) ; % isRefresh = 1 ;
    h5 = (h1+h2*2+h3*2+h4)/6 ;     
    
    Y = y0+h5*h ; 
end


function Body = calc_body_wheel(xi) 

% calculate body position
global li

l1 = li(1); l2 = li(2); l3 = li(3); l4 = li(4); l5 = li(5); % [m] 1: pelvis width 2: leg length, 3: trunk height 4: shoulder length: 5: head height
footgain = 0.2 ;

for t = 1:size(xi,1)
    x1 = xi(t,1) ; x2 = xi(t,2) ; th1 = mod(xi(t,3),2*pi) ; th2 = mod(xi(t,4),2*pi) ;
    th3 = xi(t,5) ; th4 = xi(t,6) ;

    Body.wheel.xi(t,1:2) = [x1 x2] ; % body
    Body.wheel.thi(t,1:4) = [th1 th2 th3 th4] ; % body
    xr = x2+(l1/2); % 
    xl = x2-(l1/2); % 
    Body.wheel.xr(t,1) = xr; % xr 
    Body.wheel.xl(t,1) = xl; % xl
    
    Body.human.Head(t,:)     = [x1 l2+l3+l5]; % 決定
    Body.human.RShoulder(t,:)= [x1+l4/2 l2+l3]; % 決定
    Body.human.RAnkle(t,:)   = [xr 0.1-0.1*sin(th1)] ; % 履歴の情報で体幹に対して前後上下させることが必要
    if th1 < pi ; Body.human.RToe(t,:) = [xr+th3*footgain 0]; % 履歴の情報で体幹に対して前後させることが必要
    else Body.human.RToe(t,:) = [xr+th3*footgain -0.1*sin(th1)];
    end
    Body.human.RKnee(t,:)    = [xr+th3*footgain/2 (l2+Body.human.RAnkle(t,2))/2]; % 決定
    Body.human.RHip(t,:)     = [xr l2]; % 決定    
    Body.human.LShoulder(t,:)= [x1-l4/2 l2+l3]; % 決定
    Body.human.LAnkle(t,:)   = [xl 0.1-0.1*sin(th2)] ; % 履歴の情報で体幹に対して前後上下させることが必要
    if th2 < pi ; Body.human.LToe(t,:) = [xl+th4*footgain 0]; % 履歴の情報で体幹に対して前後させることが必要
    else Body.human.LToe(t,:) = [xl+th4*footgain -0.1*sin(th2)]; 
    end
    Body.human.LKnee(t,:)    = [xl+th4*footgain/2 (l2+Body.human.LAnkle(t,2))/2]; % 決定
    Body.human.LHip(t,:)     = [xl l2]; % 決定    
    
    
end
function [dy,result]=dynamics(y,param)

%rename params
m1=param.mass(1);
m2=param.mass(2);
m3=param.mass(3);
gg=param.gg;
ks2=param.ks(2);
ks3=param.ks(3);
bs2=param.bs(2);
bs3=param.bs(3);
kk2=param.kk(2);
kk3=param.kk(3);
bk2=param.bk(2);
bk3=param.bk(3);
bc1=param.bc;
kc1=param.kc;
ls2o=param.ls2o;
ls3o=param.ls3o;

%rename state variables
p12=y(1:2);
p13=y(3:4);
p2 =y(5:6);
p3 =y(7:8);
th12 = y(9);
th13 = y(10);
dp12=y(11:12);
dp13=y(13:14);
dp2 =y(15:16);
dp3 =y(17:18);
dth12 = y(19);
dth13 = y(20);

%compute sensor values
[ls2,dls2]=computeDistance(p2,p12);
[ls3,dls3]=computeDistance(p3,p13);
q12 = computePositionByAngle(p12,ls2,th12+phi12); 
q13 = computePositionByAngle(p13,ls3,th13+phi13);
[lk2,dlk2]=computeDistance(p2,q12);
[lk3,dlk3]=computeDistance(p3,q13);
[lc1,dlc1]=computeDistance(p12,p13);

%compute interaction forces
[Fs12]=computeInteractionForce(p12,p2,ls2o,ks2,bs2);
[Fs13]=computeInteractionForce(p13,p3,ls3o,ks3,bs3);
[Fk12]=computeInteractionForce(q12,p2,0,kk2,bk2);
[Fk13]=computeInteractionForce(q13,p3,0,kk3,bk3);
[Fc11]=computeInteractionForce(p12,p13,0,kc1,bc1);

%compute interaction torques
Tk12 = -Fk12*ls2; %torque between base 12 and mass 2
Tk13 = -Fk13*ls3; %torque between base 13 and mass 3
Tr11=0; %torque between base 12 and base 13

%compute ground reaction force
persistent po2;
persistent po3;
[Fg2,po2Refresh]=computeGroundReactionForce(p2,dp2,po2,yg,kg,bg,isRefresh);
[Fg3,po3Refresh]=computeGroundReactionForce(p3,dp3,po3,yg,kg,bg,isRefresh);

%compute control forces and torque
Fa12 = [0;0];
Fa13 = [0;0];
Ta11 = 0;

%dynamics
ddp12  = (m1*gg -Fs12 -Fc11    -Fa12 -Fk13)/m1;
ddp13  = (m1*gg -Fs13 +Fc11    -Fa13 -Fk12)/m1;
ddp2   = (m2*gg +Fs12     +Fg2+Fa12 +Fk12)/m2;
ddp3   = (m3*gg +Fs13     +Fg3+Fa13 +Fk13)/m3;
ddth12 = (                    -Ta11 +Tk12 -Tr11)/I1;
ddth13 = (                    +Ta11 +Tk13 +Tr11)/I1;

dy=[... 
    dp12; dp13; dp2; dp3; th12; th13;...
    ddp12; ddp13; ddp2; ddp3; dth12; dth13;...      
   ];

%return results
result.ddp12 = ddp12;
result.ddp13 = ddp13;
result.ddp2 = ddp2;
result.ddp3 = ddp3;
result.ddth2 = ddth2;
result.ddth3 = ddth3;

result.dp12 = dp12;
result.dp13 = dp13;
result.dp2 = dp2;
result.dp3 = dp3;
result.dth2 = dth2;
result.dth3 = dth3;

result.p12 = p12;
result.p13 = p13;
result.p2 = p2;
result.p3 = p3;
result.th2 = th2;
result.th3 = th3;

result.Fc11 = Fc11;

result.Fs12 = Fs12;
result.Fs13 = Fs13;
result.Fk12 = Fk12;
result.Fk13 = Fk13;
result.Tk12 = Tk12;
result.Tk13 = Tk13;
result.Tr11 = Tr11;

result.Fa12 = Fa12;
result.Fa13 = Fa13;
result.Ta11 = Ta11;

result.ls2 = ls2;
result.ls3 = ls3;
result.dls2 = dls2;
result.dls3 = dls3;

result.lk2 = lk2;
result.lk3 = lk3;
result.dlk2 = dlk2;
result.dlk3 = dlk3;

result.param = param;

end



function [l,dl]=computeDistance(p,q)

px=p(1);
py=p(2);
qx=q(1);
qy=q(2);

l = norm(p-q);
d = l^2;
dl = 1/2*d^(-1/2)*(2*(px-qx)*(dpx-dqx)+2*(py-qy)*(dpy-dqy));

end


function F=computeInteractionForce(p,q,l0,k,b)
    e=(p-q)/norm(p-q);
    
    [l,dl]=computeDistance(p,q);
    
    F=k*((p-q)-l0*e) -b*dl*e;    
end

function position=computePositionByAngle(origin,distance,alpha)
    
    position=origin + distance*[cos(alpha);sin(alpha)];
      
end


function [Fg,poRefresh]=computeGroundReactionForce(p,dp,po,yg,kg,bg,isRefresh)
        
    Fg = [0;0];
    poRefresh = po;
    
    if p(2) < yg %contact
        if isempty(po) && isRefresh
            poRefresh = p;
        else
            
        end
    else
        poRefresh = [];
    end
    
    if ~isempty(po)
        Fg(1) = -kg * (p(1) - po(1)) -  bg*dp(1);
        Fg(2) = -kg * (p(2) - yg ) -min(bg*dp(2),0); 
    end

end
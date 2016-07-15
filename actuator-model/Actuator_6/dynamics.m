function [dy,result]=dynamics(t,y,param,isRefresh)

%rename params
m1=param.mass(1);
m2=param.mass(2);
m3=param.mass(3);
r1=param.r1;
I1=param.I1;
I2=param.I2;
I3=param.I3;
gg=param.gg;
ks2=param.ks(2);
ks3=param.ks(3);
bs2=param.bs(2);
bs3=param.bs(3);
kk2=param.kk(2);
kk3=param.kk(3);
bk2=param.bk(2);
bk3=param.bk(3);
ls2o=param.ls2o;
ls3o=param.ls3o;
phi12=param.phi12;
phi13=param.phi13;
yg=param.yg;
kg=param.kg;
bg=param.bg;
vdx = param.vdx;
desiredHeight=param.desiredHeight;
desiredHeightGain=param.desiredHeightGain;
tipphi12=param.tipphi12;
actuatorGain=param.actuatorGain;

%rename state variables
p12=y(1:2);
p13=y(3:4);
p2 =y(5:6);
p3 =y(7:8);
th12 = y(9);
th13 = y(10);
th2 = y(11);
th3 = y(12);
dp12=y(13:14);
dp13=y(15:16);
dp2 =y(17:18);
dp3 =y(19:20);
dth12 = y(21);
dth13 = y(22);
dth2 = y(23);
dth3 = y(24);

%compute sensor values
[ls2,dls2]=computeDistance(p2,p12,dp2,dp12);
[ls3,dls3]=computeDistance(p3,p13,dp3,dp13);
[q12,dq12]=computePositionByAngle(p12,ls2o/2,th12+phi12,dp12,dls2,dth12);
[q13,dq13]=computePositionByAngle(p13,ls3o/2,th13+phi13,dp13,dls3,dth13);
[lk2,dlk2]=computeDistance(p2,q12,dp2,dq12);
[lk3,dlk3]=computeDistance(p3,q13,dp3,dq13);
[lc1,dlc1]=computeDistance(p12,p13,dp12,dp13);

[tip12,dtip12]=computePositionByAngle(p12,r1,th12+tipphi12,dp12,0,dth12);
%compute interaction forces

%natural length
[p2n,dp2n]=computePositionAtNaturalLength(p2,p12,dp2,dp12,ls2o);
[p3n,dp3n]=computePositionAtNaturalLength(p3,p13,dp3,dp13,ls3o);
[Fs12]=computeInteractionForce(p2,p2n,dp2,dp2n,ks2,bs2);
[Fs13]=computeInteractionForce(p3,p3n,dp3,dp3n,ks3,bs3);

%[Fc11]=computeInteractionForce(p12,p13,dp12,dq13,kc1,bc1);
%Fs12=[0;0];
%Fs13=[0;0];
%Fc11=[0;0];

%[Fk12]=computeInteractionForce(p2,q12,dp2,dq12,kk2,bk2);
%[Fk13]=computeInteractionForce(p3,q13,dp3,dq13,kk3,bk3);

dk12 = rot2dVector((q12-p12)/norm(q12-p12),pi/2);
dk13 = rot2dVector((q13-p13)/norm(q13-p13),pi/2);
distk12=dk12'*(p2-q12);
distk13=dk13'*(p3-q13);
ddistk12=dk12'*(dp2-dq12);
ddistk13=dk13'*(dp3-dq13);
fk12=-kk2*distk12; %- bk2*ddistk12;
fk13=-kk3*distk13; %- bk3*ddistk13;
[Fk12]=fk12*dk12;
[Fk13]=fk13*dk13;

%compute interaction torques
Tk12Vector = cross([Fk12;0],[(q12-p12);0]); %torque between base 12 and mass 2
Tk13Vector = cross([Fk13;0],[(q13-p13);0]); %torque between base 13 and mass 3
Tk12 = Tk12Vector(3);
Tk13 = Tk13Vector(3);

Tr11=0; %torque between base 12 and base 13
%Tk12 = computeInteractionTorque(th2,th12,dth2,dth12,kk2,bk2);
%Tk13 = computeInteractionTorque(th3,th13,dth3,dth13,kk3,bk3);

%compute ground reaction force
persistent po2;
persistent po3;
[Fg2,po2Refresh]=computeGroundReactionForce(p2,dp2,po2,yg,kg,bg,isRefresh);
[Fg3,po3Refresh]=computeGroundReactionForce(p3,dp3,po3,yg,kg,bg,isRefresh);
po2 = po2Refresh;
po3 = po3Refresh;

%compute control forces and torque
vd = [vdx; desiredHeightGain*(desiredHeight-p12(2))];
[ex,vi] = computeKinematicValues(p12,p13,p2,p3,th12,th13,th2,th3,Fg2,Fg3,tip12,dp12,dp13,dp2,dp3,dth12,r1,dls2,dls3);
[vdchilda,km]=ComputeVdchilda( ex, vi, vd );

Fa12 = 0*actuatorGain(2)*(vdchilda(:,2)-vi(:,2)); %-0*(p12-p2);
Fa13 = 0*actuatorGain(3)*(vdchilda(:,3)-vi(:,3)); %-0*(p13-p3);

%Fa12 = 20*ex(:,2);
%Fa13 = 0*ex(:,3);

omegad12Vector=cross([vdchilda(:,1);0], [tip12-p12;0])/norm(tip12-p12)^2;
omegad12= omegad12Vector(3);
Ta11 = actuatorGain(1)*(omegad12-dth12);
%Ta11 = actuatorGain(1)*omegad12;

%dynamics
A = (m1*gg   +Fa12 +Fs12 + Fk12)/m1;
B = (m1*gg   +Fa13 +Fs13 + Fk13)/m1;;
Fc11 = m1/2*(B-A);
ddp12  = (m1*gg  +Fc11    +Fa12 +Fs12 +Fk12)/m1;
ddp13  = (m1*gg  -Fc11    +Fa13 +Fs13 +Fk13)/m1;
ddp2   = (m2*gg      +Fg2 -Fa12 -Fs12 -Fk12)/m2;
ddp3   = (m3*gg      +Fg3 -Fa13 -Fs13 -Fk13)/m3;
ddth12 = ( Tk12               +Ta11 )/I1;
ddth13 = ( Tk13               -Ta11 )/I1;
ddth2  = (-Tk12                    )/I2;
ddth3  = (-Tk13                    )/I3;

dy=[... 
    dp12; dp13; dp2; dp3; dth12; dth13; dth2; dth3;...
    ddp12; ddp13; ddp2; ddp3; ddth12; ddth13; ddth2; ddth3...      
   ];

%return results
result.ddp12 = ddp12;
result.ddp13 = ddp13;
result.ddp2 = ddp2;
result.ddp3 = ddp3;
result.ddth12 = ddth12;
result.ddth13 = ddth13;

result.dp12 = dp12;
result.dp13 = dp13;
result.dp2 = dp2;
result.dp3 = dp3;
result.dth12 = dth12;
result.dth13 = dth13;

result.p12 = p12;
result.p13 = p13;
result.p2 = p2;
result.p3 = p3;
result.th12 = th12;
result.th13 = th13;

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

result.p2n = p2n;
result.p3n = p3n;
result.q12 = q12;
result.q13 = q13;

result.vd = vd;
result.vdchilda = vdchilda;
result.km = km;
result.vi = vi;
result.ex = ex;
result.omegad12 = omegad12;

result.tip12 = tip12;
result.dtip12 = dtip12;

result.param = param;

end


function [l,dl]=computeDistance(p,q,dp,dq)

    px=p(1);
    py=p(2);
    qx=q(1);
    qy=q(2);
    dpx=dp(1);
    dpy=dp(2);
    dqx=dq(1);
    dqy=dq(2);
    
    l = norm(p-q);
    d = l^2 + 1e-07;
    dl = 1/2*d^(-1/2)*(2*(px-qx)*(dpx-dqx)+2*(py-qy)*(dpy-dqy));
    
end

function [pn,dpn]=computePositionAtNaturalLength(p,q,dp,dq,lo)
    
    pn = q + (p-q)/norm(p-q)*lo;
    dpn = dq +(dp-dq)/norm(p-q)*lo + (p-q)*(-1/2*((p-q)'*(p-q))^(-3/2)*(dp-dq)'*(p-q)*2*lo);

end


function F=computeInteractionForce2(p,q,dp,dq,k,b,lo)
    
    e=(p-q)/(norm(p-q));    
    
    F=k*((p-q)-l0*e) -b*(p-q)/norm(p-q)*(dp-dq)

    
end


function F=computeInteractionForce(p,q,dp,dq,k,b)
    
    F = k*(p-q)-b*(dp-dq);
    
    %e=(p-q)/(norm(p-q));    
    %[l,dl]=computeDistance(p,q,dp,dq);    
    %F=k*((p-q)-l0*e) -b*dl*e;        
    
end

function T=computeInteractionTorque(p,q,dp,dq,k,b)
    
    T = -k*(p-q)-b*(dp-dq);
        
end




function [position,dposition]=computePositionByAngle(origin,distance,alpha,dorigin,ddistance,dalpha)

    position=origin + distance*[cos(alpha);sin(alpha)];
    
    dposition=dorigin + [ddistance*cos(alpha)-distance*dalpha*sin(alpha); ...
                         ddistance*sin(alpha)+distance*dalpha*cos(alpha)];    
    
end

function out=rot2dVector(in,alpha)
    m=[cos(alpha) -sin(alpha);  sin(alpha) cos(alpha)];    
    out = m*in;
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

function [ex,vi] = computeKinematicValues(p12,p13,p2,p3,th12,th13,th2,th3,Fg2,Fg3,tip12,dp12,dp13,dp2,dp3,dth12,r1,dls2,dls3);
    
%compute ex
    x2 = p2(1);
    x3 = p3(1);    
    if x2 > x3
        pfore = p2;
        phind = p3;        
        if norm(Fg2) > norm(Fg3)           
            ex1 = (phind-pfore)/norm(phind-pfore);
        else
            ex1 = -(phind-pfore)/norm(phind-pfore);
        end        
    else
        pfore = p3;
        phind = p2;      
        if norm(Fg2) > norm(Fg3)
            ex1 = -(phind-pfore)/norm(phind-pfore);            
        else
            ex1 = (phind-pfore)/norm(phind-pfore);
        end
    end
    ex(:,1) = rot2dVector((tip12-p12)/norm(tip12-p12),-pi/2); 
    ex(:,2) = (p12-p2)/norm(p12-p2);
    ex(:,3) = (p13-p3)/norm(p13-p3);
    
    %compute vi
    vi(:,1) = dth12*r1*ex(:,1);
    vi(:,2) = dls2*ex(:,2);
    vi(:,3) = dls3*ex(:,3);
end


function jti = calc_joint_pos(xi,li)

% calculate joint position
l2 = li(2);
jti = zeros(size(xi,1),6) ;
for t = 1:size(xi,1)
    jti(t,1:2) = xi(t,1:2) ; % Hip
    jti(t,3) = xi(t,1) + l2*cos(xi(t,5)) ; % foot
    jti(t,4) = xi(t,2) - l2*sin(xi(t,5)) ;
    jti(t,5) = xi(t,1) + l2*cos(xi(t,8)) ;
    jti(t,6) = xi(t,2) - l2*sin(xi(t,8)) ;
end
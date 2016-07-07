function [ex1 ex2 ex3] = calc_ex_1(xi)
% ex2,ex3‚Í‹r‚É‰ˆ‚Á‚ÄAex1‚à‹rŠÔ‚É‰ˆ‚Á‚Ä

x1 = xi(1) ; y1 = xi(2) ; 
x2 = xi(3) ; y2 = xi(4) ; 
x3 = xi(5) ; y3 = xi(6) ; 
xx1 = [x1; y1];%top position
xx2 = [x2; y2];%foretip
xx3 = [x3; y3];%hindtip

ex1=(xx2-xx3)/norm(xx2-xx3);
ex2=(xx1-xx2)/norm(xx1-xx2);
ex3=(xx1-xx3)/norm(xx1-xx3);
if x2 > x3
    xxfore = xx2;
    xxhind = xx3;
else
    xxfore = xx3;
    xxhind = xx2;
end
ex1 = (xxfore-xxhind)/norm(xxfore-xxhind);
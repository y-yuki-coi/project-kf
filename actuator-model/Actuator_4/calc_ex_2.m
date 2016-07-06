function [ex1, ex2, ex3] = calc_ex_2(xi,Fgi,deltaL)

% ex2,ex3�͋r�ɉ����āAex1�͋r���s�ς����肵�ĎO�p�`�̊p�x����v�Z
x1 = xi(1) ; y1 = xi(2) ; 
x2 = xi(3) ; y2 = xi(4) ; 
x3 = xi(5) ; y3 = xi(6) ; 
d23=(x2-x3)^2+(y2-y3)^2;
d12=(x1-x2)^2+(y1-y2)^2; 
d31=(x1-x3)^2+(y1-y3)^2;
l1 = sqrt(d23); 
l2 = sqrt(d12); 
l3 = sqrt(d31);
xx1 = [x1; y1];%top position
xx2 = [x2; y2];%foretip
xx3 = [x3; y3];%hindtip
ex2=(xx1-xx2)/norm(xx1-xx2);
ex3=(xx1-xx3)/norm(xx1-xx3);

if x2 > x3
    xxfore = xx2; xfore = x2;
    xxhind = xx3; xhind = x3;
else
    xxfore = xx3; xfore = x3;
    xxhind = xx2; xhind = x2;
end
th1 = acos((xfore-xhind)/norm(xxfore-xxhind));

tmp1 = calc_ang_triang(l1+deltaL,l2,l3) ; % l1�������ʐL�т����̊p�x
if Fgi(2) ~= 0 && Fgi(4) == 0% �E�r�����n
    tmp2 = x2+l2*[cos(pi-th1+tmp1(2)) cos(pi-th1+tmp1(2))] -x1 ; % ������l1���L�т�����x1�̕ψʃx�N�g��
elseif Fgi(4) ~= 0 && Fgi(2) == 0% ���r�����n
    tmp2 = x3+l3*[cos(pi-th1+tmp1(3)) cos(pi-th1+tmp1(3))] -x1 ; % ������l1���L�т�����x1�̕ψʃx�N�g��
else
    if x2 >= x3
        tmp2 = x3+deltaL*[ cos(th1) -sin(th1)]+l3*[cos(pi-th1+tmp1(3)) cos(pi-th1+tmp1(3))] -x1 ; % ������l1���L�т�����x1�̕ψʃx�N�g��
    else tmp2 = x2+deltaL*[ cos(th1) -sin(th1)]+l2*[cos(pi-th1+tmp1(2)) cos(pi-th1+tmp1(2))] -x1 ; % ������l1���L�т�����x1�̕ψʃx�N�g��
    end
end
ex1 = tmp2'./norm(tmp2);
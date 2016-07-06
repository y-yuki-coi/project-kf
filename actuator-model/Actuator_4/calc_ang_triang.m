function y = calc_ang_triang(a,b,c)

% Heron's formula 
% http://keisan.casio.jp/exec/system/1209543011

s = (a+b+c)/2 ;
S = sqrt(s*(s-a)*(s-b)*(s-c)) ;
if a >= b && a >= c
    h = 2*S/a ; B = asin(h/c); C = asin(h/b); A = pi-B-C; 
elseif b >= a && b >= c
    h = 2*S/b ; C = asin(h/a); A = asin(h/c); B = pi-A-C; 
elseif c >= a && c >= b
    h = 2*S/c ; A = asin(h/b); B = asin(h/a); C = pi-A-B; 
else A = NaN;  B = NaN; C = NaN;
end

y = [A B C] ;
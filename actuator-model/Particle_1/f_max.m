function y = f_max(x)
y = x ; 
if length(x) >= 1
    for p = 1:length(x)
        y(p) = max(0,x(p)) ;
    end
else error('f_max�ɋ�s�񂪓����Ă��܂�')
end
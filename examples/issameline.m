function isflag = issameline(Point)
% judege if 3 or 4 different points are on the same line
[s_m,s_n] = size(Point);
isflag = 0;
eps = 10^-20;

if s_n == 1 || s_n ==2
    isflag = 1;
elseif s_n ==3
    area_p = [(Point(1)-Point(5))*(Point(4)-Point(6))-(Point(3)-Point(5))*(Point(2)-Point(6))]/2;
    if abs(area_p) <=eps
        isflag = 1;
    end
elseif s_n == 4
    area_p = [(Point(1)-Point(5))*(Point(4)-Point(6))-(Point(3)-Point(5))*(Point(2)-Point(6))]/2 + [(Point(7)-Point(5))*(Point(4)-Point(6))-(Point(3)-Point(5))*(Point(8)-Point(6))]/2;
    if abs(area_p) <=eps
        isflag = 1;
    end  
end
end

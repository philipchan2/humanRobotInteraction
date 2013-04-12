function [out] = adjustFlockData(pos, isDiagonal)


if isDiagonal
    A1 = makehgtform('zrotate',45*pi/180);
else
    A1 = makehgtform;
end

A2 = makehgtform('xrotate',190*pi/180);

for i = 1:size(pos,1) % loop over rows
    
    A = A1*A2*makehgtform('translate',pos(i,:)); %bird position
    out(i,:) = A(1:3,4).'; %save data
    
end

end
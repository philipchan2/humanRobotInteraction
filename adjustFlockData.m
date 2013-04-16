function [out] = adjustFlockData(pos, isDiagonal)
% tranform the flock of birds position data to the robot coordinate frame
% philip.chan2@gmail.com
% april 2013

% use flockSetup.m or roboarm.m to work on live calibration

if isDiagonal % if the transmitter is oriented on the bias
    A1 = makehgtform('zrotate',45*pi/180);
else
    A1 = makehgtform;
end

A2 = makehgtform('xrotate',190*pi/180);

for i = 1:size(pos,1) % loop over rows
    
    A = A1*A2*makehgtform('translate',pos(i,:)); %bird position
%     A = makehgtform('zrotate',pi)*A;
    out(i,:) = A(1:3,4).'; %save data
    
end

end
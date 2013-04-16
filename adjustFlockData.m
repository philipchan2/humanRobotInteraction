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

% get to table frame
A2 = makehgtform('xrotate',190*pi/180);
   % center on the robot frame in x and y - equivalent to a translate
A3 = makehgtform('translate',[-270 -70 0]/1000);
    % rotate about z to match the robot frame
A4 = makehgtform('zrotate',pi);

for i = 1:size(pos,1) % loop over rows
    
    A = A4*A3*A1*A2*makehgtform('translate',pos(i,:)); %bird position

    out(i,:) = A(1:3,4).'; %save position data
   
end

end
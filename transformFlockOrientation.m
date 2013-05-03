function out = transformFlockOrientation(flockFrame)
% chanp1
% 2013 MAY 2
%transforms a 4x4 HG tranform in the Flock of birds frame and transforms to
%the robot frame. Returns the tranformed HG form
% uses the same orientation transforms as adjustFlockData
% does not do translations

A1 = makehgtform('zrotate',45*pi/180); %assuming the transmitter is oriented on the bias

% get to table frame
A2 = makehgtform('xrotate',190*pi/180);

% rotate about z to match the robot frame
A4 = makehgtform('zrotate',pi);

out = A4*A1*A2*flockFrame;



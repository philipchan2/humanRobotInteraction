function flockCalibrate
% read in the sampled flock of birds data in order to generate the
% tranformation to the labeled space.
% philip.chan2@gmail.com

% the recorded data goes around points counterclockwise
% birds are fixed to a ruler 30 cm apart
close all;clear

% create a figure
figure('name', 'Table Space');hold on; grid on;

% draw the fixed points
% right handed the transmitter is the center
% table lines make x and y, z is up
x = .70; % sets up a cube
y = .76;
z = .30;
% no particular order, 8 points of a cube
calibrationPoints = [
    0 0 0
    x 0 0
    x y 0
    0 y 0
    0 0 z
    x 0 z
    x y z
    0 y z
    ];% rows of xyz

plot3(calibrationPoints(:,1), ...
    calibrationPoints(:,2), ...
    calibrationPoints(:,3), ...
    'o', 'markersize', 10, ...
    'markerfacecolor', 'm');

plot3(0, 0, 0,... % draw the transmitter
    'o', 'markersize', 30, ...
    'markerfacecolor', 'k');

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
% set the camera observer
set(gca, 'cameraPosition', [-1 -1 1]);

%% Flock Data
diagonalShift = 0; % whether to shift diagonal
switch 1
    case 0
        % read the trajectory
        load('flockSample_201345164355');
    case 1
        load('flockSampleDiag_201345164749');
        diagonalShift = 1; % whether to shift diagonal
end
% outdata.time;
% outdata.bird1pos; % elbow
% outdata.bird2pos; % hand


% adjustments
outdata.bird1pos = adjustFlockData(outdata.bird1pos,diagonalShift);
outdata.bird2pos = adjustFlockData(outdata.bird2pos,diagonalShift);

    pos = outdata.bird1pos;
    plot3(pos(:,1), ...
        pos(:,2), ...
        pos(:,3), ...
        'r-');
    pos = outdata.bird2pos;
    plot3(pos(:,1), ...
        pos(:,2), ...
        pos(:,3), ...
        'g-');


% plot over time
if 0
figure; hold on;grid on;
t = outdata.time;
pos = outdata.bird2pos;
plot(t,pos)
legend({'x', 'y', 'z'})
end

end


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



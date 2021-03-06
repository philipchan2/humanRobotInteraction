function flockCalibrate
% read in the sampled flock of birds data in order to generate the
% tranformation to the labeled space.
% philip.chan2@gmail.com
% see the defined function adjustFlockData

% the recorded data goes around points counterclockwise
% birds are fixed to a ruler 30 cm apart
close all;clear

% create a figure
figure('name', 'Robot Space');hold on; grid on;

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


% transform from table frame to robot frame
% center on the robot frame in x and y - equivalent to a translate
A3 = makehgtform('translate',[-270 -70 0]/1000);
% rotate about z to match the robot frame
A4 = makehgtform('zrotate',pi);
for ii = 1:size(calibrationPoints,1)
    temp = A4*A3*makehgtform('translate',calibrationPoints(ii,:));
    calibrationPoints(ii,:) = 1000*temp(1:3,4).';
end

plot3(calibrationPoints(:,1), ...
    calibrationPoints(:,2), ...
    calibrationPoints(:,3), ...
    'o', 'markersize', 10, ...
    'markerfacecolor', 'm');

plot3(calibrationPoints(1,1), ...
    calibrationPoints(1,2), ...
    calibrationPoints(1,3), ... % draw the transmitter
    'o', 'markersize', 30, ...
    'markerfacecolor', 'k');

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
% set the camera observer
set(gca, 'cameraPosition', [-.5 -1 .5]);

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
outdata.bird1pos = adjustFlockData(outdata.bird1pos,diagonalShift)*1000; % mm
outdata.bird2pos = adjustFlockData(outdata.bird2pos,diagonalShift)*1000;

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



function [outdata] = getRecordedFlockData()
% example usage
% bird1 = [200 200  500];
% bird2 = [0 200 500];
% hCyton.hDisplay.setBird1(bird1);
% hCyton.hDisplay.setBird2(bird2);

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
outdata.bird1pos = adjustFlockData(outdata.bird1pos,diagonalShift)*1e3; % mm
outdata.bird2pos = adjustFlockData(outdata.bird2pos,diagonalShift)*1e3; % mm
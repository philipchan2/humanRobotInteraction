% set up the flock of birds system for collection
% philip.chan2@gmail.com
% April 2013
close all;clc

if ~exist('Inputs.FlockOfBirds' );
    addpath(genpath('/Users/chanp1/myopen'));
end

% Requires MiniVIE Utilities
obj = Inputs.FlockOfBirds;
obj.NumSensors = 2;

useSimulatedData = 1;
if useSimulatedData
    obj.IsSimulator = true;
else
    obj.initialize('COM1');
end



% get the position of the two birds and draw a line segment between them

h = figure('name','FoB segment');grid on; hold on;
p = plot3([0 1], [0 1], [0 1],'m.-', 'linewidth', 6); % draw a line
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')


width = 2;
set(gca,'xlim', width*[-1 1],'ylim', width*[-1 1],'zlim', width*[-1 1]);
set(gca, 'cameraPosition', [1 .5 .5]);

k=[];
set(gcf,'keypress','k=get(gcf,''currentchar'');');
disp('Press p key to pause and s key to stop')
title('Press p key to pause and s key to stop')



while 1
    [pos] = obj.getBirdGroup; % get the points
    
    % update the line points
    set(p,'xdata', pos(1,:),...
        'ydata', pos(2,:),...
        'zdata', pos(3,:));
    
    if ~isempty(k)
        if strcmp(k,'s'); break; end;
        if strcmp(k,'p'); pause; k=[]; end;
    end
    
    pause(1/2);
    
    
end

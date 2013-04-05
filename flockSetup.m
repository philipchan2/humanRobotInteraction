% set up the flock of birds system for collection
% philip.chan2@gmail.com
% April 2013
close all;clc;clear
% all switches and power need to be turned on first, 3 in the front
%% Inputs
InLab = 1; % use for the lab
useSimulatedData = 0; % whether to use simulated data
recordTrajectory = 1; % whether to record data for use in testing
%%
if ~exist('Inputs.FlockOfBirds' );
    if InLab
        % in the lab
        addpath(genpath('C:\usr\myopen'));
    else
        %use path on my laptop
        addpath(genpath('/Users/chanp1/myopen'));
    end
end

% Requires MiniVIE Utilities
obj = Inputs.FlockOfBirds;
obj.NumSensors = 2; % using birds 1 and 2

if useSimulatedData
    obj.IsSimulator = true;
else
    obj.initialize('COM1');
end

% get the position of the two birds and draw a line segment between them

% create a figure
h = figure('name','FoB segment');grid on; hold on;
p = plot3([0 1], [0 1], [0 1],'m.-', 'linewidth', 6); % draw a line
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')


% set the axes
width = 2;
set(gca,'xlim', width*[-1 1],'ylim', width*[-1 1],'zlim', width*[-1 1]);
% set the camera observer
set(gca, 'cameraPosition', [1 .5 2]);

% set the figure title and define code to get keypresses on the figure
k=[];
set(gcf,'keypress','k=get(gcf,''currentchar'');');
disp('Press p key to pause and s key to stop')
title('Press p key to pause and s key to stop')

if recordTrajectory
    durationOfRecording = 10*60; %samples
    outdata.time = zeros(durationOfRecording,1); % init the trajectory data
    outdata.bird1pos = zeros(durationOfRecording,3);
    outdata.bird2pos = zeros(durationOfRecording,3);
    counter = 0;% start an index to the outdata
    tic; % start a timer
    pause(3); % wait for little
end
% loop
while 1
    [pos] = obj.getBirdGroup; % get the points from the Flock
    if isempty(pos), continue, end % didn't return data
    
    if obj.IsSimulator
        % massage the data to get into the data capture format
        set(p,'xdata', pos(1,:),...
            'ydata', pos(2,:),...
            'zdata', pos(3,:));
    else
        
        % update the line points on the plot
        set(p,'xdata', pos(:,1),...
            'ydata', pos(:,2),...
            'zdata', pos(:,3));
    end
    
    % pause and stop based on the key presses
    if ~isempty(k)
        if strcmp(k,'s'); break; end;
        if strcmp(k,'p'); pause; k=[]; end;
    end
    
    % display the position data
    %     disp(pos) % this also makes the key presses stop working
    
    if recordTrajectory
        % record data for testing
        counter = counter+1;
        disp(counter);
        if counter > durationOfRecording
            % save the outdata to a matlab file
            a = 'phil chan saved flock of birds data bird1 on elbow, bird2 in hand';
            save(['flockSample_' regexprep(num2str((round(clock))),' ', '')], 'outdata' ,'a');
            
            return % finish
        end
        
        % fill trajectory data
        outdata.time(counter) = toc; % time seconds
        outdata.bird1pos(counter,:) = pos(1,:); %xyz data
        outdata.bird2pos(counter,:) = pos(2,:);
    end
    
    % slow the processing rate by pausing
    pause(1/15); % sec pause
    
end

%Robot#2
clear; close all;clc
% cleanup
%% setpaths
if ~exist('MiniVIE')
    
    %     addpath(genpath('c:\usr\myopen\MiniVIE')); %in lab
    
    %     on Mac laptop
    addpath(genpath(pwd));
    addpath(genpath('/Users/chanp1/myopen'));
end

%% options
useVelocityControl = 0; % whether to use velocity-only control

useFlock = 0; % master switch to enable the flock of birds

FlockLive = 1;% whether to use the live Flock of Birds data 1, or recorded 0
flockCOMstr = 'COM4'; % select the comm port
diagonalShift = 1; % whether the live flock transmitter is set diagonally

useAvoidance = 1; % whether to avoid the flock
maximumRepelDistance = 500; %mm, max distance to consider avoidance
closestAllowedApproach = 300; % mm
useCAAbasedOnFlockSeparation = 1; % whether to bind the closestAllowedApproach to the bird1-bird2 separation

useAttraction = 0;
maximumAttractDistance = 500;
closestAttractApproach = 300;

timeStep = 20; % tuneable parameter that controls speed of robot
% can be dynamic based on the distance from the goal
% The command velocity is vector with this magnitude, mm


%% Flock of birds setup

if useFlock

    if FlockLive
        % setup the flock data
        % Requires MiniVIE Utilities
        objFlock = Inputs.FlockOfBirds;
        objFlock.NumSensors = 2; % using birds 1 and 2
        objFlock.initialize(flockCOMstr);% connect to the system
        
        % init flockPos to arbitrary value
        flockPos(1:2,1:3) = 1000; % outside of the workspace
        
    else % using recorded Flock data
        outdata = getRecordedFlockData();
        ibird = 0; % init index to bird data
    end
end


%%

%Setup communication to robot
MiniVIE.configurePath
import Presentation.CytonI.*

%Controls the VIE plant
hCyton=CytonI;

%Syncs up with the actual robot
% hCyton.connectToHardware('COM1')

hCyton.hPlant.ApplyLimits=true;

% init command to all zeros
q=[0 0 0 0 0 0 0 0].';

% get joint parameters
% q=hCyton.JointParameters;

% q=[-60 -75 0 0 0 0 0 0.6].'*pi/180;% smart starting point

hCyton.setJointParameters(q); pause(3)

%% compute the goal position, units of mm
% trajx = chooses goal trajectory
% trajx = 0  is original test goal trajectory
% trajx = 1  SWEEP is a sinusoidal wave in x-y plane , z=200;
% trajx = 2  is a sinusoidal wave in x-z plane, circle in x-y plane
% trajx = 3  is a box motion; quirky, will work on singularities.
trajx = 6;
switch trajx
    case 0
        xtraj = zeros(1,8)-300;
        ytraj = [zeros(1,4)-100,zeros(1,4)];
        ztraj = [100 200 300 350 350 300 200 100];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 1
        xtraj = [-200:-20:-400];
        ztraj = [ones(1,length(xtraj))*200];
        ytraj = [100*sin((xtraj/100)*pi)+100];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 2
        rady= 200;
        xseed1 = [0:-5:-200];
        %        xseed2 = [-150:50:200];
        yseed1 = -1*[sqrt(rady^2-xseed1.^2)];
        %        yseed2 = -1*[sqrt(rady^2-xseed2.^2)];
        zseed1 = [200*sin((xseed1./100)*pi)+200];
        %        zseed2 = [50*sin((xseed2./200)*pi)+200];
        xtraj = [xseed1];
        ytraj = [yseed1];
        ztraj = [zseed1];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 3
        startPt=[-100;-100;300;1];
        posZ = makehgtform('translate',[0 0 100]);
        negZ = makehgtform('translate',[0 0 -100]);
        posY = makehgtform('translate',[0 100 0]);
        negY = makehgtform('translate',[0 -50 0]);
        posX = makehgtform('translate',[100 0 0]);
        negX = makehgtform('translate',[-50 0 0]);
        pt1=negZ*startPt;
        pt2=posZ*pt1;
        pt3=negX*pt2;
        pt4=negX*pt3;
        pt5=negZ*pt4;
        pt6=posZ*pt5;
        pt7=negY*pt6;
        pt8=negY*pt7;
        pt9=negZ*pt8;
        pt10=posZ*pt9;
        pt11=posX*pt10;
        pt12=posX*pt11;
        pt13=negZ*pt12;
        pt14=posZ*pt13;
        pt15=posY*pt14;
        pt16=posY*pt15;
        goalTraj=[startPt(1:3)';pt1(1:3)';pt2(1:3)';pt3(1:3)';pt4(1:3)';
            pt5(1:3)';pt6(1:3)';pt7(1:3)';pt8(1:3)';pt9(1:3)';pt10(1:3)';
            pt11(1:3)';pt12(1:3)';pt13(1:3)';pt14(1:3)';pt15(1:3)';pt16(1:3)'];
    case 4
        goalTraj=[
            200 -100 0
            350 -250 0
            400 -150 0
            400 150 0
            320 -50 0];
        goalTraj(:,2)=goalTraj(:,2); %Make edit to do correction
        %if trying to get to a point outside of reachable space
        goalOrient= [0  0 -1; 0 -1 0; -1 0 0];
    case 5 % Arc of points
        goalTraj=[
            368 -14  46
            320 -14  218
            250 -270 46
            250 -270 218
            352 -108 46
            ];
        % orientation of the x axis (out of the end effector)
        goalOrient= [0  0 1; 0 1 0; -1 0 0];
    case 6
        % go up and down
        goalTraj=[ repmat([0 -400],20,1), [linspace(100,350,10) -1*linspace(100,350,10)+350].'];
        
        %         useRadialOrientation = 1; % whether to set the orientation of the end effector to a radially outward direction
        goalOrient = [1 -1 0].';
        goalOrient = goalOrient/norm(goalOrient); % must be a unit vector
        
end
if ~exist('useRadialOrientation', 'var'), useRadialOrientation=0;end

% saved points that are marked and in the usable space
pointA = [200 -100 0];
pointB = [350 -250 0];
pointC = [400 -150 0];
pointD = [400 150 0];
pointE = [320   -50     0];

% set the initial trajectory target position
i = 1; % index to traj
goalPos = goalTraj(i,:).';
hCyton.hDisplay.setTarget(goalPos);

%% control loop
keepRunning = 1;
while  keepRunning
    %% update the flock
    if useFlock
        if FlockLive % using the real live flock
            flockPos_prev = flockPos; % save the current position
            [flockPos] = objFlock.getBirdGroup; % get the points from the Flock
            %flockPos  indexed by bird, coordinate 1:3
            
            if ~isempty(flockPos) % if returned data
                
                % transform the flock data to the robot frame
                flockPos(1,:) = adjustFlockData(flockPos(1,:),diagonalShift)*1e3; % mm
                flockPos(2,:) = adjustFlockData(flockPos(2,:),diagonalShift)*1e3;
                
                % plot the birds on the VIE
                hCyton.hDisplay.setBird1(flockPos(1,:));
                hCyton.hDisplay.setBird2(flockPos(2,:));
            else
                % no data returned
                flockPos = flockPos_prev; % keep the flock in the same place
            end
        else % use recorded data
            ibird = ibird+1; %increment
            if ibird > size(outdata.bird1pos,1), ibird=1; end % loop back to the beginning
            hCyton.hDisplay.setBird1(outdata.bird1pos(ibird,:));
            hCyton.hDisplay.setBird2(outdata.bird2pos(ibird,:)); % plot the birds
            %    hCyton.hDisplay.setArm([outdata.bird1pos(ibird,:);outdata.bird2pos(ibird,:)]); %plot line connecting birds
            % set the flock data to a common format with the live option
            flockPos = [outdata.bird1pos(ibird,:) ;outdata.bird2pos(ibird,:)];
        end
    end
    %% update the arm
    
    %get the current position of the end effector in the world frame
    endeffPos = hCyton.hControls.getT_0_N;
    endeffOrient = endeffPos(1:3,1:3); % orientation
    endeffPos = endeffPos(1:3,4); % position
    
    % compute the difference between the goal and current position
    commandVector = goalPos - endeffPos;
    posDiff = norm(commandVector);
       
    % if the goal is reached, go to the next position
    goalMargin = 10; % mm
    
    if posDiff < goalMargin % within some margin
        i = i+1; % increment
        if i > size(goalTraj,1), i=1; end % loop back to the beginning
        
        goalPos = goalTraj(i,:).';
        
        if 1
            % Limit the goal position to within the workspace reach
            tempDist = norm(goalPos); % mag
            if tempDist > 400
                goalPos = goalPos/tempDist*400; % set to be 400 long
            end
        end
        
        hCyton.hDisplay.setTarget(goalPos); % update the display
        %         beep; beep; % alarm when goal reached
        %         keyboard
        
    else
        % compute the unit vector scaled by timeStep
        % can be scaled by posDiff
        commandVector = commandVector/norm(commandVector)*timeStep;
        
        % get the command/desired velocity vector
        if useVelocityControl
            % use velocity only control
            % set angular velocities to do-not-care
            commandVel= [commandVector.' nan nan nan];
        else
            
            % compute the rotation between goal and current orientation wrt base
            thetaPerTimeStep_rad = 5*pi/180; % angle to rotate the end effector each step
   
            if useRadialOrientation
                % compute the goal orientation as the end effector position
                % unit vector
                goalOrient = endeffPos/norm(endeffPos);
            end
            
            % compute the axis of rotation as the cross product of the
            % current and desired frame x axis vector in the world frame
            rotAxis = cross(endeffOrient(:,1),goalOrient(:,1));
            
            % compute the magnitude of the rotation for 1 time step
            magRot  = thetaPerTimeStep_rad;
            
            % compute the angular velocity as the desired magnitude and the
            % rotation axis unit vector
            wmove = magRot*rotAxis/norm(rotAxis);          
            
            % set the velocity and angular velocity
            commandVel= [commandVector.' wmove'];
        end
        
        
        if useAvoidance && useFlock % change the command 3D velocity to avoid the flock
            % flockPos, position of the two birds
            % endeffPos, effector position
            
            % select points to be sources of repulsion
            % repelPos = flockPos(1,:); % select a bird
            repelPos = flockPos; % select all birds
            
            numRepellers = size(repelPos,1);
            % be repelled by all positions in repelPos
            for iRepel = 1:numRepellers
                repelVec = endeffPos.' - repelPos(iRepel,:); % vector of repulsion
                repelDist = norm(repelVec); % 3D distance, mm
                
                if repelDist <= maximumRepelDistance
                    %generate a repelling velocity from the repel position
                    
                    % velocity is parabolic with zero at the
                    % maximumRepelDistance and matches timeStep at the
                    % closestAllowedApproach
                    
                    if useCAAbasedOnFlockSeparation
                        % modify the closestAllowedApproach based on the separation between birds
                        distBird2bird = flockPos(1,:) - flockPos(2,:); %requires 2 birds
                        distBird2bird = norm(distBird2bird); % magnitude
                        closestAllowedApproach = distBird2bird; % set the closest allowed approach
                    end
                    
                    repelMag = (maximumRepelDistance-repelDist)^2*timeStep/(maximumRepelDistance-closestAllowedApproach)^2;
                    repelVelocity = repelMag*repelVec/repelDist;
                    
                    % vector sum of repel and command velocity
                    commandVel(1:3) = commandVel(1:3) + repelVelocity;
                    
                else % free movement, don't change the command
                end
            end
        elseif useAttraction && useFlock %--- Attraction/Chase sequence
            % flockPos, position of the two birds
            % endeffPos, effector position
            repelPos = flockPos(1,:); % select a bird
            %                 repelPos = goalTraj(4,:); % avoid a goal point
            
            repelVec = endeffPos.' - repelPos; % vector of repulsion
            repelDist = norm(repelVec); % 3D distance, mm
            
            if repelDist <= maximumRepelDistance
                %generate an attraction velocity from the attraction position
                
                % velocity is parabolic with zero at the
                % maximumAttractDistance and matches timeStep at the
                % closestAllowedApproach
                repelMag = (maximumRepelDistance-repelDist)^2*timeStep/(maximumRepelDistance-closestAllowedApproach)^2;
                repelVelocity = -repelMag*repelVec/repelDist;
                
                % vector sum of repel and command velocity
                %commandVel(1:3) = commandVel(1:3) + repelVelocity;
                
                commandVel(1:3) = repelVelocity; % use attraction only
                
            else % free movement, don't change the command
            end
        end
        
        [qdot, J] = hCyton.hControls.computeVelocity(commandVel); % get the joint velocities
                
        % compute the command position
        q = q + qdot; % qdot is already scaled by time
         
        % command the robot position in joint space q
        hCyton.setJointParameters(q);
        
        pause(.5); % wait
        
    end
end
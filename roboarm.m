%Robot#2
clear; close all;clc
% cleanup
%% setpaths
if ~exist('MiniVIE')
    
    %     addpath(genpath('c:\usr\myopen\MiniVIE')); %in lab
    
    %     on Mac laptop
    %     addpath(genpath(pwd));
    %     addpath(genpath('/Users/chanp1/myopen'));
    
    % in the lab thursday
    restoredefaultpath;
    addpath(genpath('C:\Documents and Settings\vradmin\Desktop\chan\thu\myopen'));
end

%% options
robotCOMstr = 'COM1';
flockCOMstr = 'COM4'; % select the comm ports

modeString = 'demoAvoidance1';
% demoCalibration
% demoAvoidance1
% demoAttraction
% demoOrientationControl
% manual

switch modeString
    case 'demoCalibration'
        % Settings for the demonstration calibration between the Flock and
        % the robot. Use this mode to make adjustments to the robot
        % location, transmitter location, or the tranform between the two
        % frames. End effector fully attractive to bird 1.
        useVelocityControl = 1; % whether to use velocity-only control
        useFlock = 1; % master switch to enable the flock of birds
        FlockLive = 1;% whether to use the live Flock of Birds data 1, or recorded 0
        diagonalShift = 1; % whether the live flock transmitter is set diagonally
        
        useAvoidance = 0; % whether to avoid the flock
        useAttraction = 1;
        timeStep = 20; % move 1 cm at each step
        timeDelayBetweenCommands = .1; % fast movements for calibration
        trajx = 6;
    case 'demoAvoidance1'
        % follow the goal trajectory while avoiding the flock sensors
        % arm will avoid the birds separately
        % closest approach distance is controlled by inter-bird distance
        % end effector can be chased away
        
        useVelocityControl = 1; % whether to use velocity-only control
        useFlock = 1; % master switch to enable the flock of birds
        FlockLive = 1;% whether to use the live Flock of Birds data 1, or recorded 0
        diagonalShift = 1; % whether the live flock transmitter is set diagonally
        
        useAvoidance = 1; % whether to avoid the flock
        maximumRepelDistance = 500; %mm, max distance to consider avoidance
        useCAAbasedOnFlockSeparation = 1; % whether to bind the closestAllowedApproach to the bird1-bird2 separation
        
        useAttraction = 0;
        timeStep = 20; % move cm at each step
        timeDelayBetweenCommands = .2; % update rate
        trajx = 5;
        
    case 'demoAttraction'
        % attract to the bird sensor only if commanded to do so. Otherwise,
        % the robot will follow its other goals
        useVelocityControl = 1; % whether to use velocity-only control
        useFlock = 1; % master switch to enable the flock of birds
        FlockLive = 1;% whether to use the live Flock of Birds data 1, or recorded 0
        diagonalShift = 1; % whether the live flock transmitter is set diagonally
                
        useAttraction = 1;
        useAvoidance = 0;
        timeStep = 20; % move cm at each step
        timeDelayBetweenCommands = .2; % update rate
        trajx = 7;
        
        useAttractionCommand = 1; % command attraction on and off using bird orientation
        holdPosition = 1; % hold a single point and continue to respond
        
    case 'demoOrientationControl'
        % control the end effector orientation using the flock
        % constellation. The orientation goal is in the direction of bird
        % 1 to 2 point direction.
        
        useVelocityControl = 0; % whether to use velocity-only control
        useFlock = 1; % master switch to enable the flock of birds
        FlockLive = 1;% whether to use the live Flock of Birds data 1, or recorded 0
        diagonalShift = 1; % whether the live flock transmitter is set diagonally
        useAvoidance = 0; % whether to avoid the flock
        useAttraction = 0;
        useOrientationControl = 1;
        timeStep = 20; % move 2 cm at each step
        timeDelayBetweenCommands = .5; % slower update rate
        holdPosition = 1;
        trajx = 7;
        
    otherwise %default to 'manual'    manual entry
        useVelocityControl = 1; % whether to use velocity-only control
        
        useFlock = 1; % master switch to enable the flock of birds
        
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
        timeDelayBetweenCommands = .5; % added delay between robot commands
        trajx = 6;
end

% if varibles were not set, then set to default
if ~exist('useOrientationControl', 'var'), useOrientationControl=0;end
if ~exist('holdPosition', 'var'), holdPosition=0;end
if ~exist('useAttractionCommand', 'var'), useAttractionCommand=0;end  


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
hCyton.connectToHardware(robotCOMstr)

hCyton.hPlant.ApplyLimits=true;


% init command to all straight up with gripper closed
q=[0 0 0 0 0 0 0 .8].';

% get joint parameters
% q=hCyton.JointParameters;

% q=[-60 -75 0 0 0 0 0 0.6].'*pi/180;% smart starting point

hCyton.setJointParameters(q);
while ~hCyton.hPlant.allMovesComplete
    pause(timeDelayBetweenCommands); % wait a cyle
end

%% compute the goal position, units of mm
% trajx = chooses goal trajectory
% trajx = 0  is original test goal trajectory
% trajx = 1  SWEEP is a sinusoidal wave in x-y plane , z=200;
% trajx = 2  is a sinusoidal wave in x-z plane, circle in x-y plane
% trajx = 3  is a box motion; quirky, will work on singularities.

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
        arcRad = 350;
        xvec = linspace(350,0,51);
        yvec = -sqrt(arcRad^2-xvec.^2);
        zvec =100*sin(xvec*3*pi/180)+120;
% goalTraj=[
% 368 -14 46
% 320 -14 218
% 250 -270 46
% 250 -270 218
% 352 -108 46
% ];
        goalTraj=[xvec' yvec' zvec'];
        % orientation of the x axis (out of the end effector)
        goalOrient= [0 -1 -0.5].';
        goalOrient=goalOrient/norm(goalOrient);
%         goalTraj=[
%             368 -14  46
%             320 -14  218
%             250 -270 46
%             250 -270 218
%             352 -108 46
%             ];
%         % orientation of the x axis (out of the end effector)
%         goalOrient= [0  0 1; 0 1 0; -1 0 0];
    case 6
        % go up and down
        goalTraj=[ repmat([200 -300],20,1), [linspace(100,400,10) -1*linspace(100,400,10)+400].'];
        
        %         useRadialOrientation = 1; % whether to set the orientation of the end effector to a radially outward direction
        goalOrient = [0 -1 -.5].';
        goalOrient = goalOrient/norm(goalOrient); % must be a unit vector
    case 7
        goalTraj=[ 0 -350 350];
        goalOrient = [0 0 1].';
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
updateCommand = 1; % init
flockFrame = eye(3);% init
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
            
            if useAttractionCommand % use the bird to make a binary command
                flockFrame_prev = flockFrame; % save the old
                flockFrame = objFlock.getframes; % get the new
                if ~isempty(flockFrame) % if returned data
                    flockFrame = flockFrame(:,:,1); % isolate bird 1
                    flockFrame = transformFlockOrientation(flockFrame); % return the transform in the robot frame
                else
                    flockFrame = flockFrame_prev;
                end
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
    
    if posDiff < goalMargin && ~holdPosition % within some margin
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
        commandVector = commandVector/norm(commandVector)*timeStep; %mm
        
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
                
            elseif useOrientationControl && useFlock
                % compute the goal pointing orientation as the vector from
                % bird 1 to 2
                goalOrient = flockPos(2,:) - flockPos(1,:); %requires 2 birds
                goalOrient = goalOrient.'/norm(goalOrient); % make unit vector
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
        
        
        % modify commandVel from strictly seeking its current goal to
        % another task
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
                        disp(['closest allowed approach ' num2str(closestAllowedApproach)])
                    end
                    
                    repelMag = (maximumRepelDistance-repelDist)^2*timeStep/(maximumRepelDistance-closestAllowedApproach)^2;
                    repelVelocity = repelMag*repelVec/repelDist;
                    
                    % vector sum of repel and command velocity
                    commandVel(1:3) = commandVel(1:3) + repelVelocity;
                    
                else % free movement, don't change the command
                end
            end
        elseif useAttraction && useFlock %--- Attraction/Chase sequence

            repelPos = flockPos(1,:); % select a bird
            
            repelVec = endeffPos.' - repelPos; % vector of repulsion
            repelDist = norm(repelVec); % 3D distance, mm
            disp(['Distance to bird ' num2str(repelDist)])
            attractionGoal = 20; % mm
            if repelDist > attractionGoal
                %generate an attraction velocity from the attraction position
                
                % set the magnitude to be fixed or the error
                repelMag = min(20,repelDist);
                repelVelocity = -repelMag*repelVec/repelDist;
                
                if useAttractionCommand
                    % decide whether to attract or not based on the bird
                    % orientation
                    
                    % if the bird z is facing the endeffector within x
                    % degress, then attract otherwise, don't

                    if dot(flockFrame(1:3,3),repelVec/repelDist) >= cosd(60)
                        attract2bird = 1;
                        disp('attracting')
                    else
                        attract2bird = 0;
                    end
                    
                    if attract2bird
                        % use attraction only without other goal
                        commandVel(1:3) = repelVelocity;
                    else
                        % the arms will attract to its original goal
                    end
                    
                else
                    % use attraction only without other goal
                    commandVel(1:3) = repelVelocity;
                end
                updateCommand = 1; % update
                
                q(8) = 0.8;hCyton.setJointParameters(q); % close the gripper
            
            else % the attraction goal has been reached
                q(8) = 0;hCyton.setJointParameters(q); % open the gripper
                
                updateCommand = 0; % don't update if the attraction goal is reached
                % no velocity
                commandVel(1:3) = [0 0 0];
                % this may cause some jitter in the robot - consider using
                % a flag to toggle motion on and off
            end
            
        end
        
        if updateCommand
            [qdot, J] = hCyton.hControls.computeVelocity(commandVel); % get the joint velocities
            
            % compute the command position
            q = q + qdot; % qdot is already scaled by time
            
            % command the robot position in joint space q
            hCyton.setJointParameters(q);
            while ~hCyton.hPlant.allMovesComplete
                pause(timeDelayBetweenCommands); % wait a cyle
            end
        end
        pause(timeDelayBetweenCommands); % wait
        
    end
end

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

useFlock = 1; % master switch to enable the flock of birds

FlockLive = 0;% whether to use the live Flock of Birds data 1, or recorded 0
flockCOMstr = 'COM4'; % select the comm port
diagonalShift = 1; % whether the live flock transmitter is set diagonally

useAvoidance = 1; % whether to avoid the flock
maximumRepelDistance = 500; %mm, max distance to consider avoidance 
closestAllowedApproach = 200; % mm

timeStep = 10; % tuneable parameter that controls speed of robot
% can be dynamic based on the distance from the goal
% The command velocity is vector with this magnitude, mm

%% constant
m2mm = 1e3;
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
        outdata.bird1pos = adjustFlockData(outdata.bird1pos,diagonalShift)*m2mm;
        outdata.bird2pos = adjustFlockData(outdata.bird2pos,diagonalShift)*m2mm;
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

% init command to all zeros
q=[0 0 0 0 0 0 0 0].';
hCyton.setJointParameters(q);
hCyton.hPlant.ApplyLimits=true;


%% compute the goal position, units of mm
% trajx = chooses goal trajectory
% trajx = 0  is original test goal trajectory
% trajx = 1  SWEEP is a sinusoidal wave in x-y plane , z=200;
% trajx = 2  is a sinusoidal wave in x-z plane, circle in x-y plane
% trajx = 3  is a box motion; quirky, will work on singularities.
trajx = 4;
switch trajx
    case 0
        xtraj = zeros(1,8)-300;
        ytraj = [zeros(1,4)-100,zeros(1,4)];
        ztraj = [100 200 300 350 350 300 200 100];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 1
        xtraj = [200:20:400];
        ztraj = [ones(1,length(xtraj))*200];
        ytraj = [100*sin((xtraj/100)*pi)+100];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 2
        rady= 200;
        xseed1 = [200:-50:-200];
        xseed2 = [-150:50:200];
        yseed1 = [sqrt(rady^2-xseed1.^2)];
        yseed2 = -1*[sqrt(rady^2-xseed2.^2)];
        zseed1 = [50*sin((xseed1./200)*pi)+200];
        zseed2 = [50*sin((xseed2./200)*pi)+200];
        xtraj = [xseed1,xseed2];
        ytraj = [yseed1,yseed2];
        ztraj = [zseed1,zseed2];
        goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
    case 3
        startPt=[-200;-200;100;1];
        posZ = makehgtform('translate',[0 0 100]);
        negZ = makehgtform('translate',[0 0 -100]);
        posY = makehgtform('translate',[0 400 0]);
        negY = makehgtform('translate',[0 -400 0]);
        posX = makehgtform('translate',[400 0 0]);
        negX = makehgtform('translate',[-400 0 0]);
        pt1=posZ*startPt;
        pt2=posY*pt1;
        pt3=negZ*pt2;
        pt4=posZ*pt3;
        pt5=posX*pt4;
        pt6=negZ*pt5;
        pt7=posZ*pt6;
        pt8=negY*pt7;
        pt9=negZ*pt8;
        pt10=posZ*pt9;
        pt11=negX*pt10;
        goalTraj=[startPt(1:3)';pt1(1:3)';pt2(1:3)';pt3(1:3)';pt4(1:3)';
            pt5(1:3)';pt6(1:3)';pt7(1:3)';pt8(1:3)';pt9(1:3)';pt10(1:3)';
            pt11(1:3)'];
    case 4
        goalTraj=[0 0 400; 200 0 300];
end
%goalTraj=[xtraj(:),ytraj(:),ztraj(:)];

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
                
                % tranform the flock data to the robot frame
                flockPos(1,:) = adjustFlockData(flockPos(1,:),diagonalShift)*m2mm;
                flockPos(2,:) = adjustFlockData(flockPos(2,:),diagonalShift)*m2mm;
%                 flockPos(:,1:2) = flockPos(:,1:2) - 200;
%                 flockPos(:,1) = flockPos(:,1)-70;
%                 flockPos(:,2) = flockPos(:,2)+130;
%                 A = makehgtform('zrotate',pi);
%                 A = A(1:3,1:3);
%                 flockPos(1,:) = (A*flockPos(1,:).').';
%                 flockPos(2,:) = (A*flockPos(2,:).').';
                
                
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
    
    %get the current position of the end effector
    endeffPos = hCyton.hControls.getT_0_N;
    endeffPos = endeffPos(1:3,4);
    
    % compute the difference between the goal and current position
    commandVector = goalPos - endeffPos;
    posDiff = norm(commandVector);
    
    % if the goal is reached, stop
    goalMargin = 10;
    if posDiff < goalMargin % within some margin
        i = i+1; % increment
        if i > size(goalTraj,1), i=1; end % loop back to the beginning
        
        goalPos = goalTraj(i,:).';
        hCyton.hDisplay.setTarget(goalPos);
        
        %         keepRunning = 0;
        %         disp('goal reached')
        
    else
        % compute the unit vector scaled by timeStep
        % can be scaled by posDiff
        commandVector = commandVector/norm(commandVector)*timeStep;
        
        % get the command/desired velocity vector
        if 0
            commandVel= [commandVector.' 0 0 0]; % set orientations to zero
            % this has problems because the orientation is locked
            % get the Jacobian and pseudoinverse
            q    = hCyton.JointParameters;
            numJ = hCyton.hControls.numericJacobian(q);
            invJ = pinv(numJ);
            
            % compute the joint rates to realize the motion
            qdot = invJ*commandVel';
            
            % use the time step to compute the command position
            q    = q + [qdot; 0]; % the last command is for the gripper
        else
            commandVel= [commandVector.' nan nan nan]; % set orientations to do-not-care
            
            if useAvoidance && useFlock % change the command 3D velocity to avoid the flock
                % flockPos, position of the two birds
                % endeffPos, effector position
                repelPos = flockPos(2,:); % select a bird
%                 repelPos = goalTraj(4,:); % avoid a goal point
                
                
                repelVec = endeffPos.' - repelPos; % vector of repulsion
                repelDist = norm(repelVec); % 3D distance, mm
                
                if repelDist <= maximumRepelDistance
                    %generate a repelling velocity from the repel position
                    
                    % velocity is parabolic with zero at the
                    % maximumRepelDistance and matches timeStep at the
                    % closestAllowedApproach
                    repelMag = (maximumRepelDistance-repelDist)^2*timeStep/(maximumRepelDistance-closestAllowedApproach)^2;
                    repelVelocity = repelMag*repelVec/repelDist;
                    
                    % vector sum of repel and command velocity
                    commandVel(1:3) = commandVel(1:3) + repelVelocity;
                    
                else % free movement, don't change the command
                end

            end
            [qdot, J] = hCyton.hControls.computeVelocity(commandVel); % get the joint velocities
            
            % use the time step to compute the command position
            q    = q + [qdot]; % the last command is for the gripper
        end
        
        % command the robot position in joint space q
        hCyton.setJointParameters(q);
        
        pause(.1); % wait
        
    end
end








return

% for x=100:10:400;
%
%     %Planned trajectory of robot arm
%     y = 300*sin(x);
%     p = [x y 300];
%
%     %Create line segment for user forearm
%     birdline = [outdata.bird1pos(1,:);outdata.bird2pos(1,:)]
%     birdline_len = sqrt(sum((birdline(1,:)-birdline(2,:)).^2));
%     bird_dir = (birdline(1,:)-birdline(2,:))/birdline_len;
%
%     %Intersection
%     %Check if robot end-effector intersects forearm
%     %If intersection: Move up in z-direction
%     %If no intersection: Robot continues to point p
%     robo = p-outdata.bird1pos(1,:);
%     robo_len = sqrt(robo.^2);
%     robo_dir = robo/robo_len;
%     if (robo_dir == bird_dir & robo_len<=birdline_len)
%         p = [x y 400];
%     end
%     M = makehgtform('translate',p);
%     hCyton.hDisplay.setTarget(M);
%     %hCyton.hControls.goto(p);
%
%
%
% end

%Test case1:
%Set birdline to a direction that will not intersect with the planned
%trajectory

%Test case2:
%Set birdline to intersect with planned trajectory

% I've updated the Cyton VIE code based on some questions during class. The short version is that you should control the robot using the Jacobian directly. The setEndEffectorPose() and goto() methods are examples and don't account for joint limits and don't converge for every point in space.
%
% An example of this can be seen here:
%
% >> Presentation.CytonI.CytonEndpointGui.Run
% This function uses the Jacobian directly, as you should:
%
% % Get dh param constants
% [~, a, d] = hCyton.hControls.getDHParams();
%
% % Get Jacobian at the current position
% J_ = hCyton.hControls.symJacobianFull(a(6),d(2),d(3),d(4),d(5),d(6),q(1),q(2),q(3),q(4),q(5),q(6));
% the next steps are to invert the jacobian, multiply by the desired velocity, and apply the joint velocities.
% Using the -Z slider you can see the robot does behave well when it is not in contact with joint limits. However unexpected behavior results when the 'elbow' joint locks out. Think about why this is.
%


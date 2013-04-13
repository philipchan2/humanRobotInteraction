%Setup communication to robot
 cd c:\usr\myopen\MiniVIE
clear; close all;clc

MiniVIE.configurePath
import Presentation.CytonI.*

%Controls the VIE plant
hCyton=CytonI;
obj.hCyton = hCyton; %strange usage

%Syncs up with the actual robot
%  hCyton.connectToHardware('COM5')

% Get dh parameter constants
%     [xform, a, d] = hCyton.hControls.getDHParams();

% init command to all zeros
q=[0 0 0 0 0 0 0 0].';
obj.hCyton.setJointParameters(q);
obj.hCyton.hPlant.ApplyLimits=true;

timeStep = 10; % tuneable parameter that controls speed -


% can be dynamic based on the distance from the goal

% compute the goal position
% trajx = chooses goal trajectory
% trajx = 0  is original test goal trajectory
% trajx = 1  is a sinusoidal wave in x-z plane , straight line in x-y plane;
% trajx = 2  is a sinusoidal wave in x-z plane, circle in x-y plane 
trajx = 2;
switch trajx
    case 0
        xtraj = zeros(1,8)-300;
        ytraj = [zeros(1,4)-100,zeros(1,4)];
        ztraj = [100 200 300 350 350 300 200 100];
    case 1
        xtraj = [-300:50:300];
        ytraj = [ones(1,length(xtraj))*200];
        ztraj = [100*sin((xtraj/300)*pi/2)+100];
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
    case 3
        
end   
goalTraj=[xtraj(:),ytraj(:),ztraj(:)];
i = 1; % index to traj
goalPos = goalTraj(i,:).';
hCyton.hDisplay.setTarget(goalPos);


%% Flock of birds data
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
m2mm = 1e3;
outdata.bird1pos = adjustFlockData(outdata.bird1pos,diagonalShift)*m2mm;
outdata.bird2pos = adjustFlockData(outdata.bird2pos,diagonalShift)*m2mm;
ibird = 0; % index to bird data
% center on the robot frame with recorded data
outdata.bird1pos(:,1:2) = outdata.bird1pos(:,1:2) - 300;% just for testing with recorded data
outdata.bird2pos(:,1:2) = outdata.bird2pos(:,1:2) - 300;% just for testing with recorded data

counter = 0; % init run counter
keepRunning = 1; % init
while  keepRunning
    counter = counter +1;
    if counter > 200
        keepRunning = 0;
        beep;beep;
        disp('finished time')
    end
    
    %% update the flock
    ibird = ibird+3; %increment
    if ibird > size(outdata.bird1pos,1), ibird=1; end % loop back to the beginning
    hCyton.hDisplay.setBird1(outdata.bird1pos(ibird,:));
    hCyton.hDisplay.setBird2(outdata.bird2pos(ibird,:)); % plot the birds
%    hCyton.hDisplay.setArm([outdata.bird1pos(ibird,:);outdata.bird2pos(ibird,:)]); %plot line connecting birds
    
    %% update the arm
    
    %get the current position of the end effector
    endeffPos = obj.hCyton.hControls.getT_0_N;
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
        % compute the unit vector
        % can be scaled by posDiff
        commandVector = commandVector/norm(commandVector)*timeStep;
        
        % get the command/desired velocity vector
        if 0
            commandVel= [commandVector.' 0 0 0]; % set orientations to zero
            % this has problems because the orientation is locked
            % get the Jacobian and pseudoinverse
            q    = obj.hCyton.JointParameters;
            numJ = obj.hCyton.hControls.numericJacobian(q);
            invJ = pinv(numJ);
            
            % compute the joint rates to realize the motion
            qdot = invJ*commandVel';
            
            % use the time step to compute the command position
            q    = q + [qdot; 0]; % the last command is for the gripper
        else
            commandVel= [commandVector.' nan nan nan]; % set orientations to do-not-care
            [qdot, J] = obj.hCyton.hControls.computeVelocity(commandVel);
            
            % use the time step to compute the command position
            q    = q + [qdot]; % the last command is for the gripper
        end
               
        obj.hCyton.setJointParameters(q);
        
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


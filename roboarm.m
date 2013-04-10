%Setup communication to robot
% cd c:\usr\myopen\MiniVIE
clear; close all;clc
MiniVIE.configurePath
import Presentation.CytonI.*

%Controls the VIE plant
hCyton=CytonI;
obj.hCyton = hCyton; %strange usage

%Syncs up with the actual robot
 hCyton.connectToHardware('COM5')

% Get dh parameter constants
%     [xform, a, d] = hCyton.hControls.getDHParams();

% init command to all zeros
q=[0 0 0 0 0 0 0 0].';
obj.hCyton.setJointParameters(q);

timeStep = 10; % tuneable parameter that controls speed -
% can be dynamic based on the distance from the goal

% compute the goal position
goalTraj = [
    -300 -100 100
    -300 -100 200
    -300 -100 300
    -300 -100 350
    -300 0 350
    0 0 500
    ];

i = 1; % index to traj
goalPos = goalTraj(i,:).';
hCyton.hDisplay.setTarget(goalPos);

counter = 0;
keepRunning = 1; % init
while  keepRunning
    counter = counter +1;
    if counter > 500
        keepRunning = 0;
        beep;beep;
        disp('finished time')
    end
    
    
    %get the current position of the end effector
%     jointFrame = obj.hCyton.hControls.getJointFrames;
%     jointFrame = jointFrame(:,:,7); % end effector only
%     endeffPos = jointFrame(1:3,4); % position only
    
    endeffPos = obj.hCyton.hControls.getT_0_N;
    endeffPos = endeffPos(1:3,4);
    
    % compute the difference between the goal and current position
    commandVector = goalPos - endeffPos;
    posDiff = norm(commandVector);
    
    % if the goal is reached, stop
    goalMargin = 10;
    if posDiff < goalMargin % within some margin
        i = i+1; % increment
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
        
        pause(.25); % wait
        
    end
end

return

for x=100:10:400;
    
    %Planned trajectory of robot arm
    y = 300*sin(x);
    p = [x y 300];
    
    %Create line segment for user forearm
    birdline = [outdata.bird1pos(1,:);outdata.bird2pos(1,:)]
    birdline_len = sqrt(sum((birdline(1,:)-birdline(2,:)).^2));
    bird_dir = (birdline(1,:)-birdline(2,:))/birdline_len;
    
    %Intersection
    %Check if robot end-effector intersects forearm
    %If intersection: Move up in z-direction
    %If no intersection: Robot continues to point p
    robo = p-outdata.bird1pos(1,:);
    robo_len = sqrt(robo.^2);
    robo_dir = robo/robo_len;
    if (robo_dir == bird_dir & robo_len<=birdline_len)
        p = [x y 400];
    end
    M = makehgtform('translate',p);
    hCyton.hDisplay.setTarget(M);
    %hCyton.hControls.goto(p);
    
    
    
end

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


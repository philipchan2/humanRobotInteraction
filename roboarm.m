cd c:\usr\myopen\MiniVIE
MiniVIE.configurePath
import Presentation.CytonI.*
%Presentation.CytonI.CytonI
hCyton=CytonI

for x=100:10:400;
    
    %Planned trajectory of robot arm
    y = 300*sin(x);
    p = [x y 300];

    %Create line segment for user forearm    
    birdline     = [outdata.bird1pos(1,:);outdata.bird2pos(1,:)]
    birdline_len = sqrt(sum((birdline(1,:)-birdline(2,:)).^2));
    bird_dir     = (birdline(1,:)-birdline(2,:))/birdline_len;
    
    %Check if robot end-effector intersects forearm
    %If intersection: Move up in z-direction
    %If no intersection: Robot continues to point p
    robo     = p-outdata.bird1pos(1,:);
    robo_len = sqrt(robo.^2));
    robo_dir = robo/robo_len;
    if (robo_dir == bird_dir)
        p = [x y 400]; 
    end
    M = makehgtform('translate',p);
    hCyton.hDisplay.setTarget(M);
    hCyton.hControls.goto(p);
   
    hCyton.T_0_EndEffector(1:3,4)
    %pause(2);
    
end

%Set birdline to a direction that will not intersect with the planned
%trajectory

%Set birdline to intersect with planned trajectory

% I've updated the Cyton VIE code based on some questions during class.  The short version is that you should control the robot using the Jacobian directly.  The setEndEffectorPose() and goto() methods are examples and don't account for joint limits and don't converge for every point in space.  
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
% Using the -Z slider you can see the robot does behave well when it is not in contact with joint limits.  However unexpected behavior results when the 'elbow' joint locks out.  Think about why this is.
% 
% I'll go over this case and what you should do about it in your implementation in the next class.
% 
% -Bobby

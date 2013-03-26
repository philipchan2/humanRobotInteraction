function [theta, isSingularElbow] = IK_3_17(p,R)
% Compute the inverse kinematics for the elbow manipulator
% p is a column vector of cartesian coordinates in the base frame
% R is optional, a rotation matrix of the end effector
% theta is a 6 element vector giving the joint angles for the configuration
% isSingularElbow is 1 if the elbow config is singular
% HW2 problem 3-17
% philip chan MAR 2013

% elbow manipulator DH convention table 3.6
% for each link; set to match the DEMO
a = [0 4 4];
alpha = [pi/2 0 0];
d = [5 0 0];
elbowOffset = 0;

%set the notation for xyz
x = p(1);
y = p(2);
z = p(3);

if isempty(R) % no orientation given
    R = eye(3); % end effector orientation is the same as the base frame
end

% equations 3-64 to 3-699
theta(1) = atan2(y, x);
D = (x.^2+y.^2-elbowOffset.^2+(z-d(1)).^2-a(2).^2 - a(3).^2)...
    ./(2*a(2).*a(3));

% check for singular configuration
epsilon = 1e-3; % small number
if sqrt(1-D.^2) < epsilon
    isSingularElbow = 1;
    theta(3) = 0;
else
    isSingularElbow = 0;
    signD = -1; % positive or negative, down or up solution
    % choose the elbow up position; less likely to hit the table
    theta(3) = atan2(signD*sqrt(1-D.^2), D);
end


theta(2) = atan2(z-d(1), sqrt(x.^2+y.^2-elbowOffset.^2)) ...
    - atan2(a(3)*sin(theta(3)), a(2)+a(3)*cos(theta(3)));

theta(4) = atan2(...
    -cos(theta(1))*sin(theta(2)+theta(3)*R(1,3)) ...
    - sin(theta(1))*sin(theta(2)+theta(3)*R(2,3)) ...
    + cos(theta(2)+theta(3))*R(3,3) , ... 
    cos(theta(1))*cos(theta(2)+theta(3))*R(1,3) ...
    + sin(theta(1))*cos(theta(2)+theta(3))*R(2,3) ...
    + sin(theta(2)+theta(3))*R(3,3) );

if sqrt(1-(sin(theta(1))*R(1,3)-cos(theta(1))*R(2,3)).^2) < epsilon
    isSingularEffector = 1;
    theta(5) = 0;
else
    isSingularEffector = 0;
    signR = 1; % orientation solutions, arbitrary
    theta(5) = atan2(...
        signR*sqrt(1-(sin(theta(1))*R(1,3)-cos(theta(1))*R(2,3)).^2), ...
        sin(theta(1))*R(1,3)-cos(theta(1))*R(2,3));
end

theta(6) = atan2(...
    sin(theta(1))*R(1,2)-cos(theta(1))*R(2,2), ...
    -sin(theta(1))*R(1,1) + cos(theta(1))*R(2,1) );










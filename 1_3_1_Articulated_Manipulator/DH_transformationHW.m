function Ai = DH_transformationHW(linkLength,linkTwist,linkOffset,jointAngle)
% compute the DH convention homogeneous transform 4x4 matrix
% inputs are in radians and consistent length units
% philip chan MAR 2013
% HW2 problem 1a

% equation 3.10
Ai = [
cos(jointAngle), -sin(jointAngle)*cos(linkTwist),  sin(jointAngle)*sin(linkTwist), linkLength*cos(jointAngle)
sin(jointAngle),  cos(jointAngle)*cos(linkTwist), -cos(jointAngle)*sin(linkTwist), linkLength*sin(jointAngle)
0, sin(linkTwist), cos(linkTwist), linkOffset
0, 0, 0, 1
];

